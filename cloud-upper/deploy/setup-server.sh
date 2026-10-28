#!/usr/bin/env bash
# ============================================================================
# 机器人上位机 · 云服务器一次性初始化（幂等，可重复执行）
#
# 做的事：装 ffmpeg、建系统用户与目录、建库角色、生成密钥、自签证书、
#         装 systemd 单元与 Nginx 站点、放行 8555/8556。
#
# 不做的事：上传 jar / 前端 dist（由 deploy/deploy.ps1 在本地做完再调本脚本）。
#
# 用法（在服务器上，root）：
#   bash /opt/robot-upper/deploy/setup-server.sh
#
# 幂等性：重复执行不会覆盖已生成的密钥/证书，也不会重建数据库。
# ============================================================================
set -euo pipefail

APP_USER=robot-upper
APP_DIR=/opt/robot-upper
CONF_DIR=/etc/robot-upper
LOG_DIR=/var/log/robot-upper
WEB_DIR=/var/www/robot-upper
ENV_FILE="$CONF_DIR/backend.env"
DB_NAME=robot_upper
DB_USER=robot

log() { echo "[setup] $*"; }

# ---------------------------------------------------------------- 1. ffmpeg
if ! command -v ffmpeg >/dev/null 2>&1; then
  log "安装 ffmpeg（验收 J7 的硬要求：视频链路靠它封装）"
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -qq
  apt-get install -y -qq ffmpeg
else
  log "ffmpeg 已安装：$(ffmpeg -version 2>&1 | head -1)"
fi

# ---------------------------------------------------------------- 2. 用户与目录
if ! id -u "$APP_USER" >/dev/null 2>&1; then
  log "创建系统用户 $APP_USER（无登录 shell）"
  useradd --system --no-create-home --shell /usr/sbin/nologin "$APP_USER"
fi
mkdir -p "$APP_DIR" "$CONF_DIR" "$LOG_DIR" "$WEB_DIR" /etc/nginx/ssl
chown -R "$APP_USER:$APP_USER" "$LOG_DIR"
chmod 750 "$CONF_DIR"
log "目录就绪：$APP_DIR $CONF_DIR $LOG_DIR $WEB_DIR"

# ---------------------------------------------------------------- 3. 数据库
# 幂等：角色存在则只重置密码为当前 env 文件里的值
if [ -f "$ENV_FILE" ]; then
  DB_PASSWORD="$(grep -E '^DB_PASSWORD=' "$ENV_FILE" | head -1 | cut -d= -f2-)"
fi
if [ -z "${DB_PASSWORD:-}" ]; then
  DB_PASSWORD="$(openssl rand -base64 24 | tr -d '/+=' | cut -c1-24)"
  log "生成新的数据库密码"
fi

ROLE_EXISTS=$(su - postgres -c "psql -tAc \"select 1 from pg_roles where rolname='$DB_USER'\"" || true)
if [ "$ROLE_EXISTS" != "1" ]; then
  log "创建数据库角色 $DB_USER"
  su - postgres -c "psql -qc \"CREATE ROLE $DB_USER LOGIN PASSWORD '$DB_PASSWORD'\""
else
  log "角色 $DB_USER 已存在，同步密码"
  su - postgres -c "psql -qc \"ALTER ROLE $DB_USER LOGIN PASSWORD '$DB_PASSWORD'\""
fi

DB_EXISTS=$(su - postgres -c "psql -tAc \"select 1 from pg_database where datname='$DB_NAME'\"" || true)
if [ "$DB_EXISTS" != "1" ]; then
  log "创建数据库 $DB_NAME（owner=$DB_USER）"
  su - postgres -c "psql -qc \"CREATE DATABASE $DB_NAME OWNER $DB_USER ENCODING 'UTF8'\""
else
  log "数据库 $DB_NAME 已存在，确保 owner 为 $DB_USER"
  su - postgres -c "psql -qc \"ALTER DATABASE $DB_NAME OWNER TO $DB_USER\""
fi
# Flyway 要在 public schema 建表 —— PG 15+ 默认收紧了 public 权限，这里显式授权
su - postgres -c "psql -d $DB_NAME -qc \"GRANT ALL ON SCHEMA public TO $DB_USER\"" || true

# ★ 遗留 schema 守卫：2026-09-16 那版早期演示实现的表与本项目设计文档 §6.2 的 DDL
#   不兼容（ai_conversations / patrol_route_points vs ai_sessions / patrol_points），
#   而且 V1 校验和不同 —— 直接启动会以 "permission denied" 或
#   "Flyway validate failed: checksum mismatch" 告终，且报错信息看不出真正原因。
#   这里提前识别并给出明确处置命令。
LEGACY=$(su - postgres -c "psql -d $DB_NAME -tAc \"select count(*) from pg_tables where schemaname='public' and tablename in ('ai_conversations','patrol_route_points')\"" 2>/dev/null | tr -d '[:space:]')
if [ "${LEGACY:-0}" != "0" ]; then
  log "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
  log "检测到【旧演示实现】遗留的 schema（ai_conversations / patrol_route_points）。"
  log "它与本项目设计文档 §6.2 的 DDL 不兼容，后端会启动失败。"
  log "请先执行一次性清理（会自动 pg_dump 备份到 /root/）："
  log "    bash $APP_DIR/deploy/reset-legacy-schema.sh"
  log "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
fi

# ---------------------------------------------------------------- 4. 密钥
# JWT_SECRET 只在首次生成；已存在则保留（换掉会让所有已登录 token 失效）
if [ -f "$ENV_FILE" ] && grep -qE '^JWT_SECRET=.+' "$ENV_FILE"; then
  log "JWT_SECRET 已存在，保留"
  JWT_SECRET="$(grep -E '^JWT_SECRET=' "$ENV_FILE" | head -1 | cut -d= -f2-)"
else
  JWT_SECRET="$(openssl rand -base64 48 | tr -d '\n')"
  log "生成新的 JWT_SECRET（48 字节随机）"
fi

# 初始管理员哈希：已存在则保留
if [ -f "$ENV_FILE" ] && grep -qE '^ADMIN_PASSWORD_HASH=.+' "$ENV_FILE"; then
  ADMIN_HASH="$(grep -E '^ADMIN_PASSWORD_HASH=' "$ENV_FILE" | head -1 | cut -d= -f2-)"
  log "ADMIN_PASSWORD_HASH 已存在，保留"
else
  ADMIN_HASH="${ADMIN_PASSWORD_HASH_IN:-}"
  if [ -z "$ADMIN_HASH" ]; then
    log "!! 未提供 ADMIN_PASSWORD_HASH_IN，且 env 文件里没有 —— 迁移将不创建管理员账号"
    log "!! 请用 ADMIN_PASSWORD_HASH_IN=<bcrypt> bash $0 重新执行"
  fi
fi

cat > "$ENV_FILE" <<EOF
# 机器人上位机后端环境变量（由 setup-server.sh 生成）
# ★ 本文件含机密，权限 600，不要进版本库。
DB_URL=jdbc:postgresql://localhost:5432/$DB_NAME
DB_USERNAME=$DB_USER
DB_PASSWORD=$DB_PASSWORD
DB_POOL_SIZE=10
DB_POOL_MIN_IDLE=2

JWT_SECRET=$JWT_SECRET
JWT_TTL_MINUTES=60

ADMIN_USERNAME=${ADMIN_USERNAME_IN:-admin}
ADMIN_PASSWORD_HASH=${ADMIN_HASH:-}

# ---- 机器人网关 ----
# MOCK：无真机也能全链路自测（隧道未部署时用这个）
# LIVE：连树莓派 rosbridge，需要 frpc 隧道已通
APP_ROBOT_GATEWAY=${APP_ROBOT_GATEWAY_IN:-MOCK}
APP_ROBOT_BRIDGE_URL=ws://127.0.0.1:19090
APP_ROBOT_RECONNECT_SECONDS=30
# 上限频率：指令"变化即发"时的封顶，抬高可降低摇杆量化延迟
APP_ROBOT_PUMP_MAX_HZ=50
APP_ROBOT_CLIENT_TIMEOUT_MS=3000

APP_ROBOT_LIMITS_MAX_VX=0.5
APP_ROBOT_LIMITS_MAX_WZ=1.5
APP_ROBOT_LIMITS_GIMBAL_MIN_US=500
APP_ROBOT_LIMITS_GIMBAL_MAX_US=2500
APP_ROBOT_LIMITS_OVER_LIMIT_POLICY=CLAMP

# ---- 视频链路（K230 出站直连，云端只监听）----
APP_VIDEO_H264_LISTEN=0.0.0.0:8555
APP_VIDEO_URL=/video/stream
APP_VIDEO_KIND=mpegts
APP_VIDEO_CODEC=
APP_VIDEO_WIDTH=1280
APP_VIDEO_HEIGHT=720
APP_VIDEO_FPS=30
APP_VIDEO_BITRATE_KBPS=2000
APP_FFMPEG_PATH=

# ---- 结构化结果流（K230 出站直连 8556）----
APP_RESULTS_LISTEN=0.0.0.0:8556
APP_RESULTS_MAX_HZ=10
APP_RESULTS_MAX_LINE_BYTES=8192

# ---- AI（U9：默认未接入）----
APP_AI_PROVIDER=null
APP_AI_API_KEY=
APP_AI_BASE_URL=
APP_AI_MODEL=
APP_AI_HISTORY_ROUNDS=20

# ---- 传输层安全（空 = 只允许同源）----
APP_CORS_ALLOWED_ORIGINS=

# ---- 审计 ----
APP_AUDIT_ENABLED=true
APP_AUDIT_QUEUE_CAPACITY=2000

SERVER_PORT=8080
LOG_FILE=$LOG_DIR/backend.log
EOF

chmod 600 "$ENV_FILE"
chown root:root "$ENV_FILE"
log "已写入 $ENV_FILE（600）"

# ---------------------------------------------------------------- 5. 自签证书
if [ ! -f /etc/nginx/ssl/robot-upper.crt ]; then
  PUBLIC_IP="$(curl -s --max-time 5 ifconfig.me || echo 114.215.188.147)"
  log "生成自签证书（CN=$PUBLIC_IP，10 年）"
  openssl req -x509 -nodes -days 3650 -newkey rsa:2048 \
    -keyout /etc/nginx/ssl/robot-upper.key \
    -out /etc/nginx/ssl/robot-upper.crt \
    -subj "/C=CN/O=RobotUpper/CN=$PUBLIC_IP" \
    -addext "subjectAltName=IP:$PUBLIC_IP" >/dev/null 2>&1
  chmod 600 /etc/nginx/ssl/robot-upper.key
else
  log "自签证书已存在，保留"
fi

# ---------------------------------------------------------------- 6. systemd
if [ -f "$APP_DIR/deploy/robot-upper.service" ]; then
  install -m 644 "$APP_DIR/deploy/robot-upper.service" /etc/systemd/system/robot-upper.service
  systemctl daemon-reload
  systemctl enable robot-upper >/dev/null 2>&1 || true
  log "systemd 单元已安装并 enable"
else
  log "!! 未找到 $APP_DIR/deploy/robot-upper.service，跳过 systemd 安装"
fi

# ---------------------------------------------------------------- 7. Nginx
if [ -f "$APP_DIR/deploy/nginx-robot-upper.conf" ]; then
  install -m 644 "$APP_DIR/deploy/nginx-robot-upper.conf" /etc/nginx/sites-available/robot-upper
  ln -sf /etc/nginx/sites-available/robot-upper /etc/nginx/sites-enabled/robot-upper
  # 旧的 default 站点会和我们的 default_server 冲突
  rm -f /etc/nginx/sites-enabled/default
  if nginx -t 2>&1 | tail -2; then
    # ★ 用 restart 而不是 reload：本次改动引入了**新的 listen 地址**（0.0.0.0:80/443，
    #   原来是 127.0.0.1:80）。reload 时旧 worker 还占着旧地址，新配置会以
    #   `bind() to 0.0.0.0:80 failed (98: Unknown error)` 失败，而且 nginx 不会报错退出 ——
    #   表现为"配置改了但外网还是连不上"，非常难查。restart 才是可靠做法。
    systemctl restart nginx
    sleep 2
    systemctl is-active nginx
    log "Nginx 已重启并生效"
    ss -tlnp | grep -E ':(80|443)' || log "!! 80/443 未在监听，请检查"
  else
    log "!! nginx -t 失败，请检查 /etc/nginx/sites-available/robot-upper"
    exit 1
  fi
else
  log "!! 未找到 $APP_DIR/deploy/nginx-robot-upper.conf，跳过 Nginx 安装"
fi

# ---------------------------------------------------------------- 8. 防火墙
# 8555/8556 是 K230 出站直连的收流端口，必须放行。
# ⚠️ D14：这两个端口无鉴权，理想做法是只放行机器人侧出口 IP；
#    这里先全网放行以便联调，拿到出口 IP 后按下行注释收紧。
ufw allow 8555/tcp comment 'K230 H.264 ingest' >/dev/null 2>&1 || true
ufw allow 8556/tcp comment 'K230 AI results ingest' >/dev/null 2>&1 || true
# 收紧示例（把 <ROBOT_EGRESS_IP> 换成机器人侧出口 IP）：
#   ufw delete allow 8555/tcp && ufw allow from <ROBOT_EGRESS_IP> to any port 8555 proto tcp
log "ufw: 已放行 8555/8556（阿里云安全组需另行放行，见部署手册 §6）"

# ---------------------------------------------------------------- 9. 权限收尾
chown -R "$APP_USER:$APP_USER" "$APP_DIR" 2>/dev/null || true
chown -R www-data:www-data "$WEB_DIR"
log "初始化完成"
log "下一步：本地执行 deploy/deploy.ps1 上传 jar 与前端 dist，然后 systemctl restart robot-upper"
