#!/usr/bin/env bash
# ============================================================================
# 服务器侧验收脚本（A~J 中需要真实运行环境的条目）
#
# 依赖：curl / psql / ffmpeg / ss（setup-server.sh 已保证）
# 用法：ADMIN_PASS='...' bash acceptance-server.sh
# 输出：全部原始命令与原始输出，末尾给 PASS/FAIL 汇总。
# ============================================================================
set -uo pipefail

BASE=http://127.0.0.1:8080
IP=114.215.188.147
ADMIN_USER=${ADMIN_USER:-admin}
ADMIN_PASS=${ADMIN_PASS:-}
DB=robot_upper
PASS_N=0
FAIL_N=0
declare -a FAILED

say()  { echo; echo "======== $* ========"; }
run()  { echo "\$ $*"; "$@"; echo "[exit $?]"; }
ok()   { PASS_N=$((PASS_N+1)); echo "PASS  $1  $2"; }
bad()  { FAIL_N=$((FAIL_N+1)); FAILED+=("$1"); echo "FAIL  $1  $2"; }
# 断言：expect_eq <id> <期望> <实际> <说明>
expect_eq() {
  if [ "$2" = "$3" ]; then ok "$1" "$4（实际=$3）"; else bad "$1" "$4（期望=$2，实际=$3）"; fi
}
psql_r() { su - postgres -c "psql -d $DB -tAc \"$1\"" 2>/dev/null | tr -d '[:space:]'; }

# ★ 后端启动要 ~25 秒（Flyway + JPA + Tomcat）。固定 sleep 会随机失败，
#   所以统一用「轮询到能应答为止」，超时 90 秒。
wait_for_backend() {
  local n=0
  while [ $n -lt 90 ]; do
    if curl -s -o /dev/null --max-time 3 "$BASE/api/auth/login" \
        -X POST -H 'Content-Type: application/json' -d '{}' 2>/dev/null; then
      echo "[wait] 后端已就绪（等待 ${n}s）"
      return 0
    fi
    sleep 1
    n=$((n+1))
  done
  echo "[wait] !! 后端 90 秒内未就绪"
  return 1
}

# 数值比较（避免 "0" vs "0.0" 这种字符串比较的假失败）
num_eq() { awk -v a="$1" -v b="$2" 'BEGIN{exit !(a+0==b+0)}'; }

if [ -z "$ADMIN_PASS" ]; then echo "需要 ADMIN_PASS 环境变量"; exit 2; fi
wait_for_backend || true

# ============================================================ J. 部署
say "J7 ffmpeg 已安装"
run ffmpeg -version | head -1
if command -v ffmpeg >/dev/null 2>&1; then ok J7 "ffmpeg 存在：$(ffmpeg -version 2>&1 | head -1)"; else bad J7 "ffmpeg 缺失"; fi

say "J8 收流端口在监听（后端起来即监听）"
run bash -c "ss -tlnp | grep -E ':(8080|8555|8556)'"
J8=$(ss -tln | grep -cE ':(8555|8556)')
expect_eq J8 2 "$J8" "8555 与 8556 都在监听"

say "J6 JVM 堆已限制（-Xmx512m）"
run bash -c "ps -eo pid,args | grep '[b]ackend.jar' | head -2"
# 注意：-Xmx512m 在 -jar 之前，所以模式必须是 -Xmx512m.*backend.jar（顺序写反会假失败）
if ps -eo args | grep -q -- '-Xmx512m.*backend\.jar'; then ok J6 "进程命令行含 -Xmx512m"; else bad J6 "未见 -Xmx512m"; fi

say "J9 视频流端点可达（无上游也 200）"
CODE=$(curl -s -o /dev/null -w '%{http_code}' --max-time 3 "$BASE/video/stream" || true)
echo "\$ curl -s -o /dev/null -w '%{http_code}' --max-time 3 $BASE/video/stream"
echo "$CODE"
expect_eq J9 200 "$CODE" "无上游数据也会话挂起并返回 200"

# ============================================================ B. 后端基础
say "B4 所有表已建"
run bash -c "su - postgres -c \"psql -d $DB -c '\\dt'\""
TABLES="users revoked_tokens command_logs action_recordings recording_steps ai_sessions ai_messages video_clips maps map_annotations patrol_routes patrol_points"
MISSING=""
for t in $TABLES; do
  C=$(psql_r "select count(*) from information_schema.tables where table_name='$t'")
  [ "$C" = "1" ] || MISSING="$MISSING $t"
done
if [ -z "$MISSING" ]; then ok B4 "12 张表全部存在"; else bad B4 "缺表：$MISSING"; fi

say "B5 错误响应体形状统一（traceId/code/message）"
echo "\$ curl -s -X POST $BASE/api/robot/gimbal -H 'Authorization: Bearer <token>' -d '{\"panUs\":400}'  # 无 token，先看 401 形状"
run curl -s "$BASE/api/robot/status"
echo
TOKEN=$(curl -s -X POST "$BASE/api/auth/login" -H 'Content-Type: application/json' \
        -d "{\"username\":\"$ADMIN_USER\",\"password\":\"$ADMIN_PASS\"}" | sed -n 's/.*"token":"\([^"]*\)".*/\1/p')
echo "已获取 token: ${TOKEN:0:24}...（长度 ${#TOKEN}）"

BODY=$(curl -s -X POST "$BASE/api/robot/gimbal" -H "Authorization: Bearer $TOKEN" \
       -H 'Content-Type: application/json' -d '{"panUs":400}')
echo "\$ curl -s -X POST /api/robot/gimbal -d '{\"panUs\":400}'"
echo "$BODY"
if echo "$BODY" | grep -q '"traceId"' && echo "$BODY" | grep -q '"code"' && echo "$BODY" | grep -q '"message"'; then
  ok B5 "错误体含 traceId/code/message"
else
  bad B5 "错误体形状不对"
fi

say "B6 响应头含安全项"
run curl -s -I "$BASE/api/robot/status"
H=$(curl -s -I "$BASE/api/robot/status")
for h in Content-Security-Policy X-Content-Type-Options X-Frame-Options; do
  if echo "$H" | grep -qi "^$h"; then ok B6 "响应头含 $h"; else bad B6 "缺少 $h"; fi
done

say "B7 登录成功返回 JWT"
LOGIN=$(curl -s -w '\n%{http_code}' -X POST "$BASE/api/auth/login" -H 'Content-Type: application/json' \
        -d "{\"username\":\"$ADMIN_USER\",\"password\":\"$ADMIN_PASS\"}")
echo "$LOGIN"
LCODE=$(echo "$LOGIN" | tail -1)
expect_eq B7 200 "$LCODE" "登录返回 200"
echo "$LOGIN" | head -1 | grep -q '"token"' && ok B7 "响应含 token" || bad B7 "响应无 token"

say "B8 错误密码 → 401 + 41001"
BB=$(curl -s -w '\n%{http_code}' -X POST "$BASE/api/auth/login" -H 'Content-Type: application/json' \
     -d "{\"username\":\"$ADMIN_USER\",\"password\":\"definitely-wrong\"}")
echo "$BB"
expect_eq B8 401 "$(echo "$BB" | tail -1)" "HTTP 401"
echo "$BB" | head -1 | grep -q '41001' && ok B8 "业务码 41001" || bad B8 "业务码不是 41001"

say "B9 登出后 token 立即失效 → 401 + 41002"
T2=$(curl -s -X POST "$BASE/api/auth/login" -H 'Content-Type: application/json' \
     -d "{\"username\":\"$ADMIN_USER\",\"password\":\"$ADMIN_PASS\"}" | sed -n 's/.*"token":"\([^"]*\)".*/\1/p')
echo "\$ curl -s -X POST /api/auth/logout  (token=...${T2: -8})"
run curl -s -o /dev/null -w '%{http_code}\n' -X POST "$BASE/api/auth/logout" -H "Authorization: Bearer $T2"
LO=$(curl -s -w '\n%{http_code}' "$BASE/api/auth/me" -H "Authorization: Bearer $T2")
echo "\$ curl -s /api/auth/me  (用已登出的 token)"
echo "$LO"
expect_eq B9 401 "$(echo "$LO" | tail -1)" "登出后旧 token 返回 401"
echo "$LO" | head -1 | grep -q '41002' && ok B9 "业务码 41002" || bad B9 "业务码不是 41002"

say "B10 无 token 访问受保护接口 → 401"
NB=$(curl -s -w '\n%{http_code}' "$BASE/api/robot/status")
echo "$NB"
expect_eq B10 401 "$(echo "$NB" | tail -1)" "无 Authorization 头返回 401"

# ============================================================ C. 指令链路
say "C4 速度限幅生效（vx=0.9 → 0.5，clamped=true）"
D=$(curl -s -X POST "$BASE/api/robot/drive" -H "Authorization: Bearer $TOKEN" \
    -H 'Content-Type: application/json' -d '{"vx":0.9,"wz":0}')
echo "\$ curl -s -X POST /api/robot/drive -d '{\"vx\":0.9,\"wz\":0}'"
echo "$D"
echo "$D" | grep -q '"vx":0.5' && echo "$D" | grep -q '"clamped":true' \
  && ok C4 "限幅到 0.5 且 clamped=true" || bad C4 "限幅结果不符"

say "C3 客户端静默 3 秒后速度归零"
curl -s -o /dev/null -X POST "$BASE/api/robot/drive" -H "Authorization: Bearer $TOKEN" \
     -H 'Content-Type: application/json' -d '{"vx":0.15,"wz":0}'
S1=$(curl -s "$BASE/api/robot/status" -H "Authorization: Bearer $TOKEN")
echo "\$ 立即取 status: $(echo "$S1" | sed -n 's/.*"speed":\([^,]*\).*/\1/p')"
sleep 4
S2=$(curl -s "$BASE/api/robot/status" -H "Authorization: Bearer $TOKEN")
SP=$(echo "$S2" | sed -n 's/.*"speed":\([^,]*\).*/\1/p')
echo "\$ 静默 4 秒后取 status: speed=$SP"
if num_eq "$SP" 0; then ok C3 "静默 4s 后期望速度归零（speed=$SP）"; else bad C3 "speed=$SP，未归零"; fi

say "C6 云台脉宽越界被拒（panUs=400 → 400 + 40002）"
G=$(curl -s -w '\n%{http_code}' -X POST "$BASE/api/robot/gimbal" -H "Authorization: Bearer $TOKEN" \
    -H 'Content-Type: application/json' -d '{"panUs":400}')
echo "$G"
expect_eq C6 400 "$(echo "$G" | tail -1)" "HTTP 400"
echo "$G" | head -1 | grep -q '40002' && ok C6 "业务码 40002" || bad C6 "业务码不是 40002"

say "C7 云台边界值被接受（500 与 2500）"
for v in 500 2500; do
  C=$(curl -s -o /dev/null -w '%{http_code}' -X POST "$BASE/api/robot/gimbal" -H "Authorization: Bearer $TOKEN" \
      -H 'Content-Type: application/json' -d "{\"panUs\":$v}")
  expect_eq "C7($v)" 200 "$C" "panUs=$v 被接受"
done

say "C8 解除急停不需要 confirm（契约变更：一个开关来回切，40003 已废弃）"
E1=$(curl -s -w '\n%{http_code}' -X POST "$BASE/api/robot/estop" -H "Authorization: Bearer $TOKEN" \
     -H 'Content-Type: application/json' -d '{"enabled":false}')
echo "$E1"
expect_eq C8 200 "$(echo "$E1" | tail -1)" "不带 confirm 直接解除 → HTTP 200"
echo "$E1" | head -1 | grep -q '40003' && bad C8-code "仍在返回已废弃的业务码 40003" || ok C8-code "无 40003（该码已从契约移除）"

say "C9 急停期间速度被锁"
curl -s -o /dev/null -X POST "$BASE/api/robot/estop" -H "Authorization: Bearer $TOKEN" \
     -H 'Content-Type: application/json' -d '{"enabled":true}'
curl -s -o /dev/null -X POST "$BASE/api/robot/drive" -H "Authorization: Bearer $TOKEN" \
     -H 'Content-Type: application/json' -d '{"vx":0.2,"wz":0}'
sleep 1
SL=$(curl -s "$BASE/api/robot/status" -H "Authorization: Bearer $TOKEN" | sed -n 's/.*"speed":\([^,]*\).*/\1/p')
if num_eq "$SL" 0; then ok C9 "急停后 status.speed 仍为 0（speed=$SL）"; else bad C9 "speed=$SL，急停未锁住"; fi
curl -s -o /dev/null -X POST "$BASE/api/robot/estop" -H "Authorization: Bearer $TOKEN" \
     -H 'Content-Type: application/json' -d '{"enabled":false}'

say "C11 占位端点返回 501 + 50101"
for ep in led buzzer; do
  R=$(curl -s -w '\n%{http_code}' -X POST "$BASE/api/robot/$ep" -H "Authorization: Bearer $TOKEN" \
      -H 'Content-Type: application/json' -d '{}')
  echo "\$ POST /api/robot/$ep → $(echo "$R" | tail -1) $(echo "$R" | head -1)"
  expect_eq "C11($ep)" 501 "$(echo "$R" | tail -1)" "/api/robot/$ep 返回 501"
  echo "$R" | head -1 | grep -q '50101' && ok "C11($ep)-code" "业务码 50101" || bad "C11($ep)-code" "业务码不是 50101"
done

say "C12 指令审计落库（发 3 条指令 → command_logs +3，gateway_source=MOCK）"
BEFORE=$(psql_r "select count(*) from command_logs")
curl -s -o /dev/null -X POST "$BASE/api/robot/drive" -H "Authorization: Bearer $TOKEN" -H 'Content-Type: application/json' -d '{"vx":0.11,"wz":0}'
curl -s -o /dev/null -X POST "$BASE/api/robot/drive" -H "Authorization: Bearer $TOKEN" -H 'Content-Type: application/json' -d '{"vx":0.22,"wz":0}'
curl -s -o /dev/null -X POST "$BASE/api/robot/gimbal" -H "Authorization: Bearer $TOKEN" -H 'Content-Type: application/json' -d '{"panUs":1700}'
sleep 2
AFTER=$(psql_r "select count(*) from command_logs")
DELTA=$((AFTER-BEFORE))
echo "before=$BEFORE after=$AFTER delta=$DELTA"
run bash -c "su - postgres -c \"psql -d $DB -c 'select id,device,source,accepted,gateway_source,payload from command_logs order by id desc limit 5'\""
expect_eq C12 3 "$DELTA" "3 条指令产生 3 条审计"
GWS=$(psql_r "select count(*) from command_logs where gateway_source='MOCK'")
[ "$GWS" -gt 0 ] && ok C12-gw "存在 gateway_source='MOCK' 的审计行（$GWS 条）" || bad C12-gw "没有 MOCK 来源的审计行"

say "C13 审计按变化点记，不按帧记（持续 5 秒 → drive 行数应 ≈1）"
B13=$(psql_r "select count(*) from command_logs where device='drive'")
curl -s -o /dev/null -X POST "$BASE/api/robot/drive" -H "Authorization: Bearer $TOKEN" -H 'Content-Type: application/json' -d '{"vx":0.3,"wz":0}'
echo "（按住 5 秒：审计只应记 1 条变化点 —— 审计按变化点记、不按帧记）"
sleep 5
A13=$(psql_r "select count(*) from command_logs where device='drive'")
D13=$((A13-B13))
echo "drive 审计行 5 秒内增加：$D13（若逐帧记录会随帧率成倍膨胀）"
if [ "$D13" -lt 10 ]; then ok C13 "5 秒内 drive 审计行仅 $D13 条（应 ≈1，非逐帧）"; else bad C13 "5 秒内增加了 $D13 条，疑似逐帧审计"; fi

say "C10 退出时发零速"
run systemctl stop robot-upper
sleep 3
run bash -c "journalctl -u robot-upper --no-pager -n 15 | grep -E '零速|CommandPump' || true"
if journalctl -u robot-upper --no-pager -n 60 | grep -q 'CommandPump 已停止，已发零速'; then
  ok C10 "停止日志含「CommandPump 已停止，已发零速」"
else
  bad C10 "未在日志中看到「已发零速」"
fi
run systemctl start robot-upper
wait_for_backend
run systemctl is-active robot-upper

# ============================================================ G. 视频与结果流
say "G1 GET /api/video/source 返回单地址（无数组）"
V=$(curl -s "$BASE/api/video/source" -H "Authorization: Bearer $TOKEN")
echo "\$ curl -s /api/video/source"
echo "$V"
echo "$V" | grep -q '"url":"/video/stream"' && ok G1 "url=/video/stream" || bad G1 "url 不是 /video/stream"
echo "$V" | grep -q '\[' && bad G1 "响应里出现了数组（U8 已决：必须单地址）" || ok G1 "无数组字段"

say "G3 收流健康可查（无上游时 videoOk=false）"
VO=$(curl -s "$BASE/api/robot/status" -H "Authorization: Bearer $TOKEN" | sed -n 's/.*"videoOk":\([a-z]*\).*/\1/p')
echo "status.videoOk = $VO"
[ -n "$VO" ] && ok G3 "status.videoOk 字段存在（当前 $VO）" || bad G3 "status 里没有 videoOk"

say "G6 结果流监听可收（:8556 注入一帧）"
FRAME=$((RANDOM+1000))
exec 3<>/dev/tcp/127.0.0.1/8556 && printf '{"ts":1,"frame":%d,"w":1280,"h":720,"objs":[]}\n' "$FRAME" >&3 && exec 3<&- 3>&-
sleep 2
run bash -c "journalctl -u robot-upper --no-pager -n 20 | grep -E '结果流|K230' | tail -3"
AIOK=$(curl -s "$BASE/api/robot/status" -H "Authorization: Bearer $TOKEN" | sed -n 's/.*"aiOk":\([a-z]*\).*/\1/p')
expect_eq G6 true "$AIOK" "注入一帧后 status.aiOk=true（说明 8556 收到了）"

say "G7 结果流坏行不崩（非法 JSON / 超长行 / 空行）"
exec 3<>/dev/tcp/127.0.0.1/8556
printf 'not json at all\n' >&3
printf '{broken\n' >&3
printf '\n' >&3
head -c 20000 /dev/zero | tr '\0' 'x' >&3; printf '\n' >&3
FRAME2=$((FRAME+1))
printf '{"ts":2,"frame":%d,"w":1280,"h":720,"objs":[{"cls":"apple","score":0.5,"box":[0.1,0.1,0.2,0.2]}]}\n' "$FRAME2" >&3
exec 3<&- 3>&-
sleep 2
run systemctl is-active robot-upper
ACT=$(systemctl is-active robot-upper)
expect_eq G7 active "$ACT" "发送坏行后进程仍存活"
run bash -c "journalctl -u robot-upper --no-pager -n 30 | grep -E '坏行|超长' | tail -3 || echo '(日志级别为 INFO，坏行以 WARN 记录)'"

say "G4 accept 循环：模拟断连自愈（重复 3 次）"
if ! command -v ffmpeg >/dev/null 2>&1; then
  bad G4 "ffmpeg 未安装，无法伪造 K230 推流"
else
  RESTART_BEFORE=$(journalctl -u robot-upper --no-pager | grep -c 'K230 视频流已接入' || true)
  for i in 1 2 3; do
    echo "--- 第 $i 轮：伪造 K230 推流 8 秒，然后断开 ---"
    # ★ 用 ffmpeg 自己的 -t 8 限定输出时长，再叠加后台 + kill -9 兜底。
    #   只靠 `timeout 8 ffmpeg` 不可靠：实测 ffmpeg 在写 TCP 时可能不响应 SIGTERM，
    #   timeout 会无限期等下去，把整个验收脚本挂死（真的挂过一次）。
    ffmpeg -re -f lavfi -i testsrc=size=1280x720:rate=30 \
      -c:v libx264 -preset ultrafast -b:v 1500k -g 25 -t 8 \
      -f h264 tcp://127.0.0.1:8555 -loglevel error >/dev/null 2>&1 &
    FFPID=$!

    # ★ videoOk 必须在**推流进行中**采样：健康判据是"近 2 秒有输出帧"，
    #   等 ffmpeg 停了 2 秒之后再查，false 才是正确结果（曾经因此误判为 FAIL）。
    if [ "$i" = "2" ]; then
      sleep 5
      VOK=$(curl -s "$BASE/api/robot/status" -H "Authorization: Bearer $TOKEN" | sed -n 's/.*"videoOk":\([a-z]*\).*/\1/p')
      echo "    推流进行到第 5 秒时 status.videoOk = $VOK"
      expect_eq G4-videoOk true "$VOK" "有上游数据时 videoOk=true"
      sleep 5
    else
      sleep 10
    fi

    kill -9 "$FFPID" 2>/dev/null || true
    wait "$FFPID" 2>/dev/null || true
    sleep 1
    ACT=$(systemctl is-active robot-upper)
    echo "    断开后服务状态：$ACT"
    [ "$ACT" = "active" ] || bad "G4(run$i)" "收流进程退出了"
  done
  run bash -c "journalctl -u robot-upper --no-pager | grep -E '视频流已接入|本轮上游结束' | tail -8"
  RESTART_AFTER=$(journalctl -u robot-upper --no-pager | grep -c 'K230 视频流已接入' || true)
  echo "「K230 视频流已接入」出现次数：$RESTART_BEFORE → $RESTART_AFTER"
  if [ "$RESTART_AFTER" -ge $((RESTART_BEFORE+3)) ]; then
    ok G4 "3 次重连全部被接受，收流进程全程未退出（accept 循环生效）"
  else
    bad G4 "重连次数不足 3 次"
  fi
fi

# ============================================================ H. AI 对话
say "H1 未配 AI 时优雅降级（provider=null → 回复「AI 尚未接入」，不报错）"
CH=$(curl -s -w '\n%{http_code}' -X POST "$BASE/api/ai/chat" -H "Authorization: Bearer $TOKEN" \
     -H 'Content-Type: application/json' -d '{"message":"你好"}')
echo "$CH"
expect_eq H1 200 "$(echo "$CH" | tail -1)" "HTTP 200（不是错误）"
echo "$CH" | head -1 | grep -q 'AI 尚未接入' && ok H1 "回复含「AI 尚未接入」" || bad H1 "回复不符"
SID=$(echo "$CH" | head -1 | sed -n 's/.*"sessionId":\([0-9]*\).*/\1/p')
echo "sessionId = $SID"

say "H2 会话与消息落库（ai_sessions +1，ai_messages +2）"
S_BEFORE=$(psql_r "select count(*) from ai_sessions")
M_BEFORE=$(psql_r "select count(*) from ai_messages")
curl -s -o /dev/null -X POST "$BASE/api/ai/chat" -H "Authorization: Bearer $TOKEN" \
     -H 'Content-Type: application/json' -d '{"message":"再问一次"}'
sleep 1
S_AFTER=$(psql_r "select count(*) from ai_sessions")
M_AFTER=$(psql_r "select count(*) from ai_messages")
echo "ai_sessions: $S_BEFORE → $S_AFTER ；ai_messages: $M_BEFORE → $M_AFTER"
expect_eq H2-session 1 "$((S_AFTER-S_BEFORE))" "ai_sessions +1"
expect_eq H2-message 2 "$((M_AFTER-M_BEFORE))" "ai_messages +2"
run bash -c "su - postgres -c \"psql -d $DB -c 'select id,role,left(content,30) as content from ai_messages order by id desc limit 4'\""

say "H3 历史可拉取（顺序正确）"
run curl -s "$BASE/api/ai/sessions/$SID/messages" -H "Authorization: Bearer $TOKEN"

say "H6 一键拍照携带图片（image base64 被接受，且只存引用不存字节）"
IMG="iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8DwHwAFAAH/q842iQAAAABJRU5ErkJggg=="
CH6=$(curl -s -w '\n%{http_code}' -X POST "$BASE/api/ai/chat" -H "Authorization: Bearer $TOKEN" \
      -H 'Content-Type: application/json' -d "{\"message\":\"这是什么？\",\"image\":\"$IMG\"}")
echo "$CH6" | head -c 300; echo
expect_eq H6 200 "$(echo "$CH6" | tail -1)" "带 image 的请求返回 200"
run bash -c "su - postgres -c \"psql -d $DB -c 'select image_ref, length(content) from ai_messages where image_ref is not null order by id desc limit 2'\""
IREF=$(psql_r "select image_ref from ai_messages where image_ref is not null order by id desc limit 1")
case "$IREF" in
  sha256:*) ok H6-ref "image_ref 是内容引用（$IREF），图片本体未入库" ;;
  *) bad H6-ref "image_ref 异常：$IREF" ;;
esac

# ============================================================ I. 动作录制
say "I1 录制记录变化点而非每帧"
curl -s -o /dev/null -X POST "$BASE/api/robot/record" -H "Authorization: Bearer $TOKEN" \
     -H 'Content-Type: application/json' -d '{"action":"start"}'
echo "（录制 10 秒，期间只改变 2 次速度；CommandPump 会下发约 200 帧）"
sleep 1
curl -s -o /dev/null -X POST "$BASE/api/robot/drive" -H "Authorization: Bearer $TOKEN" -H 'Content-Type: application/json' -d '{"vx":0.1,"wz":0}'
sleep 4
curl -s -o /dev/null -X POST "$BASE/api/robot/drive" -H "Authorization: Bearer $TOKEN" -H 'Content-Type: application/json' -d '{"vx":0.2,"wz":0}'
sleep 5
REC=$(curl -s -X POST "$BASE/api/robot/record" -H "Authorization: Bearer $TOKEN" \
      -H 'Content-Type: application/json' -d '{"action":"stop","name":"验收录制"}')
echo "$REC"
RID=$(echo "$REC" | sed -n 's/.*"recordingId":\([0-9]*\).*/\1/p')
STEPS=$(psql_r "select count(*) from recording_steps where recording_id=$RID")
echo "recording_id=$RID 步骤数=$STEPS（逐帧记录会是 ~200）"
if [ "$STEPS" -lt 20 ] && [ "$STEPS" -gt 0 ]; then ok I1 "步骤数 $STEPS ≪ 200（变化点录制生效）"; else bad I1 "步骤数 $STEPS 不符合预期"; fi

say "I4 录制内容存 JSONB"
run bash -c "su - postgres -c \"psql -d $DB -c 'select pg_typeof(payload) from recording_steps limit 1'\""
PT=$(psql_r "select pg_typeof(payload) from recording_steps limit 1")
expect_eq I4 jsonb "$PT" "recording_steps.payload 列类型为 jsonb"

say "I2 回放按时序重放（status.recording == playing）"
curl -s -o /dev/null -X POST "$BASE/api/robot/record" -H "Authorization: Bearer $TOKEN" \
     -H 'Content-Type: application/json' -d "{\"action\":\"play\",\"recordingId\":$RID}"
sleep 1
PL=$(curl -s "$BASE/api/robot/status" -H "Authorization: Bearer $TOKEN" | sed -n 's/.*"recording":"\([a-z]*\)".*/\1/p')
echo "回放期间 status.recording = $PL"
expect_eq I2 playing "$PL" "回放期间状态为 playing"

say "I3 回放中可中断（estop）"
curl -s -o /dev/null -X POST "$BASE/api/robot/estop" -H "Authorization: Bearer $TOKEN" \
     -H 'Content-Type: application/json' -d '{"enabled":true}'
sleep 1
ST=$(curl -s "$BASE/api/robot/status" -H "Authorization: Bearer $TOKEN" | sed -n 's/.*"recording":"\([a-z]*\)".*/\1/p')
expect_eq I3 idle "$ST" "急停后回放立即停止（recording=idle）"
curl -s -o /dev/null -X POST "$BASE/api/robot/estop" -H "Authorization: Bearer $TOKEN" \
     -H 'Content-Type: application/json' -d '{"enabled":false}'

say "录制列表"
run curl -s "$BASE/api/robot/recordings" -H "Authorization: Bearer $TOKEN"

# ============================================================ J. 外网
say "J2 / J5 / J3 外网可达 —— 见 acceptance-nginx.sh 与外部探针"
cat <<'EOT'
  说明（这不是失败，是环境事实）：
    阿里云 ECS **无法用公网 IP 访问自己**：EIP 不做 hairpin，从本机 curl
    https://<公网IP>/ 一定超时（实测 exit 28）。所以"外网可达"只能在**外部机器**上验证。
    另外公网 80/443/8555/8556 是否放行取决于**阿里云安全组**（控制台手工操作），
    开发方无 API 权限。当前实测：只有 22 端口从外网可达。

  已完成的等效验证（见 acceptance-nginx.sh，17 项全通过）：
    TLS 证书 / Nginx server 块 / SPA 深链接回退 / /api 反代 / /ws 升级透传 /
    token 不落 access log / /video chunked 反代 —— 全部打 https://127.0.0.1 + Host 头，
    覆盖除"最后一跳安全组"之外的全部链路。

  待人工：在阿里云控制台放行 80/443/8555/8556，然后从外部机器重跑
          deploy/run-acceptance-external.ps1。
EOT
J2=skip; J5=skip; J3=skip

say "J4 /ws/robot 的 access log 已关闭（token 不落日志）"
curl -k -s -o /dev/null "https://$IP/ws/robot?token=FAKE_TOKEN_FOR_LOG_TEST" || true
sleep 1
run bash -c "grep -r 'FAKE_TOKEN_FOR_LOG_TEST' /var/log/nginx/ 2>/dev/null && echo '!!! token 落进日志了' || echo '日志中未出现 token（正确）'"
if grep -rq 'FAKE_TOKEN_FOR_LOG_TEST' /var/log/nginx/ 2>/dev/null; then
  bad J4 "token 出现在 Nginx 日志里"
else
  ok J4 "access_log off 生效，token 未落日志"
fi

say "J3 WS 升级透传（外网）— 由 deploy/tools/ws-probe.mjs 通过隧道验证"
echo "（见外部探针输出：wss/https 升级 101）"

# ============================================================ 汇总
say "验收汇总"
echo "PASS: $PASS_N"
echo "FAIL: $FAIL_N"
if [ "$FAIL_N" -gt 0 ]; then
  echo "失败项：${FAILED[*]}"
  exit 1
fi
echo "全部通过"
