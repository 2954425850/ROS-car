#!/usr/bin/env bash
# ============================================================================
# J 组验收：Nginx 反代 / SPA 托管 / WS 升级 / token 不落日志
#
# ⚠️ 为什么打 127.0.0.1 而不是公网 IP：
#    阿里云 ECS **无法用公网 IP 访问自己**（EIP 不走环回，hairpin 不通），
#    所以"外网可达"这条只能从外部机器验证 —— 见 deploy/run-acceptance-external.ps1。
#    本脚本验证的是**除最后一跳（安全组）之外的全部链路**：
#    TLS 证书 → Nginx server 块 → SPA 回退 → /api 反代 → 后端。
#    带 Host 头是为了让 Nginx 走与公网一致的分支。
#
# 用法：bash acceptance-nginx.sh [公网IP]
# ============================================================================
set -uo pipefail

IP=${1:-114.215.188.147}
H="Host: $IP"
PASS_N=0; FAIL_N=0; declare -a FAILED
ok()  { PASS_N=$((PASS_N+1)); echo "PASS  $1  $2"; }
bad() { FAIL_N=$((FAIL_N+1)); FAILED+=("$1"); echo "FAIL  $1  $2"; }
expect() { if [ "$2" = "$3" ]; then ok "$1" "$4（实际=$3）"; else bad "$1" "$4（期望=$2，实际=$3）"; fi; }

say() { echo; echo "======== $* ========"; }

say "J7' TLS 生效（自签证书）"
echo "\$ openssl s_client -connect 127.0.0.1:443 -servername $IP"
echo | timeout 8 openssl s_client -connect 127.0.0.1:443 -servername "$IP" 2>/dev/null \
  | openssl x509 -noout -subject -issuer -dates -ext subjectAltName 2>/dev/null | head -8
CN=$(echo | timeout 8 openssl s_client -connect 127.0.0.1:443 -servername "$IP" 2>/dev/null | openssl x509 -noout -subject 2>/dev/null)
echo "$CN" | grep -q "$IP" && ok "TLS" "证书 CN 含 $IP" || bad "TLS" "证书 CN 不含 $IP：$CN"

say "J2 Nginx 反代 /api → 后端（HTTPS + Host 头）"
R=$(curl -k -s -w '\n%{http_code}' -X POST "https://127.0.0.1/api/auth/login" \
     -H "$H" -H 'Content-Type: application/json' \
     -d '{"username":"'"${ADMIN_USER:-admin}"'","password":"'"${ADMIN_PASS:-}"'"}')
echo "\$ curl -k -X POST https://127.0.0.1/api/auth/login -H '$H'"
echo "$R" | head -c 400; echo
expect J2 200 "$(echo "$R" | tail -1)" "经 Nginx 的 HTTPS 登录可达"
TOKEN=$(echo "$R" | head -1 | sed -n 's/.*"token":"\([^"]*\)".*/\1/p')
[ -n "$TOKEN" ] && ok J2-token "拿到 token（长度 ${#TOKEN}）" || bad J2-token "响应里没有 token"

say "J2' 错误密码经 Nginx 也返回统一错误体"
R2=$(curl -k -s -w '\n%{http_code}' -X POST "https://127.0.0.1/api/auth/login" \
      -H "$H" -H 'Content-Type: application/json' \
      -d '{"username":"admin","password":"wrong-password"}')
echo "$R2"
expect J2-bad 401 "$(echo "$R2" | tail -1)" "错误密码 → 401"
echo "$R2" | head -1 | grep -q '"traceId"' && ok J2-shape "错误体含 traceId" || bad J2-shape "错误体缺 traceId"

say "J5 前端静态托管（/ 与深链接都 200）"
for path in / /login /move /map /radar /ai; do
  C=$(curl -k -s -o /dev/null -w '%{http_code}' "https://127.0.0.1$path" -H "$H")
  expect "J5($path)" 200 "$C" "$path 返回 200"
done
CT=$(curl -k -s -o /dev/null -w '%{content_type}' "https://127.0.0.1/" -H "$H")
echo "       / 的 Content-Type = $CT"
echo "$CT" | grep -q 'text/html' && ok J5-html "/ 返回 HTML" || bad J5-html "/ 不是 HTML：$CT"

say "J5' 前端资源确实被托管（assets 可取）"
ASSET=$(curl -k -s "https://127.0.0.1/" -H "$H" | grep -oE '/assets/[a-zA-Z0-9._-]+\.js' | head -1)
echo "      index.html 引用的脚本：$ASSET"
if [ -n "$ASSET" ]; then
  C=$(curl -k -s -o /dev/null -w '%{http_code}' "https://127.0.0.1$ASSET" -H "$H")
  expect J5-asset 200 "$C" "静态资源 $ASSET 可取"
else
  bad J5-asset "index.html 里找不到 /assets/*.js 引用"
fi

say "J3 WS 升级透传（HTTPS → Nginx → 后端）"
# 不带 token 应当被后端拒绝（401），但**必须已经穿过 Nginx 的 upgrade 通道**
u=$(curl -k -s -o /dev/null -w '%{http_code}' \
    -H "$H" -H 'Connection: Upgrade' -H 'Upgrade: websocket' \
    -H 'Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==' -H 'Sec-WebSocket-Version: 13' \
    "https://127.0.0.1/ws/robot")
echo "\$ curl 无 token 的 WS 升级请求 → $u"
expect J3-noperm 401 "$u" "无 token 的升级请求被后端拒绝（说明请求已到达后端）"
if [ -n "$TOKEN" ]; then
  echo "      带 token 的升级由 deploy/tools/ws-probe.mjs 验证（Node 手写 WS 帧）"
fi

say "J4 /ws/robot access_log 已关闭（token 不落日志）"
MARK="FAKE_TOKEN_$(date +%s)"
curl -k -s -o /dev/null "https://127.0.0.1/ws/robot?token=$MARK" -H "$H" || true
sleep 1
if grep -rq "$MARK" /var/log/nginx/ 2>/dev/null; then
  bad J4 "token 出现在 Nginx 日志里"
else
  ok J4 "access_log off 生效，测试 token 未落日志（标记 $MARK）"
fi

say "视频反代（/video/ 缓冲已关）"
echo "\$ curl -k -s -o /dev/null -w '%{http_code}' --max-time 3 https://127.0.0.1/video/stream -H '$H'"
VC=$(curl -k -s -o /dev/null -w '%{http_code}' --max-time 3 "https://127.0.0.1/video/stream" -H "$H" || true)
expect J9-nginx 200 "$VC" "经 Nginx 的视频会话返回 200（无上游也挂起）"

say "汇总"
echo "PASS: $PASS_N   FAIL: $FAIL_N"
if [ "$FAIL_N" -gt 0 ]; then echo "失败项：${FAILED[*]}"; exit 1; fi
echo "全部通过"
