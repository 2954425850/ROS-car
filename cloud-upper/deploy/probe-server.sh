#!/usr/bin/env bash
# Server readiness probe (read-only). ASCII-only on purpose: avoids
# PowerShell -> ssh stdin encoding pitfalls on Windows.
echo "=== HOST ==="
hostname; uname -sr; uptime

echo
echo "=== JAVA (backend needs JRE 17 only) ==="
java -version 2>&1 | head -3
readlink -f "$(command -v java)"

echo
echo "=== FFMPEG (acceptance J7) ==="
if command -v ffmpeg >/dev/null 2>&1; then
  ffmpeg -version 2>&1 | head -2
  echo "path: $(command -v ffmpeg)"
else
  echo "MISSING -- needs: apt install -y ffmpeg"
fi

echo
echo "=== POSTGRESQL ==="
psql --version
su - postgres -c "psql -tAc 'select version();'" 2>&1 | head -1
echo "--- databases ---"
su - postgres -c "psql -tAc '\l'" 2>&1 | head -10
echo "--- roles ---"
su - postgres -c "psql -tAc '\du'" 2>&1 | head -10

echo
echo "=== NGINX ==="
nginx -v 2>&1
echo "--- sites-enabled ---"
ls -1 /etc/nginx/sites-enabled/ 2>&1
echo "--- existing robot-upper config ---"
cat /etc/nginx/sites-enabled/robot-upper 2>&1 | head -80

echo
echo "=== LISTENING PORTS (8080/8555/8556/80/443/7000/19090) ==="
ss -tlnp 2>/dev/null | grep -E ':(8080|8555|8556|80|443|7000|19090)' || echo "(none)"

echo
echo "=== UFW ==="
ufw status 2>&1 | head -25

echo
echo "=== FRPS ==="
if command -v frps >/dev/null 2>&1; then command -v frps; else echo "frps not installed"; fi

echo
echo "=== TARGET DIRS ==="
for d in /opt /var/www /etc/robot-upper /var/log/robot-upper; do
  if [ -e "$d" ]; then echo "$d EXISTS"; else echo "$d MISSING"; fi
done

echo
echo "=== DISK / MEM ==="
df -h / | tail -1
free -m | head -2

echo
echo "=== EXISTING JAVA PROCESSES ==="
ps -eo pid,etime,rss,cmd | grep -E '[j]ava' | head -5 || echo "(no java process)"
