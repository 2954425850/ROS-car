#!/usr/bin/env bash
# ============================================================================
# 清理「2026-09-16 旧演示后端」遗留的 schema
#
# 背景：这台服务器上曾部署过一版**早期演示实现**，它的 schema 与本项目设计文档
#       §6.2 的 DDL 不兼容：
#         旧: ai_conversations / patrol_route_points, users.password_hash VARCHAR(255)
#         新: ai_sessions / patrol_points,       users.password_hash VARCHAR(100)
#       而且它的 V1 校验和与我们的 V1 不同 —— Flyway 会直接 validate 失败。
#
# 因此必须先备份、再清空 public schema，让 Flyway 从零建新表。
#
# ⚠️ 会删除 robot_upper 库里的全部表。执行前会自动 pg_dump 备份到 /root/。
# 用法：bash reset-legacy-schema.sh
# ============================================================================
set -euo pipefail

DB=robot_upper
DB_USER=robot
STAMP=$(date +%Y%m%d-%H%M%S)
BACKUP="/root/${DB}-backup-${STAMP}.sql"

echo "[reset] 1/4 备份 $DB -> $BACKUP"
su - postgres -c "pg_dump $DB" > "$BACKUP"
echo "[reset]     备份大小：$(du -h "$BACKUP" | cut -f1)"

echo "[reset] 2/4 清空 public schema"
su - postgres -c "psql -d $DB -qc 'DROP SCHEMA public CASCADE'"
su - postgres -c "psql -d $DB -qc 'CREATE SCHEMA public'"
su - postgres -c "psql -d $DB -qc \"ALTER SCHEMA public OWNER TO $DB_USER\""
su - postgres -c "psql -d $DB -qc \"GRANT ALL ON SCHEMA public TO $DB_USER\""

echo "[reset] 3/4 确认清空"
su - postgres -c "psql -d $DB -c \"select count(*) as remaining_tables from pg_tables where schemaname='public'\""

echo "[reset] 4/4 重启后端，让 Flyway 建新 schema"
systemctl restart robot-upper
sleep 25
systemctl is-active robot-upper
echo
echo "[reset] 迁移结果："
su - postgres -c "psql -d $DB -c 'select installed_rank, version, description, checksum, success from flyway_schema_history order by installed_rank'" 2>&1 | head -12
echo
echo "[reset] 表清单（属主应为 $DB_USER）："
su - postgres -c "psql -d $DB -c \"select tablename, tableowner from pg_tables where schemaname='public' order by tablename\""
echo
echo "[reset] 初始管理员："
su - postgres -c "psql -d $DB -c 'select id, username, role, enabled, length(password_hash) as hash_len from users'"
echo
echo "[reset] 备份保留在 $BACKUP（确认无误后可删）"
