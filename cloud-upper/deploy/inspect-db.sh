#!/usr/bin/env bash
# 检查 robot_upper 库里现有的对象与 Flyway 历史（判断能否直接接管）
DB=robot_upper
echo "=== 表清单与属主 ==="
su - postgres -c "psql -d $DB -c \"select tablename, tableowner from pg_tables where schemaname='public' order by tablename\""
echo
echo "=== flyway_schema_history 内容 ==="
su - postgres -c "psql -d $DB -c 'select installed_rank, version, description, type, script, checksum, success from flyway_schema_history order by installed_rank'" 2>&1
echo
echo "=== 行数抽样（判断是不是有价值的正式数据）==="
for t in users command_logs action_recordings recording_steps ai_sessions ai_messages video_clips maps; do
  C=$(su - postgres -c "psql -d $DB -tAc 'select count(*) from $t'" 2>/dev/null | tr -d '[:space:]')
  echo "  $t = ${C:-（表不存在）}"
done
echo
echo "=== users 表结构 ==="
su - postgres -c "psql -d $DB -c '\\d users'" 2>&1 | head -20
echo
echo "=== 角色 ==="
su - postgres -c "psql -tAc '\\du'" 2>&1
