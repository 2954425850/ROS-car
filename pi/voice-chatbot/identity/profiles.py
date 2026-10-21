"""身份档案：一人一个 JSON，放在仓库**外面**（默认 ~/.jarvis/profiles/）。

## 为什么不放仓库里
Pi 的开发方式是「本地写 → push → Pi 上 git pull」。运行时往仓库里写文件
（哪怕是 untracked 的）早晚会撞上 pull/clean；档案又是**助手自己写**的东西
（README 的判据：脚本写的不入库）。所以放 HOME 下。

## id 与名字
- `id` 就是 K230 人脸库里的名字（板子只认 ASCII）：老的手工录入是 `people.json`
  里那批（如 `id1`），对话中自动建档的是 `u001`、`u002`……
- `name` 是中文称呼，**可以为空**（自动建档时还不知道叫什么，聊着聊着才知道）。
- 启动时把 `people.json`（id → 名字）里有、档案里没有的人补成档案，名字照抄。

## 档案内容
    {"id": "u003", "name": "张三", "created": "...", "last_seen": "...", "seen": 12,
     "facts": {"喜欢的歌手": "周杰伦", "职业": "学生"}, "auto": true}
`facts` 是对话里提取出来的（identity/memory.py），键值都是短中文。
"""

from __future__ import annotations

import json
import os
import re
import threading
import time
from pathlib import Path

from loguru import logger

_ID_RE = re.compile(r"^[A-Za-z0-9_]{1,24}$")
MAX_FACTS = 30


def _now_str() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


class ProfileStore:

    def __init__(self, root: str | os.PathLike, seed_people_json: str | None = None):
        self.root = Path(os.path.expanduser(str(root)))
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._cache: dict[str, dict] = {}
        for p in sorted(self.root.glob("*.json")):
            try:
                d = json.loads(p.read_text(encoding="utf-8"))
                if _ID_RE.match(str(d.get("id", ""))):
                    self._cache[d["id"]] = d
            except Exception as e:  # noqa: BLE001
                logger.warning(f"档案：读 {p.name} 失败，跳过 —— {e}")
        if seed_people_json:
            self._seed(seed_people_json)

    # ---- 读 ----

    def get(self, pid: str | None) -> dict | None:
        if not pid:
            return None
        with self._lock:
            d = self._cache.get(pid)
            return json.loads(json.dumps(d)) if d else None

    def all(self) -> list[dict]:
        with self._lock:
            return [json.loads(json.dumps(d)) for d in self._cache.values()]

    def find_by_name(self, name: str) -> str | None:
        """中文称呼 → id。完全相等优先，其次互相包含（「张三」/「张三同学」）。"""
        name = (name or "").strip()
        if not name:
            return None
        with self._lock:
            for pid, d in self._cache.items():
                if d.get("name") == name or pid == name:
                    return pid
            for pid, d in self._cache.items():
                n = d.get("name") or ""
                if n and (n in name or name in n):
                    return pid
        return None

    def display_name(self, pid: str | None) -> str | None:
        d = self.get(pid)
        return (d or {}).get("name") or None

    # ---- 写 ----

    def ensure(self, pid: str, *, auto: bool = False, name: str | None = None) -> dict:
        if not _ID_RE.match(pid or ""):
            raise ValueError(f"非法 id {pid!r}（板子只认 ASCII 字母数字下划线）")
        with self._lock:
            d = self._cache.get(pid)
            if d is None:
                d = {"id": pid, "name": name or "", "created": _now_str(),
                     "last_seen": None, "seen": 0, "facts": {}, "auto": auto}
                self._cache[pid] = d
                self._write(d)
            return json.loads(json.dumps(d))

    def next_auto_id(self) -> str:
        with self._lock:
            n = 0
            for pid in self._cache:
                m = re.match(r"^u(\d+)$", pid)
                if m:
                    n = max(n, int(m.group(1)))
            return "u%03d" % (n + 1)

    def touch(self, pid: str) -> None:
        with self._lock:
            d = self._cache.get(pid)
            if d is None:
                return
            d["last_seen"] = _now_str()
            d["seen"] = int(d.get("seen") or 0) + 1
            self._write(d)

    def set_name(self, pid: str, name: str) -> None:
        name = (name or "").strip()[:16]
        if not name:
            return
        with self._lock:
            d = self._cache.get(pid)
            if d is None or d.get("name") == name:
                return
            logger.info(f"档案：{pid} 的称呼 {d.get('name') or '（空）'} → {name}")
            d["name"] = name
            self._write(d)

    def add_facts(self, pid: str, facts: dict) -> None:
        if not facts:
            return
        with self._lock:
            d = self._cache.get(pid)
            if d is None:
                return
            cur = d.setdefault("facts", {})
            for k, v in facts.items():
                k = str(k).strip()[:16]
                v = str(v).strip()[:60]
                if not k or not v:
                    continue
                cur.pop(k, None)           # 重新插入 = 挪到最新
                cur[k] = v
            while len(cur) > MAX_FACTS:    # 丢最老的
                cur.pop(next(iter(cur)))
            self._write(d)

    def remove(self, pid: str) -> bool:
        with self._lock:
            d = self._cache.pop(pid, None)
            if d is None:
                return False
            try:
                (self.root / f"{pid}.json").unlink()
            except FileNotFoundError:
                pass
            return True

    # ---- 内部 ----

    def _seed(self, people_json: str) -> None:
        try:
            m = json.loads(Path(people_json).read_text(encoding="utf-8"))
        except Exception as e:  # noqa: BLE001
            logger.warning(f"档案：读 {people_json} 失败，不做种子 —— {e}")
            return
        for pid, name in (m or {}).items():
            if _ID_RE.match(str(pid)) and pid not in self._cache:
                # people.json 里"称呼就是 id 本身"（如 id1→id1）= 其实不知道叫什么，别让模型念 id
                name = str(name or "")
                self.ensure(pid, name="" if name == pid else name)

    def _write(self, d: dict) -> None:
        """**调用方持锁**。原子写：先写临时文件再 rename。"""
        p = self.root / f"{d['id']}.json"
        tmp = p.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(d, ensure_ascii=False, indent=1), encoding="utf-8")
        os.replace(tmp, p)


def profile_summary(d: dict | None, max_facts: int = 8) -> str:
    """档案 → 给模型看的一行摘要。"""
    if not d:
        return ""
    parts = []
    facts = list((d.get("facts") or {}).items())[-max_facts:]
    if facts:
        parts.append("；".join(f"{k}：{v}" for k, v in facts))
    if d.get("seen"):
        parts.append(f"见过 {d['seen']} 次")
    return "，".join(parts)
