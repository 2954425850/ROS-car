"""身份 v1 的编排：唤醒 → 认人 → 给模型的身份上下文 → 对话后记忆 / 自动建档 / 留言。

ConversationManager 只调这里的五个入口：
    on_wake()                     唤醒那一刻（记时间，往回看人脸）
    begin_turn() -> str           每轮 THINKING 开头：认人，返回要拼进 system 的身份段落
    end_turn(user_text, reply)    每轮说完：后台提取记忆
    speaker_changed() -> bool     这一轮的人和上一轮认出的人不同（调用方据此清历史）
    current_pid / resolve_name()  给工具用（"跟着我"、给某人留言）

## 自动建档（用户定：不做"记住我"，对话中主动建）
`guess_speaker` 判成 `stranger`（单脸、活体、清楚、库里没有）⇒ 后台调
`k230ctl enroll uNNN`（板子采 8 帧入库）。成功就建一个空档案并把这一轮的人设成它，
名字等后台记忆提取从对话里拿。限流：同一段时间只录一次（`enroll_cooldown_s`）。
`unsure`（模糊分数）**不建** —— 那多半是库里某人换了角度，建了就是一人两份。

## 留言（身份绑定任务 v1）
"告诉张三……" ⇒ `leave_message` 工具存进 ~/.jarvis/messages.json；
下次认出张三在跟助手说话时，把留言塞进身份段落让模型先转告，转告后删除。
（主动看到人就开口要等以后有"主动说话"的通路，v1 只在他唤醒时转告。）
"""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path

from loguru import logger

from identity import memory
from identity.profiles import ProfileStore, profile_summary
from identity.speaker import SpeakerConfig, SpeakerGuess, guess_speaker

LOOKBACK_S = 2.0          # 唤醒前往回看多久的人脸


class MessageBox:
    """留言：{to_pid: [{"from": 名字或空, "text": .., "t": ..}, ...]}，原子写 JSON。"""

    def __init__(self, path: str | os.PathLike):
        self.path = Path(os.path.expanduser(str(path)))
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        try:
            self._data = json.loads(self.path.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001
            self._data = {}

    def add(self, to_pid: str, text: str, sender: str | None) -> None:
        with self._lock:
            self._data.setdefault(to_pid, []).append(
                {"from": sender or "", "text": text.strip()[:200],
                 "t": time.strftime("%m-%d %H:%M")})
            self._write()

    def peek(self, pid: str) -> list[dict]:
        with self._lock:
            return list(self._data.get(pid) or [])

    def clear(self, pid: str) -> None:
        with self._lock:
            if self._data.pop(pid, None) is not None:
                self._write()

    def _write(self) -> None:
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self._data, ensure_ascii=False, indent=1), encoding="utf-8")
        os.replace(tmp, self.path)


class IdentityManager:

    def __init__(self, config, llm, state, target, *, people_json: str | None = None):
        get = config.get
        self._llm = llm
        self._state = state                      # vision.k230_state.K230State（可为 None）
        self._target = target                    # vision.k230_follow.K230Target（可为 None）
        root = get("identity.dir", "~/.jarvis")
        self.profiles = ProfileStore(os.path.join(os.path.expanduser(root), "profiles"),
                                     seed_people_json=people_json)
        self.messages = MessageBox(os.path.join(os.path.expanduser(root), "messages.json"))
        self._cfg = SpeakerConfig(accept=float(get("identity.accept", 0.85)))
        self._auto_enroll = bool(get("identity.auto_enroll", True))
        self._cooldown = float(get("identity.enroll_cooldown_s", 120.0))
        self._remember = bool(get("identity.remember", True))
        self.owners = set(get("identity.owners", []) or [])

        self._wake_t = time.monotonic()
        self._last_enroll_t = -1e9
        self._enrolling = False
        self._lock = threading.Lock()
        self.current_pid: str | None = None      # 这一轮认定的说话人
        self.last_guess: SpeakerGuess | None = None
        self._prev_known: str | None = None
        self._changed = False
        self._delivered_to: str | None = None    # 本轮身份段落里塞了谁的留言

    # ---- 入口 ----

    def on_wake(self) -> None:
        self._wake_t = time.monotonic()

    def begin_turn(self) -> str:
        """认人 + 返回身份段落。任何异常都降级成"看不清是谁"。"""
        try:
            return self._begin_turn()
        except Exception as e:  # noqa: BLE001
            logger.error(f"身份：认人出错，按看不清处理 —— {e}")
            with self._lock:
                self.current_pid = None
            return ""

    def end_turn(self, user_text: str, reply: str) -> None:
        pid = self.current_pid
        if self._delivered_to and self._delivered_to == pid and reply.strip():
            self.messages.clear(pid)
            logger.info(f"身份：给 {pid} 的留言已转告，清除")
        self._delivered_to = None
        if not (self._remember and pid and user_text.strip()):
            return
        threading.Thread(target=self._remember_turn, args=(pid, user_text, reply),
                         name="identity-memory", daemon=True).start()

    def speaker_changed(self) -> bool:
        return self._changed

    def resolve_name(self, name: str | None) -> str | None:
        """用户说的人名/代词 → 板子 id。"我""我自己" ⇒ 当前说话人。"""
        n = (name or "").strip()
        if n in ("", "我", "我自己", "自己", "本人"):
            return self.current_pid
        return self.profiles.find_by_name(n)

    def is_owner(self, pid: str | None = None) -> bool:
        pid = self.current_pid if pid is None else pid
        return bool(pid) and pid in self.owners

    def forget(self, pid: str) -> str:
        """删档案 + 删板子人脸库 + 清留言。返回给模型的说明。"""
        name = self.profiles.display_name(pid) or pid
        self.profiles.remove(pid)
        self.messages.clear(pid)
        board = ""
        if self._target is not None:
            try:
                self._target.forget(pid)
            except Exception as e:  # noqa: BLE001
                board = f"（但板子上的人脸没删掉：{e}）"
        with self._lock:
            if self.current_pid == pid:
                self.current_pid = None
        logger.info(f"身份：已忘记 {pid}（{name}）{board}")
        return f"已经把 {name} 的档案和人脸都删了{board}"

    # ---- 内部 ----

    def _begin_turn(self) -> str:
        if self._state is None or not self._state.enabled:
            return ""
        if not self._state.fresh():
            with self._lock:
                self.current_pid = None
            return "【在场的人】摄像头暂时没有数据，不知道是谁在说话；不要猜对方名字。"
        g = guess_speaker(self._state.window.faces_since(self._wake_t - LOOKBACK_S), self._cfg)
        self.last_guess = g
        logger.info(f"身份：{g.status} {g.pid or ''} —— {g.detail}")

        pid = g.pid if g.status == "known" else None
        if pid:
            self.profiles.ensure(pid)
            self.profiles.touch(pid)
        elif g.status == "stranger":
            self._maybe_enroll()
        with self._lock:
            self._changed = bool(pid and self._prev_known and pid != self._prev_known)
            if pid:
                self._prev_known = pid
            self.current_pid = pid
        return self._context_text(g, pid)

    def _context_text(self, g: SpeakerGuess, pid: str | None) -> str:
        if pid:
            prof = self.profiles.get(pid) or {}
            name = prof.get("name")
            who = f"{name}（档案 {pid}）" if name else \
                f"一位还不知道称呼的人（档案 {pid}）；合适的时候可以自然地问一句怎么称呼，别每轮都问"
            lines = [f"【在场的人】正在跟你说话的是 {who}。"]
            summ = profile_summary(prof)
            if summ:
                lines.append(f"关于 TA：{summ}。可以自然地用上，但别逐条复述。")
            if self.is_owner(pid):
                lines.append("TA 是这台机器人的主人。")
            msgs = self.messages.peek(pid)
            if msgs:
                body = "；".join(f"{m['from'] or '有人'}（{m['t']}）说：{m['text']}" for m in msgs)
                lines.append(f"有给 TA 的留言，请在这次回答里先转告：{body}")
                self._delivered_to = pid
            return "\n".join(lines)
        if g.status == "crowd":
            return "【在场的人】面前有好几个人，不确定是谁在说话；不要称呼名字。"
        if g.status in ("stranger", "unsure"):
            return "【在场的人】面前的人还认不出来；不要猜名字。"
        return ""

    def _maybe_enroll(self) -> None:
        if not self._auto_enroll or self._target is None:
            return
        now = time.monotonic()
        with self._lock:
            if self._enrolling or now - self._last_enroll_t < self._cooldown:
                return
            self._enrolling = True
            self._last_enroll_t = now
        pid = self.profiles.next_auto_id()
        threading.Thread(target=self._enroll, args=(pid,), name="identity-enroll",
                         daemon=True).start()

    def _enroll(self, pid: str) -> None:
        try:
            out = self._target.enroll(pid)
            self.profiles.ensure(pid, auto=True)
            logger.info(f"身份：新面孔已录入 {pid} —— {out}")
            with self._lock:
                self.current_pid = pid
                self._prev_known = pid
        except Exception as e:  # noqa: BLE001
            logger.info(f"身份：自动录入 {pid} 没成 —— {e}")
        finally:
            with self._lock:
                self._enrolling = False

    def _remember_turn(self, pid: str, user_text: str, reply: str) -> None:
        name, facts = memory.extract(self._llm, user_text, reply, self.profiles.get(pid))
        if name:
            other = self.profiles.find_by_name(name)
            if other and other != pid:
                logger.warning(f"身份：{pid} 自称「{name}」，但这个称呼已属于 {other}；"
                               "不自动合并（可能是同一人被录了两份，也可能重名）")
                facts = dict(facts, 自称=name)
            else:
                self.profiles.set_name(pid, name)
        if facts:
            self.profiles.add_facts(pid, facts)
            logger.info(f"身份：{pid} 记下 {facts}")
