"""QQ 音乐登录 —— 弄一份「本机现在就能用」的 QQ_MUSIC_COOKIE。

## 为什么需要它（这不是一次性的事）

腾讯按**请求方出口 IP** 校验 `qm_keyst`。实测：cookie 在家庭宽带
（111.61.124.136）上取的，机器挪到手机热点（39.144.85.159）之后，
同一个 cookie 直接报

    104009  msg="39.144.85.159;invalidq;"

`qm_keyst` 还会被别处的登录挤掉（同一个号在手机/浏览器上再登一次就换了）。
所以 cookie 会反复失效，需要的是**一条能在本机重新登录的路径**，而不是粘贴一次。

## 为什么用 qqmusic-api-python 而不是手搓 ptlogin2

扫码要两步 —— PTUI 二维码 → 拿 `p_skey` 换 music credential，中间还有 DES/RSA 签名。
`qqmusic-api-python` 全实现了（`LoginApi.get_qrcode` → `check_qrcode` → `Credential`），
还白送一个 `refresh_credential()` —— 运气好时**不扫码**就能续期。手搓是重复劳动，
而且追不上腾讯改协议。

注意别和 `qq_music_api`（下划线）搞混：那是 qq-music-mcp 0.1.0 自带的、真正发请求的
客户端，**没有**登录能力。两个包名只差一个下划线，读 import 时看仔细。

## cookie 长什么样（硬约束）

`qq_music_api.QQMusicClient` **不解析** cookie 的字段，只把整串塞进 `Cookie:` 头，
再用 `re.search(r"uin=(\\d+)")` 抠出 uin 来设 `loginflag = 1 if cookie else 0`。
所以最小可用 cookie 就是：

    uin=<musicid>; qm_keyst=<musickey>; qqmusic_key=<musickey>

（后两个历史上同值；多带一份是照抄浏览器 cookie 的形状，不亏。）

## 【同样重要】必须传 device_path，否则每次登录都算一台新设备

`qqmusic_api` 会模拟一台安卓机（`utils/device.py` 里 `model = "MI 6"` 是**硬编码**的，
所以腾讯设备列表里那些"小米6"就是它）。但设备指纹里的 `open_udid` / `imei` / `boot_id` /
`android_id` 全是**随机生成**的，而 `DeviceManager` 的注释写明：
**`device_path=None` 时设备信息只在内存里维护**（不落盘、下次重跑又是新的一台）。

后果实测：不传 `device_path` 时，**每跑一次 login/refresh，腾讯那边就多一台新"小米6"**。
（2026-09-17 我连做 3 次认证，账号设备列表里正好多出 3 台小米6 —— 就这么来的。配额一满，
扫码换凭证那步直接 `20279 登录设备超限`，救都救不回来。）

**所以这里必须传 `DEVICE_PATH`**，让设备指纹落盘、每次复用同一台 —— 否则"自动续期"
等于每 60 小时吃一个设备配额，几天就把账号堵死。附带好处：库里还缓存了
`session_uid/sid/vkey`（见 `_is_session_valid`），同一台设备可能直接复用会话。

## 【最重要】refresh 不是幂等的，别乱刷

`refresh` 走的是 `music.login.LoginServer/Login` —— **它就是一次真登录**。实测到的机制
（2026-09-17 踩的，代价是那份 key 报废）：

- refresh 必须拿**还活着的** key 去换。key 一死，refresh 只会回
  `登录鉴权参数无效或已过期`，再也救不回来，只能重新扫码。
- **每次尝试都会替换服务端 key，哪怕这次尝试失败**（例如回 `登录设备超限`），
  正在用的那个 key 也一起报废 —— 也就是说：**失败的一次 refresh 比不刷更糟。**
- 短时间连续刷会撞 `登录设备超限`（我 5 分钟内刷 3 次就撞上了）。

所以 `refresh` 默认带**门槛**：key 剩余寿命 > `MIN_REMAINING_SEC` 就直接跳过，
什么都不做。想强行刷用 `--force` —— 但在按下去之前先想清楚上面那条。

## 用法

    python3 tools/qq_login.py login     # 扫码 → 校验 → 写 config.yaml → 重启服务
    python3 tools/qq_login.py refresh   # 只在 key 快到期时续期（有门槛，见上）
    python3 tools/qq_login.py check     # 只体检当前 cookie，什么都不改

`login` 会把二维码存成 PNG（`--qr` 可改路径），用手机 QQ 扫它 ——
**腾讯那边只给约 120 秒**，扫完还得在手机上点确认，别放着不管。

**验证不过就绝不落盘**：先跑两个体检（登录态 + 真取一次播放链接），都过了才写
config.yaml 并重启。之前那份 cookie 的教训是 —— 只有真拿到 purl 才算数。
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import httpx
from loguru import logger

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = PROJECT_ROOT / "config.yaml"
CRED_PATH = Path.home() / ".config" / "voice-chatbot" / "qqmusic_credential.json"
# 设备指纹落盘路径。**必须传**，理由见模块文档「必须传 device_path」一节：
# 不传的话每次 login/refresh 都会在腾讯那边注册一台新的"小米6"，把设备配额吃干净。
DEVICE_PATH = Path.home() / ".config" / "voice-chatbot" / "qqmusic_device.json"
DEFAULT_QR_PATH = Path("/tmp/qqmusic_qr.png")
SERVICE = "voice-chatbot"

# 体检用的固定曲目：《画心》张靓颖，songmid 实测存在
PROBE_SONG_MID = "0045cdSn27l6YM"
PROBE_QUALITIES = ("320", "128", "m4a")
# 腾讯给的二维码只活约 120 秒（实测 22:30:19 生成 → 22:32:20 报「二维码过期了」）。
# 轮询窗口就按这个来 —— 设更大没用，过期之后扫也白扫，只会让人以为在等。
SCAN_TIMEOUT_SEC = 120

# refresh 的门槛：key 剩余寿命高于它就拒绝续期，见模块文档「refresh 不是幂等的」。
# key 寿命 72h，留 12h 余量 —— 意味着大约每 60h 才真正刷一次。
MIN_REMAINING_SEC = 12 * 3600

_COOKIE_LINE = re.compile(r"^(\s*QQ_MUSIC_COOKIE:\s*).*$", re.MULTILINE)
_EXPIRE_LINE = re.compile(r"^(\s*#\s*到期:\s*).*$", re.MULTILINE)
_UIN_IN_COOKIE = re.compile(r"(?:^|;\s*)uin=(\d+)")

_EVENT_TEXT = {
    "DONE": "已确认登录",
    "SCAN": "已扫码，等手机那边点确认",
    "CONF": "已确认，正在换凭证",
    "TIMEOUT": "二维码过期了",
    "REFUSE": "手机上取消了登录",
}


# ------------------------------------------------------------------ cookie 读写


def current_cookie() -> str:
    """从 config.yaml 抠出当前 cookie。按行取，不碰 YAML 解析（注释要留着）。"""
    match = _COOKIE_LINE.search(CONFIG_PATH.read_text(encoding="utf-8"))
    if not match:
        return ""
    return match.group(0).split(":", 1)[1].strip().strip("'\"")


def make_cookie(credential) -> str:
    """Credential → 一条 cookie。字段见模块文档「硬约束」一节。"""
    key = credential.musickey
    return f"uin={credential.musicid}; qm_keyst={key}; qqmusic_key={key}"


def write_cookie(cookie: str, expires_at: int | None = None) -> Path:
    """把 cookie 写进 config.yaml —— 按行替换，不用 yaml.dump。

    config.yaml 里全是设计注释（`mcp<2` 为什么是硬约束、音质为什么要降级…），
    safe_load + dump 会把它们全冲掉，所以只动目标那一行。顺手把「到期」注释
    改成真实日期 —— 原来那个是手填的，实测连 key 死掉那天都没对上。
    """
    text = CONFIG_PATH.read_text(encoding="utf-8")
    if not _COOKIE_LINE.search(text):
        raise SystemExit(f"config.yaml 里找不到 QQ_MUSIC_COOKIE 行：{CONFIG_PATH}")

    backup = CONFIG_PATH.with_name(f"config.yaml.bak-qqlogin-{time.strftime('%m%d-%H%M%S')}")
    backup.write_text(text, encoding="utf-8")

    quoted = "'" + cookie.replace("'", "''") + "'"  # YAML 单引号转义
    text = _COOKIE_LINE.sub(lambda m: m.group(1) + quoted, text, count=1)
    if expires_at:
        stamp = time.strftime("%Y-%m-%d", time.localtime(expires_at))
        text = _EXPIRE_LINE.sub(lambda m: m.group(1) + stamp, text, count=1)
    CONFIG_PATH.write_text(text, encoding="utf-8")
    os.chmod(CONFIG_PATH, 0o600)
    return backup


def lock_device_file() -> None:
    """把设备文件收成 600。

    库自己写这个文件（默认 umask，实测 664），而它除了设备指纹还会缓存
    session 的 uid/sid/vkey —— 别让同组用户读得走。
    """
    with contextlib.suppress(OSError):
        os.chmod(DEVICE_PATH, 0o600)


def device_summary() -> str:
    """报一眼落盘的设备指纹。

    这只是给人看的：**每次跑之前后都该是同一台**。如果 open_udid 变了，说明设备指纹
    没复用成功 —— 那就又会去腾讯那边新注册一台"小米6"，设备配额会慢慢被吃光。
    """
    if not DEVICE_PATH.exists():
        return "（尚未落盘，本次会新建一台）"
    data = json.loads(DEVICE_PATH.read_text(encoding="utf-8"))
    return (
        f"{data.get('brand')} {data.get('model')}"
        f" / open_udid={str(data.get('open_udid'))[:12]}…"
        f" / imei={str(data.get('imei'))[:9]}…"
    )


def save_credential(credential) -> None:
    """存 credential（含 refresh_token）——refresh 子命令要用它，能免一次扫码。"""
    CRED_PATH.parent.mkdir(parents=True, exist_ok=True)
    CRED_PATH.write_text(credential.model_dump_json(by_alias=True), encoding="utf-8")
    os.chmod(CRED_PATH, 0o600)


def load_credential():
    from qqmusic_api import Credential

    if not CRED_PATH.exists():
        return None
    return Credential.model_validate_json(CRED_PATH.read_text(encoding="utf-8"))


# -------------------------------------------------------------------- 两次体检


async def probe_login(cookie: str) -> tuple[bool, str]:
    """问腾讯「这条 cookie 算登录了吗」。"""
    if not cookie.strip():
        return False, "cookie 是空的"
    uin = _UIN_IN_COOKIE.search(cookie)
    payload = {
        "comm": {"ct": 24, "cv": 0, "uin": uin.group(1) if uin else "0"},
        "req": {
            "module": "music.UserInfo.userInfoServer",
            "method": "GetLoginUserInfo",
            "param": {},
        },
    }
    async with httpx.AsyncClient(timeout=15.0) as client:
        resp = await client.get(
            "https://u.y.qq.com/cgi-bin/musicu.fcg",
            params={"format": "json", "data": json.dumps(payload)},
            headers={"Cookie": cookie, "Referer": "https://y.qq.com/"},
        )
    code = (resp.json().get("req") or {}).get("code")
    if code == 0:
        return True, "登录态有效"
    return False, f"未登录（req.code={code}）"


async def probe_play(cookie: str) -> tuple[bool, str]:
    """用**跟机器人同一条代码路径**验证：真能拿到播放链接吗。

    probe_login 只证明登录态；这一步才证明「放歌」通。旧 cookie 的教训正是
    —— 登录态和拿到 purl 是两回事，只有拿到 url 才算数。
    """
    from qq_music_api import QQMusicClient

    client = QQMusicClient(cookie=cookie)
    try:
        for quality in PROBE_QUALITIES:
            urls = await client.get_song_url(PROBE_SONG_MID, quality)
            if urls and str(urls[0].url or "").strip():
                return True, f"音质 {quality} 拿到链接（{urls[0].size} 字节）"
    finally:
        await client.close()
    return False, f"所有音质都拿不到链接（试过 {'/'.join(PROBE_QUALITIES)}）"


async def verify_and_apply(cookie: str, expires_at: int | None, restart: bool) -> int:
    """先验证、再落盘 —— 体检不过就原样退出，绝不把能用的配置改坏。"""
    ok, detail = await probe_login(cookie)
    logger.info(f"体检一 · 登录态：{detail}")
    if not ok:
        return 1

    ok, detail = await probe_play(cookie)
    logger.info(f"体检二 · 取播放链接：{detail}")
    if not ok:
        logger.error("登录态有效但拿不到播放链接 —— 不写 config.yaml")
        return 1

    backup = write_cookie(cookie, expires_at)
    logger.info(f"已写入 config.yaml（原文备份：{backup.name}）")

    if restart:
        restart_service()
    else:
        logger.info("--no-restart：服务没动，自己重启才会生效")
    return 0


def restart_service() -> None:
    """让服务捡起新 key。

    只有它**本来就活着**时才重启 —— 否则凌晨的定时续期会把用户故意停掉的服务
    偷偷拉起来。服务没跑的话，新 key 已经写在 config.yaml 里，下次启动自然生效。
    """
    live = subprocess.run(
        ["systemctl", "--user", "is-active", SERVICE],
        capture_output=True,
        text=True,
    ).stdout.strip()
    if live != "active":
        logger.info(f"{SERVICE} 当前是 {live}，不打扰它；新 key 已写进配置，下次启动生效")
        return

    subprocess.run(["systemctl", "--user", "restart", SERVICE], check=True)
    time.sleep(2)
    state = subprocess.run(
        ["systemctl", "--user", "is-active", SERVICE],
        capture_output=True,
        text=True,
    ).stdout.strip()
    logger.info(f"{SERVICE} 已重启，当前状态：{state}")


# ---------------------------------------------------------------------- 子命令


async def cmd_login(qr_path: Path, restart: bool) -> int:
    from qqmusic_api import Client
    from qqmusic_api.models.login import QRCodeLoginEvents, QRLoginType

    logger.info(f"设备指纹：{device_summary()}")
    # device_path 必须传 —— 否则每次都是一台新的"小米6"，见模块文档。
    async with Client(device_path=str(DEVICE_PATH)) as client:
        qr = await client.login.get_qrcode(QRLoginType.QQ)
        qr_path.write_bytes(qr.data)
        logger.info(f"二维码已存到 {qr_path}（{qr.mimetype}）")
        logger.info(f"用手机 QQ 扫它 —— **只有约 {SCAN_TIMEOUT_SEC} 秒有效期，扫完要在手机上点确认**")

        credential = None
        last_event = None
        deadline = time.monotonic() + SCAN_TIMEOUT_SEC
        next_nag = time.monotonic() + 30
        while time.monotonic() < deadline:
            result = await client.login.check_qrcode(qr)
            if result.event is not last_event:
                logger.info(
                    f"扫码状态：{_EVENT_TEXT.get(result.event.name, result.event.name)}"
                )
                last_event = result.event
            if result.event is QRCodeLoginEvents.DONE:
                credential = result.credential
                break
            if result.event in (QRCodeLoginEvents.TIMEOUT, QRCodeLoginEvents.REFUSE):
                logger.error("扫码没成，啥也没改。重跑一次 login 即可")
                return 1
            if time.monotonic() > next_nag:
                left = int(deadline - time.monotonic())
                logger.info(f"…还剩 {left} 秒，二维码过期前扫一下")
                next_nag = time.monotonic() + 30
            await asyncio.sleep(1)

    lock_device_file()

    if credential is None:
        logger.error(f"等了 {SCAN_TIMEOUT_SEC} 秒没扫上，啥也没改")
        return 1

    expires_at = credential.musickey_create_time + credential.key_expires_in or None
    save_credential(credential)
    logger.info(f"登录成功：uin={credential.musicid}（凭证已存 {CRED_PATH}）")
    return await verify_and_apply(make_cookie(credential), expires_at, restart)


async def cmd_refresh(restart: bool, force: bool) -> int:
    from qqmusic_api import Client

    credential = load_credential()
    if credential is None:
        logger.error(f"没有存过凭证（{CRED_PATH} 不存在）—— 先跑 login 扫一次码")
        return 1

    # 先做一次**只读**探针，问服务端这颗 key 现在到底还活着没有。
    # 不能只信 credential 里的 musickeyCreateTime / keyExpiresIn —— 那是本地记账，实测会说谎：
    # 一次失败的登录尝试会在服务端把 key 换掉，而本地仍然显示「还剩 71.9 小时」。
    # 拿一颗已死的 key 去 refresh 只会换来「鉴权参数无效」，而且每次尝试本身就是一次登录请求
    # （定时器每 6 小时撞一次，有把账号推向 104604 登录频率限制的风险）。
    # GetLoginUserInfo 是纯读取，不消耗登录次数，所以这一步很便宜。
    alive, detail = await probe_login(current_cookie())
    if not alive:
        logger.error(f"当前 key 实际已失效（{detail}）—— refresh 救不回来，只能 `login` 扫码")
        return 1

    left = credential.musickey_create_time + credential.key_expires_in - int(time.time())
    if not force and left > MIN_REMAINING_SEC:
        logger.info(
            f"key 还剩 {left / 3600:.1f} 小时（门槛 {MIN_REMAINING_SEC / 3600:.0f}h），跳过续期"
        )
        logger.info("提前续期是危险的：每次尝试都会替换服务端 key，失败的那次会把在用的 key 一起报废")
        return 0

    logger.info(f"设备指纹：{device_summary()}")
    try:
        # device_path 必须传，理由同 cmd_login：不传就会新注册一台设备。
        async with Client(credential=credential, device_path=str(DEVICE_PATH)) as client:
            fresh = await client.login.refresh_credential()
    except Exception as exc:  # 网络/设备超限/key 已死都会走这里，不该炸栈
        logger.error(f"续期失败（{type(exc).__name__}: {exc}）")
        logger.error("注意：这次失败很可能已经把那颗 key 报废了 —— 若之后体检也不过，只能 login 扫码")
        return 1

    lock_device_file()
    expires_at = fresh.musickey_create_time + fresh.key_expires_in or None
    save_credential(fresh)
    logger.info(f"续期成功：uin={fresh.musicid}")
    return await verify_and_apply(make_cookie(fresh), expires_at, restart)


async def cmd_check() -> int:
    cookie = current_cookie()
    uin = _UIN_IN_COOKIE.search(cookie)
    logger.info(f"当前 cookie：uin={uin.group(1) if uin else '?'} / {len(cookie)} 字符")

    credential = load_credential()
    if credential is not None:
        left = credential.musickey_create_time + credential.key_expires_in - int(time.time())
        note = "，该续期了（门槛见 refresh）" if left <= MIN_REMAINING_SEC else ""
        # 标明是「本地记账」：它只反映最后一次成功登录时的元数据，服务端 key 被
        # 换掉（比如一次失败的登录尝试）之后它不会跟着变，会显示得比实际长寿。
        logger.info(f"key 剩余寿命（本地记账，未必可信）：{left / 3600:.1f} 小时{note}")
    else:
        logger.info(f"没有存过凭证（{CRED_PATH} 不存在）—— 只能扫码，续期用不了")

    ok, detail = await probe_login(cookie)
    logger.info(f"体检一 · 登录态：{detail}")
    if not ok:
        logger.error("cookie 无效 —— 跑 `login` 扫码，或先试 `refresh`")
        return 1

    ok, detail = await probe_play(cookie)
    logger.info(f"体检二 · 取播放链接：{detail}")
    if not ok:
        logger.error("登录态有效但拿不到链接")
        return 1

    logger.info("两项都过，放歌链路正常")
    return 0


# ------------------------------------------------------------------------ 入口


def main() -> int:
    parser = argparse.ArgumentParser(
        description="QQ 音乐登录/续期（给 voice-chatbot 喂 QQ_MUSIC_COOKIE）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_login = sub.add_parser("login", help="扫码登录")
    p_login.add_argument("--qr", type=Path, default=DEFAULT_QR_PATH, help="二维码 PNG 存哪")
    p_login.add_argument("--no-restart", action="store_true", help="写配置但不重启服务")

    p_refresh = sub.add_parser("refresh", help="续期（只在 key 快到期时真刷）")
    p_refresh.add_argument("--no-restart", action="store_true", help="写配置但不重启服务")
    p_refresh.add_argument(
        "--force",
        action="store_true",
        help="无视剩余寿命门槛强行续期（危险：失败会报废在用的 key）",
    )

    sub.add_parser("check", help="只体检当前 cookie，不改任何东西")

    args = parser.parse_args()
    logger.remove()
    logger.add(sys.stdout, format="{time:HH:mm:ss} | {level:<8} | {message}", level="INFO")

    try:
        if args.cmd == "login":
            return asyncio.run(cmd_login(args.qr, not args.no_restart))
        if args.cmd == "refresh":
            return asyncio.run(cmd_refresh(not args.no_restart, args.force))
        return asyncio.run(cmd_check())
    except KeyboardInterrupt:
        logger.warning("被 Ctrl-C 打断，啥也没改")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
