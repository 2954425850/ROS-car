"""音频设备解析。

config 里的 `audio.device_index` 支持三种写法：

    null            → 用系统默认设备（走 PipeWire，只在有用户会话时可用）
    3               → 直接指定 PyAudio 的整数 index
    "jarvissnd"     → 按名字模糊匹配，取第一个命中的设备

推荐用名字：整数 index 会随着插拔设备、改动 asound.conf 而变化，
写死了很容易在重启后指向错误的设备。
"""

import pyaudio


def resolve_device_index(value: object) -> int | None:
    """把 config 里的 device_index 解析成 PyAudio 的整数 index。

    Args:
        value: None / int / str，含义见模块文档。

    Returns:
        PyAudio 设备 index，或 None 表示用系统默认设备。

    Raises:
        ValueError: 指定了名字但没有匹配的设备。
    """
    if value is None or isinstance(value, int):
        return value
    if not isinstance(value, str) or not value.strip():
        return None

    needle = value.strip().lower()
    pa = pyaudio.PyAudio()
    try:
        candidates = [
            (i, pa.get_device_info_by_index(i)["name"]) for i in range(pa.get_device_count())
        ]
    finally:
        pa.terminate()

    for index, name in candidates:
        if needle in name.lower():
            return index

    available = ", ".join(f"{i}:{name}" for i, name in candidates)
    raise ValueError(f"找不到名字包含 {value!r} 的音频设备。可用设备：{available}")
