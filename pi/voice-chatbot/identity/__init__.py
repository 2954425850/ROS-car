"""身份 v1：认出说话人 → 个性化应答 → 对话中自动建档。

profiles.py  —— 档案存储（~/.jarvis/profiles/<id>.json）
speaker.py   —— 唤醒前后的人脸 → 是谁（纯逻辑，宁可不认不要认错）
memory.py    —— 一轮对话结束后，后台从对话里提取称呼/偏好写进档案
"""
