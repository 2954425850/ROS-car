#!/usr/bin/env python3
"""把 t_push.py / t_push_stress.py 迁到 Task 4 的多通道 cam.py API。

老:  Camera(config.PUSH_CHN, config.WIDTH, config.HEIGHT, config.BITRATE_KBPS)
     用 cam.encoder
新:  Camera(config.WIDTH, config.HEIGHT) ; cam.add_channel(chn, bitrate)
     用 cam.encoder(chn)
"""
import os
import shutil

D = "/home/cy/k230-vision/k230"
OLD_CTOR = "cam = Camera(config.PUSH_CHN, config.WIDTH, config.HEIGHT, config.BITRATE_KBPS)"
NEW_CTOR = ("cam = Camera(config.WIDTH, config.HEIGHT)\n"
            "cam.add_channel(config.PUSH_CHN, config.BITRATE_KBPS)")
OLD_ENC = "p = Pusher(config.PUSH_HOST, config.PUSH_PORT, cam.encoder, config.PUSH_CHN)"
NEW_ENC = ("p = Pusher(config.PUSH_HOST, config.PUSH_PORT,\n"
           "           cam.encoder(config.PUSH_CHN), config.PUSH_CHN)")

for name in ("t_push.py", "t_push_stress.py"):
    p = os.path.join(D, name)
    bak = p + ".beforetask4"
    if not os.path.exists(bak):
        shutil.copy2(p, bak)
        print("backup ->", bak)
    s = open(p).read()
    n1 = s.count(OLD_CTOR)
    n2 = s.count(OLD_ENC)
    s = s.replace(OLD_CTOR, NEW_CTOR).replace(OLD_ENC, NEW_ENC)
    open(p, "w").write(s)
    print("%s: ctor_hits=%d enc_hits=%d" % (name, n1, n2))
print("done")
