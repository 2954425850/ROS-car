# Pi-side helper: 传到 K230 上跑，扫描 examples 里的多 VENC 通道用法
import os

ROOT = "/sdcard/examples"
hits = []
enc_counts = []
nfiles = 0

def walk(d):
    out = []
    try:
        items = os.listdir(d)
    except Exception:
        return out
    for it in items:
        p = d + "/" + it
        try:
            st = os.stat(p)
        except Exception:
            continue
        if st[0] & 0x4000:  # dir
            out.extend(walk(p))
        elif it.endswith(".py"):
            out.append(p)
    return out

files = walk(ROOT)
print("PYFILES", len(files))
for p in files:
    try:
        s = open(p).read()
    except Exception:
        continue
    nfiles += 1
    ids = []
    for i in range(4):
        if ("VENC_CHN_ID_%d" % i) in s:
            ids.append(i)
    if len(ids) >= 2:
        hits.append((p, ids))
    ne = s.count("Encoder()")
    if ne >= 2:
        enc_counts.append((p, ne))

print("--- files with >=2 distinct VENC_CHN_ID_n ---")
for p, ids in hits:
    print("MULTI", p, ids)
print("TOTAL_MULTI", len(hits))
print("--- files with >=2 Encoder() instances ---")
for p, ne in enc_counts:
    print("ENC2", p, ne)
print("TOTAL_ENC2", len(enc_counts))
print("SCANNED", nfiles)
