# 扫描 v2：报告读取失败原因，按块读避免大文件问题
import os

ROOT = "/sdcard/examples"

def walk(d):
    out = []
    try:
        items = os.listdir(d)
    except Exception as e:
        print("LISTDIR_FAIL", d, repr(e))
        return out
    for it in items:
        p = d + "/" + it
        try:
            st = os.stat(p)
        except Exception:
            continue
        if st[0] & 0x4000:
            out.extend(walk(p))
        elif it.endswith(".py"):
            out.append(p)
    return out

files = walk(ROOT)
print("PYFILES", len(files))
ok = 0
fail = 0
failex = {}
multi = []
enc2 = []
for p in files:
    try:
        s = open(p).read()
        ok += 1
    except Exception as e:
        fail += 1
        k = repr(e)
        if k not in failex:
            failex[k] = p
        continue
    ids = [i for i in range(4) if ("VENC_CHN_ID_%d" % i) in s]
    if len(ids) >= 2:
        multi.append((p, ids))
    ne = s.count("Encoder()")
    if ne >= 2:
        enc2.append((p, ne))

print("OK", ok, "FAIL", fail)
for k, v in failex.items():
    print("FAILEX", k, "FIRST", v)
print("--- MULTI ---")
for p, ids in multi:
    print("MULTI", p, ids)
print("TOTAL_MULTI", len(multi))
print("--- ENC2 ---")
for p, ne in enc2:
    print("ENC2", p, ne)
print("TOTAL_ENC2", len(enc2))
