# 扫描 v3：加 gc.collect()，覆盖全部 344 个 .py
import gc, os

ROOT = "/sdcard/examples"

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
rtsp = []
for p in files:
    s = None
    try:
        s = open(p).read()
    except Exception as e:
        fail += 1
        k = repr(e)
        if k not in failex:
            failex[k] = p
        gc.collect()
        continue
    ok += 1
    ids = sorted(set([i for i in range(4) if ("VENC_CHN_ID_%d" % i) in s]))
    if len(ids) >= 2:
        multi.append((p, ids))
    if s.count("Encoder()") >= 2:
        enc2.append((p, s.count("Encoder()")))
    if "rtspserver_sendvideodata" in s or "rtsp_server" in s:
        rtsp.append(p)
    del s
    gc.collect()

print("OK", ok, "FAIL", fail)
for k, v in failex.items():
    print("FAILEX", k, "FIRST", v)
print("--- MULTI (>=2 distinct VENC_CHN_ID_n in one file) ---")
for p, ids in multi:
    print("MULTI", p, ids)
print("TOTAL_MULTI", len(multi))
print("--- ENC2 (>=2 Encoder() in one file) ---")
for p, ne in enc2:
    print("ENC2", p, ne)
print("TOTAL_ENC2", len(enc2))
print("--- files mentioning rtsp server ---")
for p in rtsp:
    print("RTSP", p)
print("TOTAL_RTSP", len(rtsp))
print("MEM_FREE", gc.mem_free())
