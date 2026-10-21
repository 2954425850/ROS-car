#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A010 排障 / 标定 CLI。**不碰 ROS**，直接开串口。

    tof_probe --probe                       # 只做握手与 DISP 扫描，诊断链路
    tof_probe --live                        # 持续打 10x10 块图（mm）
    tof_probe --live --box 0.41,0.49,0.56,0.72   # 顺带打这个框映射到的区域与深度
    tof_probe --record --box L,T,R,B --tof-cell C,R --samples s.json
    tof_probe --solve --samples s.json      # 9 点标定 → map 的四/六参
    tof_probe --scale scale.json            # UNIT 尺度验收

## 标定流程（Task 4 那套，但换成 Pi 侧）

1. 目标放到画面**正中** → `--live --box ...`，看块图里目标落在哪个格 → `--record`
2. 左移、上移… **至少 6 个点，9 个更好**（要覆盖画面各处，不然仿射外推很野）
3. `--solve` 出 `map`，抄进 `config/tof.yaml`
4. 验收：再摆几个**没参与拟合**的位置，`--live` 看 `depth_mm` 对不对（水平差 < 10mm）

★ `--probe` 里 DISP 会依次试 3/6/7/2/4。**3 才是对的**（USB 位）；
  写 5（LCD|UART）的典型症状是"屏亮着、一个字节都收不到"。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

from . import a010, calib
from . import config as tof_config
from .config import load as load_config
from .serial_dev import A010


class OpenFailed(Exception):
    """串口打不开（被占 / 设备不在）。已在 `_open` 里打人话，这里只负责中断。"""


def _open(args, cfg):
    path, why = (args.device, "命令行指定") if args.device \
        else tof_config.resolve_device(cfg)
    print("设备：%s（%s）" % (path, why))
    dev = A010(device=path, baud=int(args.baud or cfg["baud"]))
    try:
        dev.open()
    except Exception as e:                       # noqa: BLE001
        msg = str(e)
        # ⚠️ 独占锁失败的 errno 是 **11 (EAGAIN / "Resource temporarily unavailable")**，
        #    不是 16 (EBUSY)。实测：`Could not exclusively lock port ... [Errno 11]`。
        if any(k in msg for k in ("exclusively lock", "busy", "Errno 11",
                                  "Errno 16", "temporarily unavailable")):
            print("\n❌ 串口被占着 —— **就是 `tof_node` 在占**（独占锁拿不到）。")
            print("   同一个口不能被两个进程同时打开。先停节点再跑本工具：")
            print("       pkill -f 'tof[_]node'        # 或者 kill 掉 `ros2 run` 那个进程")
            print("       fuser -v /dev/ttyUSB0        # 看清楚是谁占的")
            print("       # 跑完本工具，再把节点起回来")
            raise OpenFailed()
        if "No such file" in msg or "Errno 2" in msg:
            print("\n❌ 设备不存在 —— 检查：模块插好了吗？by-id 名字对不对？")
            print("   ls -l /dev/serial/by-id/ | grep -i meta")
            raise OpenFailed()
        raise
    return dev


def _frame_sync(dev, timeout=3.0):
    """等一个合法帧。"""
    return dev.read_frame(timeout=timeout)


# --------------------------------------------------------------------------

def cmd_probe(args, cfg):
    """诊断：握手 / DISP 扫描 / UNIT 回读 / 帧率。"""
    print("配置：%s" % (cfg.get("_path") or "（内置默认）"))
    print("设备：%s @ %s" % (args.device or cfg["device"], args.baud or cfg["baud"]))
    dev = _open(args, cfg)
    try:
        print("\n--- AT 探活 ---")
        if not dev.ping():
            print("  ✗ AT 无应答。检查：设备名对不对、模块有没有上电、")
            print("    有没有别的进程占着这个口（`fuser -v /dev/ttyUSB0`）。")
            return 1
        print("  ✓ AT → OK")

        # ★ **必须先停流再查询**。DISP 是掉电不保存、但**开串口也不会复位它**
        #   （本板实测：模块不会因 DTR 复位而回 DISP=1）。所以上次留的 DISP=2/3
        #   会让模块一直推帧 —— 帧数据把应答淹了，而 `cmd()` 会在**帧的 payload 里
        #   碰巧找到 "OK" 三个字节**，于是返回 ok=True 而正文是乱码。
        #   这就是本工具最早那次 `AT+BAUD? → b'@BBBCDEFGGG...'` 的来源。
        dev.cmd("AT+DISP=0", timeout=0.8)
        for q in ("AT+BAUD?", "AT+UNIT?", "AT+BINN?", "AT+DISP?"):
            ok, _ = dev.cmd(q, timeout=0.6)
            print("  %-10s ok=%-5s %r" % (q, ok, dev.last_response[:40]))
        print("\n--- DISP 扫描（找哪个位能出帧）---")
        for disp in (3, 6, 7, 2, 4):
            dev.cmd("AT+DISP=0", timeout=0.5)
            ok, _ = dev.cmd("AT+DISP=%d" % disp, timeout=0.8)
            t0 = time.time()
            n = 0
            while time.time() - t0 < 2.5 and n < 3:
                if dev.read_frame(timeout=0.6) is not None:
                    n += 1
            print("  DISP=%d  ack=%-5s  2.5s 内合法帧 %d %s"
                  % (disp, ok, n, "  ★ 用这个" if n >= 2 else ""))
            if n >= 2:
                break
        else:
            print("  ✗ 全都不出帧。先怀疑 DISP 位，再怀疑供电/接线。")
            return 1
        print("\n--- 帧率（2s）---")
        n, t0 = 0, time.time()
        while time.time() - t0 < 2.0:
            if dev.read_frame(timeout=0.4) is not None:
                n += 1
        print("  %.1f fps（USB 路径 115200 实测基准 ~9 fps）"
              % (n / (time.time() - t0)))
        return 0
    finally:
        dev.stop_stream()
        dev.close()


def cmd_live(args, cfg):
    """持续打块图；给了 --box 就打这个框的映射区域与深度。"""
    dev = _open(args, cfg)
    dev.init(unit_mm=float(cfg["unit_mm_per_count"]), binn=int(cfg["binn"]),
             disp=int(cfg["disp_mode"]))
    box = [float(v) for v in args.box.split(",")] if args.box else None
    if box:
        print("框 %s → 映射 map=%s" % (box, cfg.get("map")))
    print("Ctrl-C 退出。\n")
    try:
        n = 0
        while n < args.n or args.n <= 0:
            fr = _frame_sync(dev, timeout=2.0)
            if fr is None:
                print("  （2s 没收到合法帧）")
                continue
            n += 1
            print("=" * 78)
            print("帧 %d  id=%s %dx%d  err=%d"
                  % (n, fr.frame_id, fr.rows, fr.cols, fr.error_code))
            print(a010.format_block_map(
                a010.block_map(fr, n=args.grid,
                               unit_mm=float(cfg["unit_mm_per_count"]),
                               min_mm=float(cfg["min_mm"]),
                               val_max=int(cfg["val_max"])),
                unit_mm=float(cfg["unit_mm_per_count"])))
            if box:
                reg = a010.region_bounds(box, cfg.get("map"),
                                         pad=float(cfg["pad"]),
                                         rows=fr.rows, cols=fr.cols)
                d = a010.box_depth(fr, box,
                                   unit_mm=float(cfg["unit_mm_per_count"]),
                                   map_cfg=cfg.get("map"), pad=float(cfg["pad"]),
                                   min_mm=float(cfg["min_mm"]),
                                   val_max=int(cfg["val_max"]),
                                   exclude=cfg.get("exclude") or None)
                if reg:
                    print("  框映射到区域 (r %s..%s, c %s..%s)"
                          % (reg[0], reg[1], reg[2], reg[3]))
                else:
                    print("  框映射到了视野外")
                off = float(cfg.get("offset_mm", 0.0))
                if d is None:
                    print("  ★ depth_mm = None（框内没有有效回波）")
                else:
                    print("  ★ 原始 %d mm → 喂给 geom %d mm（偏置 %+.1f）"
                          % (d, int(round(d + off)), off))
            if args.interval > 0:
                time.sleep(args.interval)
        return 0
    except KeyboardInterrupt:
        print("\n退出。")
        return 0
    finally:
        dev.stop_stream()
        dev.close()


def cmd_record(args, cfg):
    """记一个映射样本。"""
    if not args.box:
        print("❌ --record 要 --box L,T,R,B（那一刻的框）")
        return 2
    box = [float(v) for v in args.box.split(",")]
    if args.tof_cell:
        c, r = [float(v) for v in args.tof_cell.split(",")]
        tx, ty = (c + 0.5) / 10.0, (r + 0.5) / 10.0
    elif args.tof_norm:
        tx, ty = [float(v) for v in args.tof_norm.split(",")]
    else:
        print("❌ 要 --tof-cell C,R（块图格子，0~9）或 --tof-norm x,y")
        return 2
    s = {"box_cx": (box[0] + box[2]) / 2.0,
         "box_cy": (box[1] + box[3]) / 2.0,
         "tof_cx": tx, "tof_cy": ty,
         "box": box, "t": time.strftime("%Y-%m-%d %H:%M:%S")}
    data = _load_json(args.samples)
    data.append(s)
    _save_json(args.samples, data)
    print("✓ 记下第 %d 个样本：框中心 (%.3f, %.3f) → ToF (%.3f, %.3f)"
          % (len(data), s["box_cx"], s["box_cy"], tx, ty))
    print("  样本文件：%s" % args.samples)
    return 0


def cmd_scale_record(args, cfg):
    """记一个**尺度**样本：目标平面放在尺量已知距离处，读中心区的最小计数。

    `tof_probe --scale-record --true-mm 400`

    多帧取**中位数**（每帧先取区内最小计数，再跨帧取中位数）：
    单帧受噪声影响，跨帧中位数稳；同时报 1σ 让你知道这个面稳不稳。
    """
    if args.true_mm is None:
        print("❌ --scale-record 要 --true-mm（尺量真值，毫米）")
        return 2
    dev = _open(args, cfg)
    try:
        dev.init(unit_mm=float(cfg["unit_mm_per_count"]), binn=int(cfg["binn"]),
                 disp=int(cfg["disp_mode"]))
        box = [float(v) for v in (args.box or "0.4,0.4,0.6,0.6").split(",")]
        n = args.n or 15
        counts, meds = [], []
        for _ in range(n):
            fr = dev.read_frame(timeout=2.0)
            if fr is None:
                continue
            reg = a010.region_bounds(box, None, pad=0.0,
                                     rows=fr.rows, cols=fr.cols)
            if reg is None:
                continue
            # unit_mm=1 ⇒ 返回的就是**计数**本身
            c = a010.region_min(fr.payload, reg, fr.rows, fr.cols,
                                1, int(cfg["val_max"]))
            if c is not None:
                counts.append(c)
            vals = sorted(fr.payload[r * fr.cols + cc]
                          for r in range(reg[0], reg[1])
                          for cc in range(reg[2], reg[3])
                          if 0 < fr.payload[r * fr.cols + cc] <= int(cfg["val_max"]))
            if vals:
                meds.append(vals[len(vals) // 2])
        if len(counts) < 3:
            print("❌ 只拿到 %d 帧有效数据，太少（目标对着模块了吗？）" % len(counts))
            return 1
        counts.sort()
        meds.sort()
        med = counts[len(counts) // 2]
        med_center = meds[len(meds) // 2] if meds else None
        p10_center = meds[len(meds) // 10] if meds else None
        mean = sum(counts) / len(counts)
        var = sum((c - mean) ** 2 for c in counts) / len(counts)
        unit = float(cfg["unit_mm_per_count"])
        print("  帧数 %d   计数 中位=%d 均值=%.1f  极差=%d..%d  1σ=%.2f 计数"
              % (len(counts), med, mean, counts[0], counts[-1], var ** 0.5))
        print("  模块读数 = %d × %.1f = **%d mm**   尺量真值 = %d mm   差 %+d mm"
              % (med, unit, med * unit, args.true_mm, med * unit - args.true_mm))
        if med_center is not None:
            print("  对照：区内 **中位数** = %d 计数 (=%d mm)，p10 = %d 计数。"
                  % (med_center, med_center * unit, p10_center))
            print("        min 与中位数若差很多 ⇒ 框内混进了**近场假回波**，"
                  "这时的 min 不能用。")

        data = _load_json(args.scale_out)
        data.append({"count": med, "true_mm": float(args.true_mm),
                     "median_count": med_center, "p10_count": p10_center,
                     "n": len(counts), "sigma_counts": round(var ** 0.5, 3),
                     "unit_at_time": unit,
                     "t": time.strftime("%Y-%m-%d %H:%M:%S")})
        _save_json(args.scale_out, data)
        print("  ✓ 记下第 %d 个尺度样本 → %s" % (len(data), args.scale_out))
        print("    样本够了就跑：tof_probe --scale %s" % args.scale_out)
        return 0
    finally:
        dev.stop_stream()
        dev.close()


def _grab_median(dev, n=6):
    """抓 n 帧取逐像素中位数（降噪）。返回 100x100 的 float 数组。"""
    import numpy as np
    acc = []
    for _ in range(n):
        fr = dev.read_frame(timeout=4.0)
        if fr is not None:
            acc.append(np.frombuffer(fr.payload, dtype=np.uint8).astype(float))
    if not acc:
        return None, None
    fr = dev.read_frame(timeout=2.0) or None
    return np.median(np.stack(acc), axis=0).reshape(100, 100), (fr.rows, fr.cols)


def _largest_blob(mask):
    """100x100 布尔图里最大的 4-连通块。返回 (ys, xs)。纯 Python BFS，够快。"""
    R, C = mask.shape
    seen = mask.copy()
    best = ([], [])
    for r0 in range(R):
        for c0 in range(C):
            if not seen[r0, c0]:
                continue
            stack = [(r0, c0)]
            seen[r0, c0] = False
            ys, xs = [], []
            while stack:
                r, c = stack.pop()
                ys.append(r); xs.append(c)
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < R and 0 <= cc < C and seen[rr, cc]:
                        seen[rr, cc] = False
                        stack.append((rr, cc))
            if len(ys) > len(best[0]):
                best = (ys, xs)
    return best


def _obj_centroid(ref, cur, unit, min_delta_counts=6, min_px=40):
    """`cur` 比 `ref`(空场景) 显著更近的**最大连通块** → (cx, cy, npx, delta_mm)。

    ★ 为什么要差分而不是"直接找最近的一簇"：ToF 视野里**一直有一条 ~152mm 的
      近场假回波**（贴着量程下限），它又大又近，任何"取最近"都会选中它。
      差分把**两帧都有**的东西（桌面、假回波、机械臂）全部消掉，只留你新放上去的东西。
    """
    import numpy as np                                  # 本函数自己 import：
    d = np.where((ref > 5) & (cur > 5), ref - cur, 0)     # >0 ⇒ cur 更近
    mask = d >= min_delta_counts
    if mask.sum() < min_px:
        return None
    ys, xs = _largest_blob(mask)
    if len(ys) < min_px:
        return None
    ys, xs = np.array(ys), np.array(xs)
    return (xs.mean() / 100.0, ys.mean() / 100.0, len(ys),
            float(np.mean(d[ys, xs]) * unit))


def cmd_calib(args, cfg):
    """仿射标定的采样：`--calib-ref` 存空场景基准；`--calib-record` 记一个点。"""
    import numpy as np

    dev = _open(args, cfg)
    try:
        dev.init(unit_mm=float(cfg["unit_mm_per_count"]), binn=int(cfg["binn"]),
                 disp=int(cfg["disp_mode"]))
        cur, _ = _grab_median(dev, n=args.n or 6)
        if cur is None:
            print("❌ 没抓到帧")
            return 1

        if args.calib_ref:
            np.save(args.ref, cur)
            print("✓ 已存空场景基准 → %s" % args.ref)
            print("  现在把目标放到画面某个位置，再跑 --calib-record")
            return 0

        # --calib-record
        if not os.path.isfile(args.ref):
            print("❌ 没有基准文件 %s —— 先跑一次 --calib-ref（桌面清空）" % args.ref)
            return 1
        ref = np.load(args.ref)
        found = _obj_centroid(ref, cur, float(cfg["unit_mm_per_count"]))
        if found is None:
            print("❌ 没找到「比空场景更近的块」—— 目标是不是不在 ToF 视野里？")
            return 1
        tx, ty, npx, dmm = found
        print("ToF 侧：目标重心 归一化 (%.3f, %.3f)   占 %d 像素   平均近 %.0f mm"
              % (tx, ty, npx, dmm))

        # 相机侧：优先命令行给的框 → 自动抓图找亮块 → 结果流的检测框。
        # **别被检测器绑架**：它只认训练过的类，随便放个东西多半不认。
        box = None
        if args.box:
            box = [float(v) for v in args.box.split(",")]
            print("相机侧：框来自 --box（人给的）")
        elif args.calib_auto:
            from . import camsnap
            jpg = camsnap.snap()
            blob = camsnap.bright_blob(jpg) if jpg else None
            if blob is None:
                print("❌ 自动抓图/找块失败（板子 IP 变了？球不在画面里？）")
                print("   board_host = %s" % camsnap.board_host())
                return 1
            bx, by, npx = blob
            print("相机侧：自动找到亮块，重心 (%.3f, %.3f) 占 %d 像素" % (bx, by, npx))
            box = [bx - 0.02, by - 0.02, bx + 0.02, by + 0.02]
        else:
            box = _biggest_box()
            if box is not None:
                print("相机侧：框来自结果流（检测器）")
        if box is None:
            print("❌ 没有框。用 --box L,T,R,B，或 --calib-auto，或让检测器检出。")
            return 1
        cx = (box[0] + box[2]) / 2.0
        cy = (box[1] + box[3]) / 2.0
        print("相机侧：框 [%.3f %.3f %.3f %.3f]  中心 (%.3f, %.3f)"
              % (box[0], box[1], box[2], box[3], cx, cy))

        data = _load_json(args.samples)
        data.append({"box_cx": cx, "box_cy": cy, "tof_cx": float(tx), "tof_cy": float(ty),
                     "blob_px": npx, "delta_mm": dmm,
                     "t": time.strftime("%Y-%m-%d %H:%M:%S")})
        _save_json(args.samples, data)
        print("  ✓ 记下第 %d 个映射样本 → %s" % (len(data), args.samples))
        print("    换位置再来；够了跑：tof_probe --solve --samples %s" % args.samples)
        return 0
    finally:
        dev.stop_stream()
        dev.close()


def _biggest_box(path="/tmp/k230/latest-result.json"):
    """结果流里面积最大的框（优先 track）。"""
    if not os.path.isfile(path):
        return None
    try:
        d = json.load(open(path, encoding="utf-8"))
    except Exception:                              # noqa: BLE001
        return None
    best, best_a = None, -1
    for o in d.get("objs") or []:
        b = o.get("box")
        if not b:
            continue
        a = (b[2] - b[0]) * (b[3] - b[1]) * (2.0 if o.get("src") == "track" else 1.0)
        if a > best_a:
            best, best_a = b, a
    return best


def cmd_solve(args, cfg):
    data = _load_json(args.samples)
    if len(data) < 3:
        print("❌ 样本太少（%d 个），至少 3 个、建议 9 个" % len(data))
        return 1
    print(calib.map_report(data))
    return 0


def cmd_scale(args, cfg):
    data = _load_json(args.scale)
    print(calib.scale_report(data, float(cfg["unit_mm_per_count"])))
    return 0


def _load_json(path):
    if not path or not os.path.isfile(path):
        return []
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _save_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


# --------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(prog="tof_probe",
                                 description="A010 ToF 排障 / 标定")
    ap.add_argument("--probe", action="store_true", help="只诊断链路")
    ap.add_argument("--live", action="store_true", help="持续打块图")
    ap.add_argument("--record", action="store_true", help="记一个映射样本")
    ap.add_argument("--solve", action="store_true", help="解映射（4参/6参）")
    ap.add_argument("--scale", metavar="FILE", help="解尺度（UNIT 验收）")
    ap.add_argument("--scale-record", action="store_true",
                    help="记一个尺度样本（要 --true-mm）")
    ap.add_argument("--calib-ref", action="store_true",
                    help="存一张空场景基准帧（仿射标定要先做这个）")
    ap.add_argument("--calib-record", action="store_true",
                    help="记一个映射样本（差分找 ToF 目标 + 取相机侧位置）")
    ap.add_argument("--calib-auto", action="store_true",
                    help="相机侧自动抓图找亮块（配 --calib-record 用）")
    ap.add_argument("--ref", default="~/tof-ref.npy", help="空场景基准文件")
    ap.add_argument("--scale-out", default="~/tof-scale.json",
                    help="尺度样本文件（默认 ~/tof-scale.json）")
    ap.add_argument("--true-mm", type=float, default=None,
                    help="尺量真值（毫米），配合 --scale-record")
    ap.add_argument("--samples", default="~/tof-map-samples.json",
                    help="映射样本文件")
    ap.add_argument("--device", default=None)
    ap.add_argument("--baud", default=None)
    ap.add_argument("--box", default=None, help="归一化 l,t,r,b")
    ap.add_argument("--tof-cell", default=None, help="块图格子 C,R（0~9）")
    ap.add_argument("--tof-norm", default=None, help="ToF 归一化 x,y")
    ap.add_argument("--grid", type=int, default=10, help="块图阶数（默认 10）")
    ap.add_argument("--n", type=int, default=0, help="打几帧后退出（0=一直打）")
    ap.add_argument("--interval", type=float, default=0.0)
    args = ap.parse_args(argv)

    if args.samples:
        args.samples = os.path.expanduser(args.samples)
    if args.scale_out:
        args.scale_out = os.path.expanduser(args.scale_out)
    if args.ref:
        args.ref = os.path.expanduser(args.ref)
    cfg = load_config()

    try:
        return _dispatch(args, cfg)
    except OpenFailed:
        return 1


def _dispatch(args, cfg):
    if args.probe:
        return cmd_probe(args, cfg)
    if args.live:
        return cmd_live(args, cfg)
    if args.record:
        return cmd_record(args, cfg)
    if args.solve:
        return cmd_solve(args, cfg)
    if args.calib_ref or args.calib_record:
        return cmd_calib(args, cfg)
    if args.scale_record:
        return cmd_scale_record(args, cfg)
    if args.scale:
        return cmd_scale(args, cfg)
    ap.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
