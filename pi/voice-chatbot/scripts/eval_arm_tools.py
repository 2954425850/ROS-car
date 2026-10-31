#!/usr/bin/env python3
"""机械臂语音工具形状评测（Task 1）。

用法（在 Pi 上，项目根 pi/voice-chatbot 下）：
    python3 scripts/eval_arm_tools.py --variant A
    python3 scripts/eval_arm_tools.py --variant B --runs 3 --only s01,m01

做的事：
  - 读 config.yaml 的 llm（base_url / model / api_key / temperature / max_tokens / system_prompt）；
    **api key 只用于建客户端，不打印、不写进结果文件**。
  - 把**现有全部工具**（原生工具按 conversation.py 的注册方式构造 registry；MCP 工具真起一次
    MCPHost 取 schema）+ 候选臂工具（tools/arm_schema.py 的 A 或 B）一起发给模型。
  - corpus（tests/arm_eval/corpus.yaml）每条跑 N 次；模型调了工具就喂一条假结果让它继续，
    直到它不再调工具（最多 4 轮，服务里 max_tool_iterations=5）。
  - 判定见 judge()；汇总总通过率、车/臂混淆数、按类别通过率；写 logs/arm_eval-<variant>-<ts>.json。

不碰车：不发任何 ROS 话题 / 动作，假结果都是本脚本编的。
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import yaml  # noqa: E402

from tools.arm_schema import MOTION_TOOLS, STEP_FIELDS, from_spec, schemas as arm_schemas, to_spec  # noqa: E402

CORPUS = ROOT / "tests" / "arm_eval" / "corpus.yaml"
CAR_MOTION = {"car_move", "car_turn"}
ACTION_TOOLS = MOTION_TOOLS | {"car_move", "car_turn", "car_stop", "grasp_object", "track_object",
                               "follow_person"}
CATEGORIES = {"s": "单步模糊", "n": "带数", "m": "多轮相对", "c": "混淆对照", "k": "组合", "h": "状态/复位"}
MAX_ROUNDS = 4


# ====================================================================== 现有工具

def native_schemas(config) -> list[dict[str, Any]]:
    """照 core/conversation.py 的顺序注册全部原生工具（处理函数用占位对象，不会被调用）。"""
    from tools import build_default_registry
    from tools.car import register_car_tools
    from tools.follow import register_follow_tools
    from tools.grasp import register_grasp_tools
    from tools.identity import register_identity_tools
    from tools.music import register_music_tools
    from tools.track import register_track_tools
    from tools.vision import register_vision_tools

    stub = object()
    reg = build_default_registry()
    register_music_tools(reg, stub)
    register_car_tools(reg, stub)
    register_vision_tools(reg, stub, config=config)
    register_follow_tools(reg, stub, config=config)
    identity = stub if config.get("identity.enabled", True) else None
    register_identity_tools(reg, identity)
    vision_on = config.get("vision.enabled", True)
    if vision_on:
        register_track_tools(reg, stub, stub, None)
        register_grasp_tools(reg, stub, stub)
    return reg.schemas()


def mcp_schemas(config) -> list[dict[str, Any]]:
    """真起一次 MCPHost 取 MCP 工具 schema（weather / qqmusic），取完就关。"""
    from mcp_host import MCPHost

    log_dir = ROOT / "logs" / "arm_eval_mcp"
    log_dir.mkdir(parents=True, exist_ok=True)
    host = MCPHost(config, project_root=ROOT, log_dir=log_dir)
    try:
        host.start()
        return host.tool_schemas()
    finally:
        host.close()


# ====================================================================== schema 校验

def validate(value: Any, schema: dict[str, Any], path: str = "$") -> list[str]:
    """够用的 JSON Schema 子集校验：type/enum/required/properties（不许多余键）/items/minItems/min/max。"""
    errs: list[str] = []
    t = schema.get("type")
    if t == "object":
        if not isinstance(value, dict):
            return [f"{path} 应为 object，得到 {type(value).__name__}"]
        props = schema.get("properties", {})
        for k in schema.get("required", []):
            if k not in value:
                errs.append(f"{path} 缺 {k}")
        for k, v in value.items():
            if k not in props:
                errs.append(f"{path} 多余字段 {k}")
            else:
                errs += validate(v, props[k], f"{path}.{k}")
        return errs
    if t == "array":
        if not isinstance(value, list):
            return [f"{path} 应为 array，得到 {type(value).__name__}"]
        if len(value) < schema.get("minItems", 0):
            errs.append(f"{path} 至少 {schema['minItems']} 项")
        for i, v in enumerate(value):
            errs += validate(v, schema.get("items", {}), f"{path}[{i}]")
        return errs
    if t == "string":
        if not isinstance(value, str):
            return [f"{path} 应为 string"]
    elif t == "integer":
        if isinstance(value, bool) or not (isinstance(value, int) or (isinstance(value, float) and value.is_integer())):
            return [f"{path} 应为 integer"]
    elif t == "number":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return [f"{path} 应为 number"]
    if "enum" in schema and value not in schema["enum"]:
        errs.append(f"{path}={value!r} 不在枚举 {schema['enum']}")
    if "minimum" in schema and isinstance(value, (int, float)) and value < schema["minimum"]:
        errs.append(f"{path} 小于 {schema['minimum']}")
    if "maximum" in schema and isinstance(value, (int, float)) and value > schema["maximum"]:
        errs.append(f"{path} 大于 {schema['maximum']}")
    return errs


# ====================================================================== 判定

def atoms_of(spec: dict[str, Any]) -> list[dict[str, Any]]:
    """spec → 按执行顺序摊平的原子动作。"""
    out: list[dict[str, Any]] = []
    for i, step in enumerate(spec.get("steps") or []):
        if not isinstance(step, dict):
            continue
        for f in STEP_FIELDS:
            if f not in step:
                continue
            v = step[f]
            if f == "toward":
                out.append({"kind": "toward", "to": v, "step": i})
            elif f in ("turn", "shift"):
                for it in (v if isinstance(v, list) else [v]):
                    if isinstance(it, dict):
                        out.append({"kind": f, **it, "step": i})
            elif f == "approach":
                if isinstance(v, dict):
                    out.append({"kind": "approach", **v, "step": i})
            elif f == "gripper":
                out.append({"kind": "gripper", "value": v, "step": i})
            elif f == "oscillate":
                if isinstance(v, dict):
                    out.append({"kind": "oscillate", "times": 3, **v, "step": i})
    return out


def _val_ok(want: Any, got: Any) -> bool:
    if got is None:
        return False
    if isinstance(want, list):
        return got in want
    if isinstance(want, dict):
        if not isinstance(got, (int, float)):
            return False
        return got >= want.get("gte", float("-inf")) and got <= want.get("lte", float("inf"))
    if isinstance(want, (int, float)) and not isinstance(want, bool):
        return isinstance(got, (int, float)) and abs(got - want) <= 0.5
    return got == want


def atom_matches(pattern: dict[str, Any], atom: dict[str, Any]) -> bool:
    return all(_val_ok(v, atom.get(k)) for k, v in pattern.items())


def _check_arm_alt(alt: dict[str, Any], atoms: list[dict[str, Any]], specs: list[dict[str, Any]],
                   home_called: bool) -> str | None:
    """返回 None = 通过；否则返回失败原因。"""
    pats = alt["arm"]
    if not atoms:
        return "没有臂动作"
    if home_called:
        return "多调了 arm_home"
    if alt.get("ordered"):
        pos = -1
        for p in pats:
            idx = next((i for i, a in enumerate(atoms) if i > pos and atom_matches(p, a)), None)
            if idx is None:
                return f"按顺序找不到 {p}"
            pos = idx
    else:
        for p in pats:
            if not any(atom_matches(p, a) for a in atoms):
                return f"缺 {p}"
    if not alt.get("extra_ok"):
        for a in atoms:
            if not any(atom_matches(p, a) for p in pats):
                return f"多余动作 {a}"
    if "speed" in alt and not any(s.get("speed") == alt["speed"] for s in specs):
        return f"speed 应为 {alt['speed']}"
    return None


def judge(expect: dict[str, Any], calls: list[dict[str, Any]], arm_tool_schemas: dict[str, dict]) -> dict[str, Any]:
    """calls: [{name, args}]（整轮里模型调过的全部工具，按顺序）。"""
    alts = expect.get("any") or [expect]
    names = [c["name"] for c in calls]
    expects_arm = any("arm" in a or str(a.get("tool", "")).startswith("arm_") for a in alts)
    expects_car = all(str(a.get("tool", "")).startswith("car_") for a in alts if "tool" in a) and \
        all("tool" in a for a in alts)

    # 默认禁止：臂条目禁车动、其它条目禁臂动；forbid 再往上加
    forbid = set(CAR_MOTION) if expects_arm else set(MOTION_TOOLS)
    forbid |= set(expect.get("forbid") or [])

    # schema 校验 + 还原 spec
    specs: list[dict[str, Any]] = []
    invalid: list[str] = []
    for c in calls:
        sch = arm_tool_schemas.get(c["name"])
        if sch is None:
            continue
        errs = validate(c["args"], sch["function"]["parameters"])
        try:
            spec = to_spec(c["name"], c["args"])
        except ValueError as e:
            errs.append(str(e))
            spec = None
        if spec is not None:
            for s in spec.get("steps") or []:
                if isinstance(s, dict) and "oscillate" in s and len(s) > 1:
                    errs.append("oscillate 与其它字段同一步")
                if isinstance(s, dict) and not s:
                    errs.append("空步")
            if not spec.get("steps"):
                errs.append("没有任何动作")
            specs.append(spec)
        invalid += [f"{c['name']}: {e}" for e in errs]
    atoms = [a for s in specs for a in atoms_of(s)]

    confusion = (expects_arm and any(n in CAR_MOTION for n in names)) or \
        (expects_car and any(n in MOTION_TOOLS for n in names))
    stray_arm = (not expects_arm) and (not expects_car) and any(n in MOTION_TOOLS for n in names)

    reason = None
    hit = [n for n in names if n in forbid]
    if hit:
        reason = f"调了禁止的工具 {hit}"
    elif invalid:
        reason = "参数不合 schema: " + "; ".join(invalid[:3])
    else:
        fails = []
        for alt in alts:
            if "arm" in alt:
                r = _check_arm_alt(alt, atoms, specs, "arm_home" in names)
            elif alt.get("tool") == "none":
                bad = [n for n in names if n in ACTION_TOOLS]
                r = f"不该调动作工具，却调了 {bad}" if bad else None
            elif "tools_any" in alt:
                r = None if any(n in alt["tools_any"] for n in names) else f"应调 {alt['tools_any']} 之一"
            else:
                r = None if alt["tool"] in names else f"应调 {alt['tool']}"
            if r is None:
                fails = None
                break
            fails.append(r)
        if fails:
            reason = " | ".join(fails)
    return {"pass": reason is None, "reason": reason, "confusion": confusion, "stray_arm": stray_arm,
            "atoms": atoms}


# ====================================================================== 跑模型

# 臂动作的假结果按调用内容现编（"做了什么"），不带具体姿态读数——
# 固定读数会和请求自相矛盾（让往上看却回"向下 30°"），模型会以为没做成而重调，污染评测。
_ATOM_TEXT = {
    "toward": lambda a: f"看向{a.get('to')}",
    "turn": lambda a: f"视线往{a.get('dir')}转了" + (f"{a['degrees']}°" if "degrees" in a else str(a.get("amount", "一些"))),
    "shift": lambda a: f"爪子往{a.get('dir')}挪了" + (f"{a['cm']}cm" if "cm" in a else str(a.get("amount", "一些"))),
    "approach": lambda a: f"沿视线{a.get('dir')}了" + (f"{a['cm']}cm" if "cm" in a else str(a.get("amount", "一些"))),
    "gripper": lambda a: f"夹爪{a.get('value')}",
    "oscillate": lambda a: f"{a.get('axis')}来回了{a.get('times', 3)}下",
}


def _arm_done(name: str, args: dict[str, Any]) -> str:
    try:
        spec = to_spec(name, args) or {}
        parts = [_ATOM_TEXT[a["kind"]](a) for a in atoms_of(spec)]
    except Exception:  # noqa: BLE001
        parts = []
    return ("做完了：" + "，".join(parts) + "。") if parts else "参数不完整，没有动。"
_FAKE = {
    "arm_state": "视线朝前、向下 15°；爪尖在车前 15cm、高 10cm；夹爪合上。",
    "arm_home": "已回到初始位置，夹爪保持合上。",
    "car_status": "电池 12.1V，无故障，车静止。",
    "look": "画面里是一张木桌，上面有一个白色杯子。",
    "mcp__weather__get_current_weather": "大庆 晴 8°C 西北风 3 级",
    "mcp__weather__get_forecast": "大庆 未来三天晴转多云 2~10°C",
    "grasp_object": "抓到了，已经回到初始位置。",
    "follow_person": "已经跟上了。",
    "track_object": "已经盯住了。",
}


def fake_result(name: str, args: dict[str, Any] | None = None) -> str:
    if name in ("arm_look", "arm_move", "arm_act"):
        return _arm_done(name, args or {})
    if name == "get_current_time":
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return _FAKE.get(name, "已完成。")


def render_history(variant: str, history: list[dict[str, Any]]) -> list[dict[str, Any]]:
    msgs: list[dict[str, Any]] = []
    for h_i, h in enumerate(history or []):
        msgs.append({"role": "user", "content": h["user"]})
        if "spec" in h:
            calls = from_spec(variant, h["spec"])
        else:
            calls = [(h["call"]["name"], h["call"]["args"])]
        tcs = [{"id": f"call_h{h_i}_{j}", "type": "function",
                "function": {"name": n, "arguments": json.dumps(a, ensure_ascii=False)}}
               for j, (n, a) in enumerate(calls)]
        msgs.append({"role": "assistant", "content": "", "tool_calls": tcs})
        for j, tc in enumerate(tcs):
            msgs.append({"role": "tool", "tool_call_id": tc["id"],
                         "content": h["result"] if j == len(tcs) - 1 else "已完成。"})
        msgs.append({"role": "assistant", "content": h.get("reply", "好的。")})
    return msgs


def run_one(client, llm: dict[str, Any], tools: list[dict[str, Any]], variant: str,
            item: dict[str, Any]) -> dict[str, Any]:
    messages = [{"role": "system", "content": llm["system_prompt"]}]
    messages += render_history(variant, item.get("history") or [])
    messages.append({"role": "user", "content": item["say"]})
    calls: list[dict[str, Any]] = []
    text = ""
    error = None
    for _ in range(MAX_ROUNDS):
        try:
            resp = client.chat.completions.create(
                model=llm["model"], messages=messages, tools=tools,
                temperature=llm["temperature"], max_tokens=llm["max_tokens"])
        except Exception as e:  # noqa: BLE001
            error = f"{type(e).__name__}: {str(e)[:200]}"
            break
        msg = resp.choices[0].message
        text += msg.content or ""
        tcs = msg.tool_calls or []
        if not tcs:
            break
        amsg: dict[str, Any] = {"role": "assistant", "content": msg.content or "", "tool_calls": [
            {"id": tc.id, "type": "function",
             "function": {"name": tc.function.name, "arguments": tc.function.arguments}} for tc in tcs]}
        rc = getattr(msg, "reasoning_content", None)
        if rc:
            amsg["reasoning_content"] = rc
        messages.append(amsg)
        stop = False
        for tc in tcs:
            try:
                args = json.loads(tc.function.arguments or "{}")
            except json.JSONDecodeError:
                args = {"__unparsable__": tc.function.arguments}
            calls.append({"name": tc.function.name, "args": args})
            messages.append({"role": "tool", "tool_call_id": tc.id, "content": fake_result(tc.function.name, args)})
            if tc.function.name == "ask_user":
                stop = True
        if stop:
            break
    return {"calls": calls, "text": text, "error": error}


# ====================================================================== 主程序

def summarize(results: list[dict[str, Any]]) -> dict[str, Any]:
    total = len(results)
    passed = sum(r["pass"] for r in results)
    by_cat: dict[str, list[int]] = {}
    for r in results:
        c = CATEGORIES[r["id"][0]]
        by_cat.setdefault(c, [0, 0])
        by_cat[c][0] += r["pass"]
        by_cat[c][1] += 1
    return {
        "total": total, "passed": passed, "rate": round(passed / total, 4) if total else 0,
        "confusion": sum(r["confusion"] for r in results),
        "stray_arm": sum(r["stray_arm"] for r in results),
        "errors": sum(1 for r in results if r.get("error")),
        "by_category": {c: {"passed": p, "total": t, "rate": round(p / t, 4)} for c, (p, t) in by_cat.items()},
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", required=True, choices=["A", "B", "a", "b"])
    ap.add_argument("--runs", type=int, default=3)
    ap.add_argument("--only", default="", help="逗号分隔的 id，只跑这些")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--config", default=str(ROOT / "config.yaml"))
    ap.add_argument("--no-mcp", action="store_true", help="不起 MCP（只在调试时用，正式评测别加）")
    args = ap.parse_args()
    variant = args.variant.upper()

    from loguru import logger
    logger.remove()
    logger.add(sys.stderr, level="WARNING")

    from openai import OpenAI
    from utils.config import Config

    config = Config(args.config)
    llm = {
        "model": config.get("llm.model", "deepseek-v4-flash"),
        "temperature": config.get("llm.temperature", 0.7),
        "max_tokens": config.get("llm.max_tokens", 2048),
        "system_prompt": (config.get("llm.system_prompt", "") or "").strip(),
    }
    client = OpenAI(api_key=config.get("llm.api_key"),
                    base_url=config.get("llm.base_url", "https://api.deepseek.com"))

    native = native_schemas(config)
    mcp = [] if args.no_mcp else mcp_schemas(config)
    arm = arm_schemas(variant)
    tools = native + mcp + arm
    arm_by_name = {s["function"]["name"]: s for s in arm}
    print(f"[{variant}] model={llm['model']} T={llm['temperature']} 工具 {len(tools)} 个 "
          f"(原生 {len(native)} + MCP {len(mcp)} + 臂 {len(arm)})", flush=True)

    corpus = yaml.safe_load(CORPUS.read_text(encoding="utf-8"))
    if args.only:
        keep = set(args.only.split(","))
        corpus = [c for c in corpus if c["id"] in keep]

    jobs = [(item, k) for item in corpus for k in range(args.runs)]
    results: list[dict[str, Any]] = []
    t0 = time.time()
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(run_one, client, llm, tools, variant, item): (item, k) for item, k in jobs}
        for fut in concurrent.futures.as_completed(futs):
            item, k = futs[fut]
            out = fut.result()
            if out["error"]:
                j = {"pass": False, "reason": "API 错误: " + out["error"], "confusion": False,
                     "stray_arm": False, "atoms": []}
            else:
                j = judge(item["expect"], out["calls"], arm_by_name)
            results.append({"id": item["id"], "run": k, "say": item["say"], **out, **j})

    results.sort(key=lambda r: (r["id"], r["run"]))
    summary = summarize(results)
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir = ROOT / "logs"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / f"arm_eval-{variant}-{ts}.json"
    out_path.write_text(json.dumps({
        "variant": variant, "model": llm["model"], "temperature": llm["temperature"], "runs": args.runs,
        "n_tools": len(tools), "tool_names": [t["function"]["name"] for t in tools],
        "arm_schemas": arm, "summary": summary, "results": results,
    }, ensure_ascii=False, indent=1), encoding="utf-8")

    for r in results:
        if not r["pass"]:
            brief = [f"{c['name']}({json.dumps(c['args'], ensure_ascii=False)})" for c in r["calls"]]
            print(f"  ✗ {r['id']}#{r['run']} 「{r['say']}」 → {brief or r['text'][:40]!r} :: {r['reason']}")
    print(f"[{variant}] 通过 {summary['passed']}/{summary['total']} = {summary['rate']:.1%}  "
          f"车/臂混淆 {summary['confusion']}  非臂条目误动臂 {summary['stray_arm']}  "
          f"API 错误 {summary['errors']}  用时 {time.time() - t0:.0f}s")
    for c, v in summary["by_category"].items():
        print(f"    {c}: {v['passed']}/{v['total']} = {v['rate']:.1%}")
    print(f"结果: {out_path.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
