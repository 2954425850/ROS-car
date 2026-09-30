"""天气 MCP server —— 自写，默认免注册免 key（Open-Meteo）。

为什么自己写一个而不是找个现成的：
  1. 现成的天气 MCP 基本都要 OpenWeatherMap / 和风天气的 key，会卡在注册上；
  2. 更要紧的是：实测第三方 QQ音乐 server 是 **0 resources / 0 prompts**。
     如果只用它，resources / prompts 这两条通路就**没有任何真实测试对象**，
     等于写了没验证。所以本服务器**故意**同时暴露一个 tool、一个 resource
     和一个 prompt —— 让协议的三条服务器能力都有真东西可测。

配置由 MCPHost 通过环境变量 MCP_SERVER_CONFIG（JSON）注入，这样 host 不需要知道
任何 server 私有字段名：

    {"city": "大庆", "qweather_key": ""}

qweather_key 留空 → 走 Open-Meteo（免 key，全球数据）。
填了 → 走和风天气 v7（国内数据更准，且带灾害预警接口的扩展余地）。
注意：和风那条分支在没有 key 的环境下无法被自动测试，属于「配了才走」的路径。
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.parse
import urllib.request

from mcp.server.fastmcp import FastMCP

mcp = FastMCP("weather")

_HTTP_TIMEOUT = 12
_DEFAULT_CITY = "大庆"

# ------------------------------------------------------------------ 配置

def _server_config() -> dict:
    raw = os.environ.get("MCP_SERVER_CONFIG") or "{}"
    try:
        data = json.loads(raw)
        return data if isinstance(data, dict) else {}
    except json.JSONDecodeError:
        return {}


def _default_city() -> str:
    return str(_server_config().get("city") or _DEFAULT_CITY)


def _qweather_key() -> str:
    return str(_server_config().get("qweather_key") or "").strip()


# ------------------------------------------------------------------ HTTP

def _get_json(url: str, params: dict[str, object]) -> dict:
    query = urllib.parse.urlencode({k: v for k, v in params.items() if v is not None})
    full = f"{url}?{query}"
    req = urllib.request.Request(full, headers={"User-Agent": "voice-chatbot-weather/1.0"})
    with urllib.request.urlopen(req, timeout=_HTTP_TIMEOUT) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _geocode(city: str) -> tuple[float, float, str]:
    """城市名 → (纬度, 经度, 解析出的规范名)。"""
    data = _get_json(
        "https://geocoding-api.open-meteo.com/v1/search",
        {"name": city, "count": 1, "language": "zh", "format": "json"},
    )
    results = data.get("results") or []
    if not results:
        raise LookupError(f"找不到城市「{city}」")
    hit = results[0]
    name = str(hit.get("name") or "")
    # 附上省/国家做限定，否则「大庆」和「大同」这种光看市名分不清。
    # 注意市名本身必须保留 —— 早先的写法把它一起过滤掉了，返回的是「黑龙江中国」。
    qualifiers = [str(hit[k]) for k in ("admin1", "country") if hit.get(k) and str(hit[k]) != name]
    label = "·".join([name, *qualifiers]) if name else f"{hit['latitude']},{hit['longitude']}"
    return float(hit["latitude"]), float(hit["longitude"]), label


# WMO 天气现象代码 → 中文（Open-Meteo 用的就是这套）
_WMO = {
    0: "晴", 1: "晴间多云", 2: "多云", 3: "阴",
    45: "有雾", 48: "雾凇",
    51: "小毛毛雨", 53: "毛毛雨", 55: "大毛毛雨",
    56: "冻毛毛雨", 57: "强冻毛毛雨",
    61: "小雨", 63: "中雨", 65: "大雨",
    66: "冻雨", 67: "强冻雨",
    71: "小雪", 73: "中雪", 75: "大雪", 77: "雪粒",
    80: "小阵雨", 81: "中阵雨", 82: "强阵雨",
    85: "小阵雪", 86: "大阵雪",
    95: "雷阵雨", 96: "雷阵雨伴小冰雹", 99: "雷阵雨伴大冰雹",
}


def _describe(code: object) -> str:
    try:
        return _WMO.get(int(code), f"未知天气(代码 {code})")
    except (TypeError, ValueError):
        return f"未知天气(代码 {code})"


def _at(values: object, index: int) -> object:
    """Open-Meteo 的日预报是若干**并行数组**。某个数组短了也不该让整次查询炸掉。"""
    try:
        return values[index]  # type: ignore[index]
    except (TypeError, IndexError, KeyError):
        return None


# ------------------------------------------------------------------ 取数

def _open_meteo_current(city: str) -> str:
    lat, lon, label = _geocode(city)
    data = _get_json(
        "https://api.open-meteo.com/v1/forecast",
        {
            "latitude": lat, "longitude": lon,
            "current": "temperature_2m,relative_humidity_2m,apparent_temperature,"
                       "weather_code,wind_speed_10m",
            "timezone": "auto",
        },
    )
    cur = data.get("current") or {}
    return (
        f"{label} 当前天气：{_describe(cur.get('weather_code'))}，"
        f"气温 {cur.get('temperature_2m')}°C，体感 {cur.get('apparent_temperature')}°C，"
        f"湿度 {cur.get('relative_humidity_2m')}%，"
        f"风速 {cur.get('wind_speed_10m')} km/h。"
    )


def _open_meteo_forecast(city: str, days: int) -> str:
    lat, lon, label = _geocode(city)
    data = _get_json(
        "https://api.open-meteo.com/v1/forecast",
        {
            "latitude": lat, "longitude": lon,
            "daily": "weather_code,temperature_2m_max,temperature_2m_min,"
                     "precipitation_probability_max",
            "timezone": "auto", "forecast_days": max(1, min(days, 16)),
        },
    )
    daily = data.get("daily") or {}
    dates = daily.get("time") or []
    if not dates:
        return f"{label} 暂时拿不到逐日预报。"
    codes = daily.get("weather_code")
    lows = daily.get("temperature_2m_min")
    highs = daily.get("temperature_2m_max")
    precip = daily.get("precipitation_probability_max")
    lines = [f"{label} 未来 {len(dates)} 天预报："]
    for i, day in enumerate(dates):
        lines.append(
            f"  {day}：{_describe(_at(codes, i))}，"
            f"{_at(lows, i)}~{_at(highs, i)}°C，"
            f"降水概率 {_at(precip, i)}%"
        )
    return "\n".join(lines)


def _qweather(current: bool, city: str, days: int = 3) -> str:
    key = _qweather_key()
    lat, lon, label = _geocode(city)
    endpoint = "now" if current else f"{max(1, min(days, 30))}d"
    data = _get_json(
        f"https://devapi.qweather.com/v7/weather/{endpoint}",
        {"location": f"{lon:.4f},{lat:.4f}", "key": key, "lang": "zh"},
    )
    if str(data.get("code")) != "200":
        raise RuntimeError(f"和风天气返回错误码 {data.get('code')}")
    if current:
        now = data.get("now") or {}
        return (
            f"{label} 当前天气：{now.get('text')}，气温 {now.get('temp')}°C，"
            f"体感 {now.get('feelsLike')}°C，湿度 {now.get('humidity')}%，"
            f"{now.get('windDir')}{now.get('windScale')}级。"
        )
    lines = [f"{label} 未来 {days} 天预报："]
    for day in data.get("daily") or []:
        lines.append(
            f"  {day.get('fxDate')}：{day.get('textDay')}，"
            f"{day.get('tempMin')}~{day.get('tempMax')}°C，"
            f"降水概率 {day.get('precip')}%"
        )
    return "\n".join(lines)


# ------------------------------------------------------------------ 工具

@mcp.tool()
def get_current_weather(city: str = "") -> str:
    """查询某个城市的实时天气（气温/体感/湿度/风速/天气现象）。

    Args:
        city: 城市中文名，例如「大庆」「哈尔滨」「北京」。留空则用配置里的默认城市。
    """
    target = (city or "").strip() or _default_city()
    try:
        if _qweather_key():
            return _qweather(current=True, city=target)
        return _open_meteo_current(target)
    except LookupError as exc:
        return f"查不到城市「{target}」：{exc}。请确认城市名，或换一个更常见的叫法。"
    except (urllib.error.URLError, TimeoutError) as exc:
        return f"天气服务暂时连不上（{exc}），稍后再试。"
    except Exception as exc:  # noqa: BLE001
        return f"查询天气失败：{exc}"


@mcp.tool()
def get_forecast(city: str = "", days: int = 3) -> str:
    """查询某个城市未来几天的天气预报。

    Args:
        city: 城市中文名，例如「大庆」「哈尔滨」「北京」。留空则用配置里的默认城市。
        days: 预报天数，默认 3 天，最多 16 天。
    """
    target = (city or "").strip() or _default_city()
    try:
        if _qweather_key():
            return _qweather(current=False, city=target, days=days)
        return _open_meteo_forecast(target, days)
    except LookupError as exc:
        return f"查不到城市「{target}」：{exc}。请确认城市名，或换一个更常见的叫法。"
    except (urllib.error.URLError, TimeoutError) as exc:
        return f"天气服务暂时连不上（{exc}），稍后再试。"
    except Exception as exc:  # noqa: BLE001
        return f"查询预报失败：{exc}"


# ------------------------------------------------------------------ 资源

@mcp.resource("weather://current")
def default_city_weather_resource() -> str:
    """默认城市的实时天气。无参数 —— 免去调用方构造 URI 的麻烦。"""
    return get_current_weather("")


@mcp.resource("weather://{city}/current")
def current_weather_resource(city: str) -> str:
    """指定城市的实时天气，作为可读取的资源暴露。

    这样别的 MCP 客户端可以直接 read 而不必调工具 —— 工具的返回值要进模型上下文，
    资源则可以被订阅/缓存。两条通路的语义不一样。
    """
    # 模板变量取自 URI 路径段，中文会被百分号编码成 %E5%A4%A7%E5%BA%86 这种形式，
    # 必须解码后再当地名用（实测踩到过）。
    return get_current_weather(urllib.parse.unquote(city))


# ------------------------------------------------------------------ 提示

@mcp.prompt()
def weather_briefing(city: str = "") -> str:
    """生成一段「用口语播报天气」的提示模板。"""
    target = (city or "").strip() or _default_city()
    return (
        f"请查询 {target} 的实时天气和未来三天预报，然后用口语化的中文播报给用户。"
        f"要求：先报今天的天气和气温，再提一句明天会不会下雨，"
        f"最后给一句穿衣或带伞的建议。控制在三句话以内，不要念出数据字段名。"
    )


if __name__ == "__main__":
    mcp.run()  # 默认 stdio 传输
