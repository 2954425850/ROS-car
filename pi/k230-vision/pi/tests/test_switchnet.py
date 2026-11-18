"""switchnet 的纯函数测试（不碰 nmcli / 网络）。"""
import switchnet


def test_hosts_in_24():
    hosts = switchnet.hosts_in_prefix("10.64.35.7", 24)
    assert len(hosts) == 254
    assert "10.64.35.1" in hosts
    assert "10.64.35.254" in hosts
    assert "10.64.35.0" not in hosts      # 网络地址
    assert "10.64.35.255" not in hosts    # 广播地址


def test_hosts_capped_to_24():
    # /16 不扫 6 万个地址 —— 只扫当前 /24
    hosts = switchnet.hosts_in_prefix("10.64.35.7", 16)
    assert len(hosts) == 254
    assert all(h.startswith("10.64.35.") for h in hosts)


def test_plan_wifi_only():
    p = switchnet.make_set_plan("NewSSID", "secret123", None, {})
    assert p == {"cmd": "config_set", "wifi_ssid": "NewSSID",
                 "wifi_pass": "secret123"}


def test_plan_omits_password_when_unknown():
    p = switchnet.make_set_plan("NewSSID", None, None, {})
    assert "wifi_pass" not in p


def test_plan_with_new_host():
    p = switchnet.make_set_plan("S", "p" * 8, "10.0.0.9",
                                {"result_host": "10.0.0.5", "push_host": "10.0.0.5",
                                 "obstacle_host": None})
    assert p["result_host"] == "10.0.0.9"
    assert p["push_host"] == "10.0.0.9"
    # 原来跟随 result_host（null）：不需要显式写，继续保持跟随
    assert "obstacle_host" not in p


def test_plan_makes_old_explicit_obstacle_follow_again():
    # 旧配置里 obstacle_host 显式写成了旧 Pi IP -> 改成 null，跟随新地址
    p = switchnet.make_set_plan(None, None, "10.0.0.9",
                                {"result_host": "10.0.0.5",
                                 "obstacle_host": "10.0.0.5"})
    assert p["obstacle_host"] is None


def test_plan_leaves_third_party_obstacle_host_alone():
    p = switchnet.make_set_plan(None, None, "10.0.0.9",
                                {"result_host": "10.0.0.5",
                                 "obstacle_host": "10.0.0.42"})
    assert "obstacle_host" not in p


def test_mask_plan_hides_password():
    p = switchnet.mask_plan({"cmd": "config_set", "wifi_ssid": "S",
                             "wifi_pass": "secret123"})
    assert p["wifi_pass"] == "***"
    assert p["wifi_ssid"] == "S"
