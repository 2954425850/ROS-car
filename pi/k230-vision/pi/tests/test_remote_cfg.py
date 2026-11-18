"""K230 板端 remote_cfg.validate 的纯函数测试（不碰 SD/网络）。

板端代码在 pi/k230-board/，不在这个包的 import 路径上，用 importlib 按文件加载。
"""
import importlib.util
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_PI = os.path.dirname(os.path.dirname(os.path.dirname(_HERE)))
_BOARD_FILE = os.path.join(_REPO_PI, "k230-board", "remote_cfg.py")

_spec = importlib.util.spec_from_file_location("k230_remote_cfg", _BOARD_FILE)
rc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rc)


def test_accepts_ip_and_hostname():
    changes, err = rc.validate({
        "cmd": "config_set",
        "push_host": "10.64.35.7",
        "result_host": "raspberrypi.local",
    })
    assert err is None
    assert changes == {"push_host": "10.64.35.7",
                       "result_host": "raspberrypi.local"}


def test_accepts_wifi():
    changes, err = rc.validate({"cmd": "config_set",
                                "wifi_ssid": "My WiFi",
                                "wifi_pass": "secret123"})
    assert err is None
    assert changes["wifi_ssid"] == "My WiFi"
    assert changes["wifi_pass"] == "secret123"


def test_obstacle_host_null_means_follow_result_host():
    changes, err = rc.validate({"cmd": "config_set", "obstacle_host": None})
    assert err is None
    assert changes == {"obstacle_host": None}


def test_rejects_unknown_key():
    # 拼错一个字母必须报错，不能"设了但没生效"
    changes, err = rc.validate({"cmd": "config_set", "result_hots": "1.2.3.4"})
    assert changes is None
    assert "unknown key" in err


def test_rejects_misspelled_ip():
    changes, err = rc.validate({"cmd": "config_set", "push_host": "10.64.35"})
    assert changes is None
    assert "not a valid" in err


def test_rejects_out_of_range_ip():
    changes, err = rc.validate({"cmd": "config_set", "result_host": "10.0.0.256"})
    assert changes is None


def test_rejects_empty_or_non_string_host():
    assert rc.validate({"cmd": "config_set", "push_host": ""})[0] is None
    assert rc.validate({"cmd": "config_set", "push_host": 123})[0] is None


def test_rejects_short_wifi_pass():
    # 1..7 位 WPA 密码是典型的打字错误；放过去板子就会失联
    changes, err = rc.validate({"cmd": "config_set", "wifi_pass": "short"})
    assert changes is None
    assert "8..63" in err


def test_allows_empty_pass_open_network():
    changes, err = rc.validate({"cmd": "config_set", "wifi_pass": ""})
    assert err is None
    assert changes == {"wifi_pass": ""}


def test_rejects_long_ssid():
    changes, err = rc.validate({"cmd": "config_set", "wifi_ssid": "x" * 33})
    assert changes is None
    assert "1..32" in err


def test_ssid_limit_is_bytes_not_chars():
    # WiFi SSID 上限 32 **字节**；11 个汉字 = 33 字节，必须拒
    assert rc.validate({"cmd": "config_set", "wifi_ssid": "中" * 11})[0] is None
    assert rc.validate({"cmd": "config_set", "wifi_ssid": "中" * 10})[1] is None


def test_rejects_control_chars():
    assert rc.validate({"cmd": "config_set", "wifi_ssid": "a\nb"})[0] is None
    assert rc.validate({"cmd": "config_set", "wifi_pass": "abcdefg\n"})[0] is None


def test_requires_something_to_set():
    changes, err = rc.validate({"cmd": "config_set"})
    assert changes is None
    assert "nothing to set" in err


def test_rejects_non_dict():
    assert rc.validate(["nope"])[0] is None
