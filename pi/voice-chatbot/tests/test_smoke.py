"""脚手架自检：确认 pytest 能发现测试、项目根在 sys.path 上。

注意：这里**不能**写成 `assert str(_ROOT) in sys.path or _ROOT.exists()` ——
`_ROOT` 是本文件的父目录的父目录，必然存在，`or` 那半边恒真，测试永远不会失败，
等于没测。第一版就是这么写的。
"""

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent


def test_project_root_is_on_sys_path():
    """后续每个测试都靠「从项目根 import」，这里把它钉住。"""
    assert str(_ROOT) in sys.path


def test_can_import_project_modules():
    """光看 sys.path 还不够 —— 真的 import 到项目模块，才说明路走通了。"""
    import tools.registry  # noqa: F401
    import utils.config  # noqa: F401
