import sys
from pathlib import Path

# 让测试以 `from encode import ...` / `from server import ...` 导入，
# 和 server.py 在 Pi 上以脚本方式运行时的导入方式保持一致。
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
