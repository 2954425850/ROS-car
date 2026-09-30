# -*- coding: utf-8 -*-
"""Linux joystick (/dev/input/jsN) 零依赖读取（kernel joydev 已把轴归一到 ±32767）。

js_event 8 字节: time(u32) value(i16) type(u8) number(u8)
type: 0x01=按键 0x02=轴 0x80=初始状态位
轴编号按 joydev 顺序（本手柄 6 轴）: 0=左X 1=左Y 2=右X 3=右Y 4=十字X 5=十字Y
"""
import os
import struct
import threading
import time

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

# ShanWan 2563:0575 常见键序（猜测，--probe 实测后确认）
BUTTON_HINTS = {0: '×?', 1: '○?', 2: '?(L3?)', 3: '□?', 4: '△?', 5: '?(R3?)',
                6: 'L1', 7: 'R1', 8: 'L2', 9: 'R2', 10: 'SELECT', 11: 'START', 12: 'MODE'}


class JsState:
    """手柄共享状态（JsReader 写，节点线程读）。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.buttons = {}   # {编号: 0/1}
        self.axes = {}      # {编号: -32767..32767}
        self.last_event = time.monotonic()


class JsReader(threading.Thread):
    """后台线程持续读 js 设备；设备拔出自动重开。"""

    def __init__(self, path: str, state: JsState):
        super().__init__(daemon=True)
        self.path = path
        self.state = state

    def run(self):
        while True:
            try:
                fd = os.open(self.path, os.O_RDONLY)
            except OSError:
                time.sleep(1.0)
                continue
            try:
                while True:
                    data = os.read(fd, 8)
                    if len(data) < 8:
                        continue
                    _t, value, typ, num = struct.unpack('<ihBB', data)
                    ev = typ & 0x7F
                    with self.state.lock:
                        if ev == JS_EVENT_BUTTON:
                            self.state.buttons[num] = 1 if value else 0
                        elif ev == JS_EVENT_AXIS:
                            self.state.axes[num] = value
                        self.state.last_event = time.monotonic()
            except OSError:
                pass
            finally:
                try:
                    os.close(fd)
                except OSError:
                    pass
            with self.state.lock:
                self.state.buttons.clear()
                self.state.axes.clear()
            time.sleep(1.0)   # 设备消失，稍后重开

    def stop(self):
        pass  # daemon 线程随进程退出


def run_probe(path: str):
    """--probe：打印原始事件用于核对键号/轴号。"""
    import sys
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError as e:
        print(f'打不开 {path}: {e}', file=sys.stderr)
        sys.exit(1)
    print(f'探测 {path} —— 依次按各按键/摇杆/十字键，记下编号填入 CFG（Ctrl-C 退出）')
    print('猜测键序: ' + ', '.join(f'{k}={v}' for k, v in sorted(BUTTON_HINTS.items())))
    try:
        while True:
            data = os.read(fd, 8)
            if len(data) < 8:
                continue
            _t, value, typ, num = struct.unpack('<ihBB', data)
            init = ' <初始状态>' if typ & JS_EVENT_INIT else ''
            ev = typ & 0x7F
            if ev == JS_EVENT_BUTTON:
                print(f'BUTTON {num:2d}  {"按下" if value else "松开"}  '
                      f'{BUTTON_HINTS.get(num, "?")}{init}')
            elif ev == JS_EVENT_AXIS:
                print(f'AXIS   {num:2d}  {value:+7d} ({value / 32767:+.2f}){init}')
    except KeyboardInterrupt:
        print('\n探测结束')
