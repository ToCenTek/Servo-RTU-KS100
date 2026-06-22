#!/usr/bin/env python3
"""KS100 Modbus RTU 通信参数自动探测.
从 slave=1 开始,逐个尝试到 SLAVE_MAX=254,找到第一个有效响应即停止.
假设总线上只有 1 个设备.
"""
import sys
import json
import struct

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    INSTALL_CMD = "python3 -m pip install pyserial --break-system-packages"
    msg = "Python pyserial 库未安装。\n请运行: " + INSTALL_CMD
    with open("/tmp/probe_result.json", 'w') as f:
        json.dump({"success": False, "error": msg}, f)
    print(msg, file=sys.stderr)
    sys.exit(1)

BAUD_RATES = [115200, 57600, 38400, 19200, 9600, 4800]
MODE_MAP = {0: "8N2", 1: "8E1", 2: "8O1", 3: "8N1"}
MODES = {
    "8N1": (8, 'N', 1), "8N2": (8, 'N', 2),
    "8E1": (8, 'E', 1), "8O1": (8, 'O', 1),
}
SLAVE_MAX = 254  # Modbus 最大地址
TIMEOUT = 0.04


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = ((crc >> 1) ^ 0xA001) & 0xFFFF
            else:
                crc = (crc >> 1) & 0xFFFF
    return struct.pack('<H', crc)


def try_mode(port, baud, mode_key, slave):
    """Modbus 0x03 读 0x0047~0x0049. 成功返回 dict, 失败返回 None."""
    db, par, sb = MODES[mode_key]
    try:
        ser = serial.Serial(port, baud, bytesize=db, parity=par,
                            stopbits=sb, timeout=TIMEOUT)
        body = struct.pack('>BBHH', slave, 0x03, 0x0047, 3)
        ser.write(body + crc16(body))
        resp = ser.read(256)
        ser.close()
        if len(resp) >= 11 and resp[1] == 0x03:
            fa72 = resp[5] * 256 + resp[6]
            fa73 = resp[7] * 256 + resp[8]
            if fa73 < 0 or fa73 > 3:
                fa73 = -1
            return {"success": True, "baud": baud, "slave": slave,
                    "protocol": MODE_MAP.get(fa73, "8N1"),
                    "fa72": fa72, "fa73": fa73, "port": port}
    except Exception:
        pass
    return None


def main():
    port = None
    output = "/tmp/probe_result.json"

    i = 0
    while i < len(sys.argv):
        a = sys.argv[i]
        if a == '--port' and i + 1 < len(sys.argv):
            port = sys.argv[i + 1]; i += 2
        elif a == '--output' and i + 1 < len(sys.argv):
            output = sys.argv[i + 1]; i += 2
        else:
            i += 1

    try:
        ports = [port] if port else [p.device for p in serial.tools.list_ports.comports()]
    except Exception:
        ports = []

    if not ports:
        with open(output, 'w') as f:
            json.dump({"success": False, "error": "未检测到串口设备"}, f)
        sys.exit(1)

    # 从 slave=1 开始, 逐个尝试到 SLAVE_MAX, 找到第一个有效响应即停止.
    for slave in range(1, SLAVE_MAX + 1):
        for baud in BAUD_RATES:
            for mode_key in ["8N1", "8N2", "8E1", "8O1"]:
                for p in ports:
                    r = try_mode(p, baud, mode_key, slave)
                    if r:
                        with open(output, 'w') as f:
                            json.dump(r, f)
                        print(json.dumps(r))
                        sys.exit(0)

    with open(output, 'w') as f:
        json.dump({"success": False, "error": "未找到伺服 (扫 slave 1~{})".format(SLAVE_MAX)}, f)
    sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        with open("/tmp/probe_result.json", 'w') as f:
            json.dump({"success": False, "error": "脚本异常: " + str(e)}, f)
        sys.exit(1)
