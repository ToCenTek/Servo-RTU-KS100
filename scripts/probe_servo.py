#!/usr/bin/env python3
"""KS100 Modbus RTU 通信参数自动探测 + 强制 8N1.
从 slave=1 开始,逐个尝试到 SLAVE_MAX=254,找到第一个有效响应即停止.
找到后自动写 FA-73=3 (8N1) + FA-60=1 (软复位) 让 servo 跟 Chataigne 端匹配.
"""
import sys
import json
import struct
import time

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
SLAVE_MAX = 254
TIMEOUT = 0.04
RESET_WAIT = 3.0  # 软复位后等 servo 重启


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


def modbus_read(ser, slave, reg, count):
    """发送 Modbus 0x03 读, 返回 bytes 或 None."""
    body = struct.pack('>BBHH', slave, 0x03, reg, count)
    ser.write(body + crc16(body))
    return ser.read(256)


def modbus_write(ser, slave, reg, value):
    """发送 Modbus 0x06 写, 返回响应 bytes 或 None."""
    body = struct.pack('>BBHH', slave, 0x06, reg, value)
    ser.write(body + crc16(body))
    return ser.read(8)


def try_mode(port, baud, mode_key, slave):
    """Modbus 0x03 读 0x0047~0x0049. 成功返回 dict, 失败返回 None."""
    db, par, sb = MODES[mode_key]
    try:
        ser = serial.Serial(port, baud, bytesize=db, parity=par,
                            stopbits=sb, timeout=TIMEOUT)
        resp = modbus_read(ser, slave, 0x0047, 3)
        ser.close()
        if len(resp) >= 11 and resp[1] == 0x03:
            fa71 = resp[3] * 256 + resp[4]   # 从站地址
            fa72 = resp[5] * 256 + resp[6]   # 波特率 (值 = baud/100)
            fa73 = resp[7] * 256 + resp[8]   # 协议 (3=8N1)
            if fa73 not in MODE_MAP:
                fa73 = -1
            return {"success": True, "baud": baud, "slave": fa71,
                    "protocol": MODE_MAP.get(fa73, "?"),
                    "fa72": fa72, "fa73": fa73, "port": port,
                    "detected_mode": mode_key}
    except Exception:
        pass
    return None


def force_8n1(port, baud, slave):
    """写 FA-73=3 (8N1) + FA-60=1 (软复位), 等 servo 重启.
    返回 True (成功) / False (失败)."""
    try:
        ser = serial.Serial(port, baud, bytesize=8, parity='N', stopbits=1, timeout=TIMEOUT)
        # 写 FA-73 = 3
        r1 = modbus_write(ser, slave, 0x0049, 3)
        # 写 FA-60 = 1 (软复位)
        r2 = modbus_write(ser, slave, 0x003C, 1)
        ser.close()
        ok1 = len(r1) >= 8 and r1[1] == 0x06
        ok2 = len(r2) >= 8 and r2[1] == 0x06
        if not (ok1 and ok2):
            return False
        time.sleep(RESET_WAIT)
        return True
    except Exception as e:
        print("force_8n1 error: " + str(e), file=sys.stderr)
        return False


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

    # 第一阶段: 扫描所有 baud × mode × slave
    found = None
    for slave in range(1, SLAVE_MAX + 1):
        for baud in BAUD_RATES:
            for mode_key in ["8N1", "8N2", "8E1", "8O1"]:
                for p in ports:
                    r = try_mode(p, baud, mode_key, slave)
                    if r:
                        found = r
                        break
                if found: break
            if found: break
        if found: break

    if not found:
        with open(output, 'w') as f:
            json.dump({"success": False, "error": "未找到伺服 (扫 slave 1~{})".format(SLAVE_MAX)}, f)
        sys.exit(1)

    # 第二阶段: 强制 servo 为 8N1
    if found["fa73"] != 3:
        print("Forcing servo to 8N1 (was {})".format(found["protocol"]), file=sys.stderr)
        ok = force_8n1(found["port"], found["baud"], found["slave"])
        if not ok:
            with open(output, 'w') as f:
                json.dump({"success": False, "error": "Found servo but force 8N1 failed"}, f)
            sys.exit(1)
        found["forced"] = True
    else:
        found["forced"] = False

    # 写 result (servo 现在是 8N1)
    with open(output, 'w') as f:
        json.dump(found, f)
    print(json.dumps(found))


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        with open("/tmp/probe_result.json", 'w') as f:
            json.dump({"success": False, "error": "脚本异常: " + str(e)}, f)
        sys.exit(1)
