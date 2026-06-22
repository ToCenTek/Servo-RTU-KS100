#!/usr/bin/env python3
"""Probe KS100 servo: scan baud rates and protocol mode, write result as JSON."""
import sys
import json
import struct
import os

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    INSTALL_CMD = "python3 -m pip install pyserial --break-system-packages"
    msg = "Python pyserial \u5e93\u672a\u5b89\u88c5\u3002\n\u8bf7\u8fd0\u884c: " + INSTALL_CMD
    result = {"success": False, "error": msg}
    output = "/tmp/probe_result.json"
    for i, a in enumerate(sys.argv):
        if a == '--output' and i + 1 < len(sys.argv):
            output = sys.argv[i + 1]
    with open(output, 'w') as f:
        json.dump(result, f)
    print(msg, file=sys.stderr)
    sys.exit(1)

BAUD_RATES = [115200, 57600, 38400, 19200, 9600, 4800]
MODE_MAP = {0: "8N2", 1: "8E1", 2: "8O1", 3: "8N1"}
TIMEOUT = 0.08

MODES = {
    "8N1": (8, 'N', 1),
    "8N2": (8, 'N', 2),
    "8E1": (8, 'E', 1),
    "8O1": (8, 'O', 1),
}

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

def make_read(slave, reg16):
    cmd = struct.pack('>BBHH', slave, 0x03, reg16, 3)
    return cmd + crc16(cmd)

def try_mode(port, baud, mode_key, slave):
    db, par, sb = MODES[mode_key]
    try:
        ser = serial.Serial(port, baud, bytesize=db, parity=par,
                            stopbits=sb, timeout=TIMEOUT)
        cmd = make_read(slave, 0x0047)
        ser.write(cmd)
        resp = ser.read(256)
        ser.close()
        if len(resp) >= 11 and resp[1] == 0x03:
            fa72 = resp[5] * 256 + resp[6]
            fa73 = resp[7] * 256 + resp[8]
            if fa73 < 0 or fa73 > 3:
                fa73 = -1
            proto = MODE_MAP.get(fa73, "8N1")
            return {"success": True, "baud": baud, "slave": slave,
                    "protocol": proto, "fa72": fa72, "fa73": fa73, "port": port}
    except Exception:
        pass
    return None

def write_result(result, path):
    with open(path, 'w') as f:
        json.dump(result, f)

def main():
    slave = 1
    port = None
    output = "/tmp/probe_result.json"

    i = 0
    while i < len(sys.argv):
        a = sys.argv[i]
        if a == '--slave' and i + 1 < len(sys.argv):
            slave = int(sys.argv[i + 1]); i += 2
        elif a == '--port' and i + 1 < len(sys.argv):
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
        write_result({"success": False, "error": "未检测到串口设备"}, output)
        sys.exit(1)

    # Fast pass: try common modes (8N1, 8N2) on all baud rates
    for baud in BAUD_RATES:
        for mode_key in ["8N1", "8N2"]:
            for p in ports:
                r = try_mode(p, baud, mode_key, slave)
                if r:
                    write_result(r, output)
                    print(json.dumps(r))
                    sys.exit(0)

    # Fallback: try parity modes (8E1, 8O1)
    for baud in BAUD_RATES:
        for mode_key in ["8E1", "8O1"]:
            for p in ports:
                r = try_mode(p, baud, mode_key, slave)
                if r:
                    write_result(r, output)
                    print(json.dumps(r))
                    sys.exit(0)

    write_result({"success": False, "error": "遍历所有波特率和协议模式后未找到伺服"}, output)
    sys.exit(1)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        result = {"success": False, "error": "脚本异常: " + str(e)}
        with open("/tmp/probe_result.json", 'w') as f:
            json.dump(result, f)
        sys.exit(1)
