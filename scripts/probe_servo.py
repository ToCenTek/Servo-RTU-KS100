#!/usr/bin/env python3
"""KS100 伺服 Modbus RTU 通信参数自动探测脚本.

工作原理 (How it works):
  通过串口依次尝试所有 (波特率, 数据位/校验/停止位) 组合, 在每个候选从站地址
  (1 ~ scan-end) 上发送 Modbus 0x03 (读保持寄存器) 命令读取 0x0047~0x0049
  (FA-71/FA-72/FA-73), 这三个寄存器分别保存: 站号 / 波特率 / 协议模式.
  如果伺服以匹配的串口参数和站号存在, 就会返回有效响应. 找到第一个有效
  响应即停止 (因为假设总线上最多 1 个设备), 写入 /tmp/probe_result.json.

调用: --slave N --scan-end M --port /dev/tty.usbserial-XXXX --output /tmp/probe_result.json
  --slave N      扫描起始从站地址 (默认 1)
  --scan-end M   扫描结束从站地址 (默认 10, 含)
  --port P       指定串口设备 (默认自动检测)
  --output O     结果 JSON 路径 (默认 /tmp/probe_result.json)

使用前提:
  1. 总线上同时只能有 1 个从站(其他从站断电), 否则多从站同时响应会导致
     Modbus 总线冲突, CRC 错误
  2. 调用方负责保证总线上只有一个从站
  3. 扫描范围 (slave ~ scan-end) 必须包含该设备的实际站号

返回结果 (写入 --output):
  {
    "success": True/False,
    "baud": <int>,                  # 探测到的波特率
    "slave": <int>,                 # 探测到的从站 (从 0x0047 读出, 反映伺服实际地址)
    "protocol": "8N1"|"8N2"|...,    # 协议模式标签
    "fa72": <int>,                  # FA-72 寄存器值 (= baud / 100)
    "fa73": <int>,                  # FA-73 寄存器值 (0=8N2 1=8E1 2=8O1 3=8N1)
    "port": "/dev/...",             # 找到伺服的串口
    "scan_range": [start, end],     # 扫描范围
  }
"""
import sys
import json
import struct
import os

# pyserial 是 Python 串口库, 必须安装. 缺失时给清晰错误而不是 ImportError.
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    INSTALL_CMD = "python3 -m pip install pyserial --break-system-packages"
    msg = "Python pyserial 库未安装。\n请运行: " + INSTALL_CMD
    result = {"success": False, "error": msg}
    output = "/tmp/probe_result.json"
    for i, a in enumerate(sys.argv):
        if a == '--output' and i + 1 < len(sys.argv):
            output = sys.argv[i + 1]
    with open(output, 'w') as f:
        json.dump(result, f)
    print(msg, file=sys.stderr)
    sys.exit(1)

# KS100 支持的波特率 (用户手册 FA-72, 值 = baud / 100)
BAUD_RATES = [115200, 57600, 38400, 19200, 9600, 4800]

# FA-73 寄存器值 -> 协议模式字符串 (用于显示)
MODE_MAP = {0: "8N2", 1: "8E1", 2: "8O1", 3: "8N1"}

# 协议模式字符串 -> (dataBits, parity, stopBits) 实际串口参数
# (KS100 手册 FA-73: 0=8N2 1=8E1 2=8O1 3=8N1)
MODES = {
    "8N1": (8, 'N', 1),
    "8N2": (8, 'N', 2),
    "8E1": (8, 'E', 1),
    "8O1": (8, 'O', 1),
}

# 单次发送/接收的超时秒数. KS100 响应应该立即返回, 80ms 足够.
TIMEOUT = 0.08


def crc16(data):
    """Modbus RTU CRC16 (多项式 0xA001, 初值 0xFFFF).

    注意: Modbus 协议要求 CRC 低字节先发送, 高字节后发送. 这里
    返回小端 (struct.pack('<H', crc)) 即符合该要求.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                # LSB=1: 右移后异或多项式 0xA001
                crc = ((crc >> 1) ^ 0xA001) & 0xFFFF
            else:
                # LSB=0: 仅右移
                crc = (crc >> 1) & 0xFFFF
    return struct.pack('<H', crc)


def make_read(slave, reg16, count=3):
    """构造 Modbus 0x03 读多字命令 (直接追加 CRC).

    参数:
      slave: 从站地址 (1~254)
      reg16: 起始寄存器地址 (16bit)
      count: 读几个字 (默认 3, 即读 0x0047~0x0049)

    返回: 完整 8 字节的 Modbus RTU 帧 (含 CRC)
    """
    # struct.pack('>BBHH', slave, func, reg, count)
    # > = 大端, B=1byte, H=2byte
    # func=0x03 (Read Holding Registers)
    cmd = struct.pack('>BBHH', slave, 0x03, reg16, count)
    return cmd + crc16(cmd)


def try_mode(port, baud, mode_key, slave):
    """在指定 (port, baud, mode, slave) 组合下尝试读 0x0047~0x0049.

    返回:
      None: 无响应 / CRC 错误 / 异常
      dict: 成功响应, 包含 fa72, fa73, baud, slave, protocol 等字段
    """
    db, par, sb = MODES[mode_key]
    try:
        ser = serial.Serial(port, baud, bytesize=db, parity=par,
                            stopbits=sb, timeout=TIMEOUT)
        cmd = make_read(slave, 0x0047, 3)  # 读 3 个字: FA-71/FA-72/FA-73
        ser.write(cmd)
        resp = ser.read(256)
        ser.close()

        # 响应帧: ADR(1) FUNC(1) BYTECOUNT(1) DATA(2N) CRC(2) = 5 + 2N
        # 我们读 3 个字, N=3, 所以 byteCount=6, 帧总长 5+6=11
        # 检查最低要求: 至少 11 字节, 且 FUNC=0x03
        if len(resp) >= 11 and resp[1] == 0x03:
            # FA-72 = resp[5]*256 + resp[6] (高字节先)
            fa72 = resp[5] * 256 + resp[6]
            # FA-73 = resp[7]*256 + resp[8]
            fa73 = resp[7] * 256 + resp[8]
            # FA-73 合法值 0~3, 越界视为无效
            if fa73 < 0 or fa73 > 3:
                fa73 = -1
            proto = MODE_MAP.get(fa73, "8N1")
            return {"success": True, "baud": baud, "slave": slave,
                    "protocol": proto, "fa72": fa72, "fa73": fa73, "port": port}
    except Exception:
        # 串口打开失败 / 写入失败 / 异常等, 静默返回 None
        pass
    return None


def write_result(result, path):
    """把探测结果写入 JSON 文件, 供 Chataigne JS 端读取."""
    with open(path, 'w') as f:
        json.dump(result, f)


def main():
    # 默认参数
    slave_start = 1
    slave_end = 10          # 默认扫 1~10, 覆盖大部分场景, 速度可接受
    port = None             # None = 自动检测
    output = "/tmp/probe_result.json"

    # 解析命令行参数
    i = 0
    while i < len(sys.argv):
        a = sys.argv[i]
        if a == '--slave' and i + 1 < len(sys.argv):
            slave_start = int(sys.argv[i + 1])
            i += 2
        elif a == '--scan-end' and i + 1 < len(sys.argv):
            slave_end = int(sys.argv[i + 1])
            if slave_end < slave_start:
                slave_end = slave_start
            i += 2
        elif a == '--port' and i + 1 < len(sys.argv):
            port = sys.argv[i + 1]; i += 2
        elif a == '--output' and i + 1 < len(sys.argv):
            output = sys.argv[i + 1]; i += 2
        else:
            i += 1

    # 列举可用串口
    try:
        ports = [port] if port else [p.device for p in serial.tools.list_ports.comports()]
    except Exception:
        ports = []

    if not ports:
        write_result({"success": False, "error": "未检测到串口设备"}, output)
        sys.exit(1)

    # 扫描循环:
    #   外层: 从站地址 slave_start ~ slave_end (假设总线上最多 1 个)
    #     中层: 波特率 115200 -> 4800 (高频优先)
    #       内层: 协议模式 (8N1/8N2 优先, 失败再 8E1/8O1)
    #   任意组合成功就 sys.exit(0), 找到第一个即停止.
    #
    # 多从站注意事项: 多个 KS100 同时上电, 对同一请求都会响应, 电平叠加
    # 会导致模块收到 CRC 错误的乱码. 探测前必须保证总线上只有 1 个从站.
    # 实际站号通过 FA-71 寄存器读出 (在响应帧中), 不需要预先知道.
    for slave in range(slave_start, slave_end + 1):
        for baud in BAUD_RATES:
            # Fast pass: 8N1 和 8N2 是出厂默认和最常见模式, 优先尝试
            for mode_key in ["8N1", "8N2"]:
                for p in ports:
                    r = try_mode(p, baud, mode_key, slave)
                    if r:
                        r["scan_range"] = [slave_start, slave_end]
                        write_result(r, output)
                        print(json.dumps(r))
                        sys.exit(0)

            # Fallback: 8E1 和 8O1 是带校验位的少数情况
            for mode_key in ["8E1", "8O1"]:
                for p in ports:
                    r = try_mode(p, baud, mode_key, slave)
                    if r:
                        r["scan_range"] = [slave_start, slave_end]
                        write_result(r, output)
                        print(json.dumps(r))
                        sys.exit(0)

    # 全部组合都试过没找到
    write_result({
        "success": False,
        "error": "未找到伺服",
        "detail": "扫描 slave {}~{}, 所有波特率/协议组合均无响应".format(slave_start, slave_end),
        "scan_range": [slave_start, slave_end]
    }, output)
    sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        result = {"success": False, "error": "脚本异常: " + str(e)}
        with open("/tmp/probe_result.json", 'w') as f:
            json.dump(result, f)
        sys.exit(1)
