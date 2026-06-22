# Servo RTU KS100

KS100 伺服驱动器的 Chataigne Serial Modbus RTU 工具模块。

## 命令

| 命令 | 说明 |
|------|------|
| **Send Raw** | 输入十六进制字节（如 `01 03 00 47 00 03`），自动追加 CRC16 后发送 |
| **Probe Communication** | 自动探索伺服通信参数：遍历波特率 4800~115200，读取站号和串口模式 |

## Values（输出）

| Value | 说明 |
|-------|------|
| **Last Response** | 通信收发日志（>>> 发送，<<< 接收），保留最近 50 条 |
| **Communication Information > Baud Rate** | 检测到的伺服波特率 |
| **Communication Information > Protocol** | 检测到的串口模式（8N1/8N2/8E1/8O1） |

## 使用说明

1. 创建 Serial 模块，选择正确的串口，设置波特率（可先设为 115200 8N1）
2. 打开串口连接
3. 点击 **Probe Communication**，模块会自动遍历波特率并探测伺服
4. 探索成功后，检测到的参数会自动写入 Communication Information

## 通信参数

| 寄存器 | 地址 | 说明 |
|--------|------|------|
| FA-71 | `0x0047` | Modbus 从站地址（1~254） |
| FA-72 | `0x0048` | 波特率/100（48=4800, 96=9600, ...） |
| FA-73 | `0x0049` | 串口模式（0=8N2, 1=8E1, 2=8O1, 3=8N1） |
