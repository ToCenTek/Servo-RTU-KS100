var responseValue = null;
var baudRateValue = null;
var protocolValue = null;

var waiting = false;
var waitTime = 0;
var rxBuffer = [];

var probing = false;
var probeSlave = 1;
var probePollCount = 0;
var probeTotalPolls = 0;
var probeMaxPolls = 120;
var probeRestoreBaud = 0;
var probeRestoreMode = -1;

var opActive = false;
var opType = "";
var opStage = 0;
var opSlave = 1;
var opBaudVal = 0;
var opModeVal = 0;

var lastSentHex = "";
var responseAccumulator = "";

var RESULT_PATH = "/tmp/probe_result.json";
var SHELL_PATH = "/tmp/probe_ks100.sh";
var OS_NAME = "Servo OS";

var REG_FA71 = 0x0047;
var REG_FA72 = 0x0048;
var REG_FA73 = 0x0049;

// FA-73 寄存器值转协议模式标签
function modeLabel(mode) {
    if (mode == 0) return "8N2";
    if (mode == 1) return "8E1";
    if (mode == 2) return "8O1";
    if (mode == 3) return "8N1";
    return "8N1";
}

// 协议模式标签转 FA-73 值
function modeValue(label) {
    if (label == "8N2") return 0;
    if (label == "8E1") return 1;
    if (label == "8O1") return 2;
    return 3;
}

var crcTable = null;

// 预计算 CRC16 查表（多项式 0xA001）
function initCRCTable() {
    crcTable = [];
    var i = 0;
    var j = 0;
    for (i = 0; i < 256; i++) {
        var crc = i;
        for (j = 0; j < 8; j++) {
            if (crc % 2 == 0) {
                crc = Math.floor(crc / 2);
            } else {
                crc = Math.floor(crc / 2);
                crc = xor16(crc, 0xA001);
            }
        }
        crcTable.push(crc);
    }
}

// 16 位 XOR 算术模拟（JUCE 不支持 ^ 运算符）
function xor16(a, b) {
    var out = 0;
    var bit = 1;
    var i = 0;
    for (i = 0; i < 16; i++) {
        if ((a % 2) != (b % 2)) out = out + bit;
        a = Math.floor(a / 2);
        b = Math.floor(b / 2);
        bit = bit * 2;
    }
    return out;
}

// CRC16-Modbus，返回 [低字节, 高字节]
function crc16(bytes) {
    if (crcTable == null) initCRCTable();
    var crc = 0xFFFF;
    var i = 0;
    for (i = 0; i < bytes.length; i++) {
        var index = xor16(crc % 256, bytes[i]);
        crc = xor16(Math.floor(crc / 256), crcTable[index]);
    }
    return [crc % 256, Math.floor(crc / 256)];
}

// 追加 CRC16 低字节在前到字节数组
function appendCRC(bytes) {
    var out = [];
    var i = 0;
    for (i = 0; i < bytes.length; i++) out.push(bytes[i]);
    var c = crc16(out);
    out.push(c[0]);
    out.push(c[1]);
    return out;
}

// 校验完整 Modbus 帧的 CRC16
function validCRC(frame) {
    if (frame.length < 4) return false;
    var body = [];
    var i = 0;
    for (i = 0; i < frame.length - 2; i++) body.push(frame[i]);
    var c = crc16(body);
    if (frame[frame.length - 2] != c[0]) return false;
    if (frame[frame.length - 1] != c[1]) return false;
    return true;
}

// 十六进制字符转半字节值（0-15），无效返回 -1
function nibble(c) {
    var code = c.charCodeAt(0);
    if (code >= 48 && code <= 57) return code - 48;
    if (code >= 65 && code <= 70) return code - 55;
    if (code >= 97 && code <= 102) return code - 87;
    return -1;
}

// 十六进制字符串转字节数组，忽略非十六进制字符
function hexToBytes(text) {
    var clean = "";
    var i = 0;
    var c = "";
    for (i = 0; i < text.length; i++) {
        c = text.charAt(i);
        if (nibble(c) >= 0) clean = clean + c;
    }
    if ((clean.length % 2) != 0) return [];
    var bytes = [];
    for (i = 0; i < clean.length; i = i + 2) {
        bytes.push(nibble(clean.charAt(i)) * 16 + nibble(clean.charAt(i + 1)));
    }
    return bytes;
}

// 字节转 2 字符十六进制字符串
function toHexByte(v) {
    var s = "0123456789ABCDEF";
    return s.charAt(Math.floor(v / 16) % 16) + s.charAt(v % 16);
}

// 字节数组转空格分隔的十六进制字符串
function bytesToHex(bytes) {
    var out = "";
    var i = 0;
    for (i = 0; i < bytes.length; i++) {
        if (i > 0) out = out + " ";
        out = out + toHexByte(bytes[i]);
    }
    return out;
}

// 取高字节、低字节
function hi(v) { return Math.floor(v / 256) % 256; }
function lo(v) { return v % 256; }

// 构建 Modbus 读请求帧（不含 CRC）
function makeRead(slave, reg, count) {
    return [slave, 0x03, hi(reg), lo(reg), hi(count), lo(count)];
}

// 构建 Modbus 写单个寄存器帧（不含 CRC）
function makeWrite(slave, reg, value) {
    return [slave, 0x06, hi(reg), lo(reg), hi(value), lo(value)];
}

// 追加 CRC → 发送 → 记录 TX 日志
// 不清空 responseAccumulator,以便多命令串联(Set Communication 内部 2 帧)时
// 所有 RX 响应按时间顺序累积到 Last Response,便于回溯完整执行过程。
function sendFrame(bytes) {
    var frame = appendCRC(bytes);
    lastSentHex = bytesToHex(frame);
    rxBuffer = [];
    waiting = true;
    waitTime = 0;
    local.sendBytes(frame);
    script.log("-TX: " + lastSentHex);
}

// 按多个名称变体搜索模块中的可写参数
function findParam(names) {
    var i = 0;
    var p = null;
    for (i = 0; i < names.length; i++) {
        p = local.getChild(names[i]);
        if (p != null) return p;
    }
    var conn = local.getChild("serial");
    if (conn == null) conn = local.getChild("Serial");
    if (conn == null) conn = local.getChild("connection");
    if (conn == null) conn = local.getChild("Connection");
    if (conn != null) {
        for (i = 0; i < names.length; i++) {
            p = conn.getChild(names[i]);
            if (p != null) return p;
        }
    }
    return null;
}

// 将 FA-73 协议值映射为 (dataBits, parity, stopBits) 三元组
function modeToSerial(mode) {
    if (mode == 0) return [8, "None", 2];
    if (mode == 1) return [8, "Even", 1];
    if (mode == 2) return [8, "Odd", 1];
    return [8, "None", 1];
}

// 尝试同步 Chataigne 串口参数到探测到的伺服状态
// 返回 true 表示至少波特率被设置
function setSerialConfig(baud, mode) {
    var ok = false;
    var baudP = findParam(["baudrate", "BaudRate", "baud_rate"]);
    if (baudP != null) {
        baudP.set(baud);
        ok = true;
    }
    var parts = modeToSerial(mode);
    var dataP = findParam(["databits", "DataBits", "data_bits"]);
    if (dataP != null) dataP.set(parts[0]);
    var parityP = findParam(["parity", "Parity"]);
    if (parityP != null) parityP.set(parts[1]);
    var stopP = findParam(["stopbits", "StopBits", "stop_bits"]);
    if (stopP != null) stopP.set(parts[2]);
    return ok;
}

// 兼容旧调用：仅设置波特率
function setSerialBaudRate(baud) {
    var p = findParam(["baudrate", "BaudRate", "baud_rate"]);
    if (p == null) return false;
    p.set(baud);
    return true;
}

// 删除我们自己创建的 Servo OS 模块
function removeOSModule() {
    var osMod = root.modules.getItemWithName(OS_NAME);
    if (osMod != null) {
        root.modules.removeItem(osMod);
        script.refreshEnvironment();
    }
}

// 确保 OS 模块存在，不存在则自动创建
function ensureOSModule() {
    var osMod = root.modules.getItemWithName(OS_NAME);
    if (osMod == null) {
        script.log(OS_NAME + " not found, auto-creating...");
        osMod = root.modules.addItem("OS");
        if (osMod == null) {
            script.log("Failed to create OS module, please add one named " + OS_NAME);
            return null;
        }
        osMod.setName(OS_NAME);
        script.refreshEnvironment();
        script.log(OS_NAME + " auto-created");
    }
    return osMod;
}

// 尝试启用/禁用本模块
function trySetModuleEnabled(enabled) {
    var names = ["enabled", "Enabled", "enable", "Enable"];
    var i = 0;
    for (i = 0; i < names.length; i++) {
        var p = local.getChild(names[i]);
        if (p != null && typeof p.set == "function") {
            p.set(enabled);
            return true;
        }
    }
    return false;
}

// 写入 shell 包装脚本到 /tmp（解决路径空格问题）
function writeShellWrapper() {
    var scriptDir = script.getScriptDirectory();
    var pyPath = scriptDir + "/scripts/probe_servo.py";
    var logPath = "/tmp/probe_ks100.log";
    var content = "#!/bin/bash\n";
    content = content + "export PATH=\"/opt/homebrew/bin:/usr/local/bin:$PATH\"\n";
    content = content + "python3 \"" + pyPath + "\" \"$@\" 2>\"" + logPath + "\"\n";
    util.writeFile(SHELL_PATH, content, true);
}

// 更新探测结果到 Communication Information
function updateDetectedValues(baud, slave, mode) {
    probeRestoreBaud = baud;
    probeRestoreMode = mode;
    if (baudRateValue != null) {
        baudRateValue.set("" + baud);
    }
    if (protocolValue != null) {
        protocolValue.set(modeLabel(mode));
    }
    var ci = local.values.getChild("Communication Information");
    if (ci != null) {
        ci.setCollapsed(false);
    }
    script.log("Servo found: slave " + slave + ", baud " + baud + ", mode " + modeLabel(mode));
}

// 模块初始化：获取值对象引用
function init() {
    if (local.values != null) {
        var ci = local.values.getChild("Communication Information");
        if (ci != null) {
            responseValue = ci.getChild("Last Response");
            baudRateValue = ci.getChild("Baud Rate");
            protocolValue = ci.getChild("Protocol");
        }
    }
    script.setUpdateRate(20);
    script.log("Servo RTU KS100 loaded");
}

// 周期调用：处理超时和探测结果轮询
function update(deltaTime) {
    if (waiting) {
        waitTime = waitTime + deltaTime;
        if (waitTime > 0.5) {
            var timedOutOp = opType;
            var timedOutStage = opStage;
            waiting = false;
            if (opActive) {
                opActive = false;
                opType = "";
                opStage = 0;
                if (timedOutOp == "set_comm") {
                    if (timedOutStage == 1) {
                        script.log("Set communication failed at stage 1 (mode " + modeLabel(opModeVal) + ")");
                        script.log("Servo did not respond. Current module serial may not match servo.");
                    } else if (timedOutStage == 2) {
                        script.log("Set communication failed at stage 2 (baud " + opBaudVal + ")");
                        script.log("Mode was updated to " + modeLabel(opModeVal) + " but baud change failed.");
                        script.log("Servo serial is now: baud=" + opBaudVal + " mode=" + modeLabel(opModeVal));
                    }
                } else if (timedOutOp == "set_slave") {
                    script.log("Set slave address failed");
                } else {
                    script.log("Operation timeout");
                }
                script.log("TX: " + lastSentHex);
                script.log("Run Probe Communication to recover, or match module serial manually.");
            } else if (!probing) {
                script.log("Command timeout, no response");
                script.log("TX: " + lastSentHex);
            }
        }
    }

    if (probing) {
        probePollCount = probePollCount + 1;
        if (probePollCount >= 10) {
            probePollCount = 0;
            probeTotalPolls = probeTotalPolls + 1;
            if (util.fileExists(RESULT_PATH)) {
                var data = util.readFile(RESULT_PATH, true);
                if (data != null && data.status != "probing") {
                    probing = false;
                    removeOSModule();
                    if (data.success) {
                        updateDetectedValues(data.baud, data.slave, data.fa73);
                        setSerialConfig(data.baud, data.fa73);
                        script.log("Probe complete, parameters updated");
                    } else {
                        var errMsg = "Probe failed";
                        if (data.error) errMsg = data.error;
                        script.log(errMsg);
                        if (data.detail) script.log(data.detail);
                    }
                    trySetModuleEnabled(true);
                }
            } else if (probeTotalPolls >= probeMaxPolls) {
                probing = false;
                removeOSModule();
                trySetModuleEnabled(true);
                script.log("Probe timeout (60s), check serial and Python");
            }
        }
    }
}

// 串口收到数据：缓冲 → 提取完整 Modbus 帧 → 更新 LastResponse
function dataReceived(data) {
    var i = 0;
    for (i = 0; i < data.length; i++) rxBuffer.push(data[i]);

    while (true) {
        var frame = tryExtractFrame();
        if (frame == null) break;

        var hex = bytesToHex(frame);
        script.log("-RX: " + hex);

        if (responseValue != null) {
            if (responseAccumulator.length > 0) {
                responseAccumulator = responseAccumulator + "\n" + hex;
            } else {
                responseAccumulator = hex;
            }
            responseValue.set(responseAccumulator);
        }

        waiting = false;

        if (opActive) {
            if (opType == "set_comm") {
                continueSetComm(frame);
            } else if (opType == "set_slave") {
                continueSetSlave(frame);
            }
        }
    }
}

// 从缓冲中提取一帧完整 Modbus 响应
function tryExtractFrame() {
    if (rxBuffer.length < 5) return null;

    var func = rxBuffer[1];
    var frameLen = 0;

    if ((func & 0x80) != 0) {
        frameLen = 5;
    } else if (func == 0x03 || func == 0x04) {
        var byteCount = rxBuffer[2];
        frameLen = 3 + byteCount + 2;
    } else if (func == 0x06 || func == 0x05) {
        frameLen = 8;
    } else if (func == 0x0F || func == 0x10) {
        frameLen = 8;
    } else {
        return null;
    }

    if (rxBuffer.length < frameLen) return null;

    var frame = [];
    var remaining = [];
    var j = 0;
    for (j = 0; j < rxBuffer.length; j++) {
        if (j < frameLen) frame.push(rxBuffer[j]);
        else remaining.push(rxBuffer[j]);
    }
    rxBuffer = remaining;
    return frame;
}

// 命令：获取伺服通信参数（外部 Python 脚本）
function getCommunication(slaveToProbe) {
    if (waiting || probing) return;
    if (slaveToProbe < 1 || slaveToProbe > 254) {
        slaveToProbe = 1;
    }
    probeSlave = slaveToProbe;
    probing = false;

    util.showMessageBox("Please wait...", "Detecting servo communication parameters...\nRe-enable module then apply new settings.", "info", "OK");

    if (!trySetModuleEnabled(false)) {
        script.log("Cannot close port, disable module or close it manually then probe");
    }

    var osMod = ensureOSModule();
    if (osMod == null) {
        script.log("Probe aborted: OS module required");
        trySetModuleEnabled(true);
        removeOSModule();
        return;
    }

    writeShellWrapper();

    var cmd = "/bin/bash " + SHELL_PATH + " --slave " + probeSlave + " --output " + RESULT_PATH;

    script.log("Probing servo parameters (slave " + probeSlave + ")");

    util.writeFile(RESULT_PATH, '{"success":false,"status":"probing"}', true);

    if (typeof osMod.launchProcess == "function") {
        osMod.launchProcess(cmd);
    } else {
        script.log("OS module has no launchProcess, run manually:");
        script.log(cmd);
        trySetModuleEnabled(true);
        removeOSModule();
        return;
    }

    probing = true;
    probePollCount = 0;
    probeTotalPolls = 0;
}

// 多步操作：写 FA-73 协议模式、写 FA-72 波特率
function continueSetComm(frame) {
    if (opStage == 1 && frame[1] == 0x06) {
        opStage = 2;
        var baudDiv = Math.floor(opBaudVal / 100);
        sendFrame(makeWrite(opSlave, REG_FA72, baudDiv));
        return;
    }
    if (opStage == 2 && frame[1] == 0x06) {
        opActive = false;
        opType = "";
        opStage = 0;
        script.log("Servo communication updated to " + opBaudVal + " " + modeLabel(opModeVal));
        return;
    }
    opActive = false;
    opType = "";
    opStage = 0;
    script.log("Set communication failed at stage " + opStage);
}

// 多步操作：写 FA-71 从站地址
function continueSetSlave(frame) {
    if (frame[1] == 0x06) {
        opActive = false;
        opType = "";
        opStage = 0;
        script.log("Slave address updated to " + opSlave);
        return;
    }
    opActive = false;
    opType = "";
    opStage = 0;
    script.log("Set slave failed");
}

// 命令：修改伺服通信参数（波特率 + 协议模式）
function setCommunication(slave, baud, mode) {
    if (waiting || probing || opActive) return;
    if (slave < 1 || slave > 254) return;
    var baudNum = parseInt(baud, 10);
    if (baudNum < 4800 || baudNum > 115200) return;
    var modeNum = modeValue(mode);

    opActive = true;
    opType = "set_comm";
    opStage = 1;
    opSlave = slave;
    opBaudVal = baudNum;
    opModeVal = modeNum;

    script.log("Setting communication: slave=" + slave + " baud=" + baudNum + " mode=" + mode);
    sendFrame(makeWrite(slave, REG_FA73, modeNum));
}

// 命令：设置从站地址
function setSlaveAddress(currentSlave, newSlave) {
    if (waiting || probing || opActive) return;
    if (currentSlave < 1 || currentSlave > 254) return;
    if (newSlave < 1 || newSlave > 254) return;

    opActive = true;
    opType = "set_slave";
    opStage = 1;
    opSlave = newSlave;

    script.log("Setting slave address from " + currentSlave + " to " + newSlave);
    sendFrame(makeWrite(currentSlave, REG_FA71, newSlave));
}

// Parameters 面板参数变化回调
function moduleParameterChanged(param) {
    if (param.niceName == "Get Communication") {
        var slaveAddr = null;
        if (local.parameters != null) {
            slaveAddr = local.parameters.getChild("Slave Address");
        }
        var slave = 1;
        if (slaveAddr != null) slave = slaveAddr.get();
        getCommunication(slave);
    }
}

// 模块被移除时清理
function moduleCleanedUp() {
    removeOSModule();
}

// 命令：发送原始十六进制数据（自动追加 CRC16）
function sendRaw(hexCommand) {
    if (waiting || probing || opActive) return;
    var body = hexToBytes(hexCommand);
    if (body.length < 2) return;
    responseAccumulator = "";
    sendFrame(body);
}
