// Servo RTU KS100 - 简化版
// 协议固定 8N1 (Chataigne Serial Module 硬编码 8N1, module.json defaults 也写 8N1)
// 探测: 扫描 baud × slave, 找到伺服后自动改 FA-73=3 强制 8N1
// 操作: Set BaudRate (写 FA-72 + 软复位) / Set Slave Address (写 FA-71)

// 状态
var opActive = false;
var opType = "";            // "set_baud" | "set_slave" | "force_8n1"
var opStage = 0;
var opSlave = 1;
var opBaudVal = 0;

var probing = false;
var probePolls = 0;
const PROBE_MAX_POLLS = 600;    // ~120s

var waiting = false;
var waitTime = 0;
var rxBuffer = [];

var resetPending = false;
var resetWaitTime = 0;
const RESET_WAIT_SEC = 2.5;

var responseValue = null;
var baudRateValue = null;
var protocolValue = null;
var slaveAddressValue = null;
var responseAccumulator = "";
var lastSentHex = "";

// 寄存器
const REG_FA60 = 0x003C;  // 软复位 (写 1 触发)
const REG_FA71 = 0x0047;  // 从站地址
const REG_FA72 = 0x0048;  // 波特率 (值 = baud/100)
const REG_FA73 = 0x0049;  // 协议 (3 = 8N1)

const RESULT_PATH = "/tmp/probe_result.json";
const SHELL_PATH = "/tmp/probe_ks100.sh";
const OS_NAME = "Servo OS";

// CRC16-Modbus (查表法, 多项式 0xA001, 算术实现避开位运算符)
var crcTable = null;
function xor16(a, b) {
    var out = 0;
    var bit = 1;
    for (var i = 0; i < 16; i++) {
        if ((a % 2) != (b % 2)) out += bit;
        a = Math.floor(a / 2);
        b = Math.floor(b / 2);
        bit *= 2;
    }
    return out;
}
function initCRCTable() {
    crcTable = [];
    for (var i = 0; i < 256; i++) {
        var crc = i;
        for (var j = 0; j < 8; j++) {
            if (crc % 2 == 0) crc = Math.floor(crc / 2);
            else crc = xor16(Math.floor(crc / 2), 0xA001);
        }
        crcTable.push(crc);
    }
}
function crc16(bytes) {
    if (crcTable == null) initCRCTable();
    var crc = 0xFFFF;
    for (var i = 0; i < bytes.length; i++) {
        var index = xor16(crc % 256, bytes[i]);
        crc = xor16(Math.floor(crc / 256), crcTable[index]);
    }
    return [crc % 256, Math.floor(crc / 256)];
}

function nibble(c) {
    var code = c.charCodeAt(0);
    if (code >= 48 && code <= 57) return code - 48;
    if (code >= 65 && code <= 70) return code - 55;
    if (code >= 97 && code <= 102) return code - 87;
    return -1;
}
function hexToBytes(text) {
    var clean = "";
    for (var i = 0; i < text.length; i++) {
        var c = text.charAt(i);
        if (nibble(c) >= 0) clean = clean + c;
    }
    if ((clean.length % 2) != 0) return [];
    var bytes = [];
    for (var i = 0; i < clean.length; i += 2) {
        bytes.push(nibble(clean.charAt(i)) * 16 + nibble(clean.charAt(i + 1)));
    }
    return bytes;
}
function toHexByte(v) {
    var s = "0123456789ABCDEF";
    return s.charAt(Math.floor(v / 16) % 16) + s.charAt(v % 16);
}
function bytesToStr(bytes) {
    var out = "";
    for (var i = 0; i < bytes.length; i++) {
        if (i > 0) out += " ";
        out += toHexByte(bytes[i]);
    }
    return out;
}

function hi(v) { return Math.floor(v / 256) % 256; }
function lo(v) { return v % 256; }

function makeRead(slave, reg, count) {
    return [slave, 0x03, hi(reg), lo(reg), hi(count), lo(count)];
}
function makeWrite(slave, reg, value) {
    return [slave, 0x06, hi(reg), lo(reg), hi(value), lo(value)];
}
function sendFrame(pdu) {
    var c = crc16(pdu);
    pdu.push(c[0], c[1]);
    lastSentHex = bytesToStr(pdu);
    script.log("-TX: " + lastSentHex);
    rxBuffer = [];
    waiting = true;
    waitTime = 0;
    local.sendBytes(pdu);
}

function findParam(name) {
    if (local.parameters != null) {
        var p = local.parameters.getChild(name);
        if (p != null) return p;
    }
    return local.getChild(name);
}

// 更新 Communication Information 显示
function updateValues(slave, baud, protocol) {
    if (slaveAddressValue != null) slaveAddressValue.set("" + slave);
    if (baudRateValue != null) baudRateValue.set("" + baud);
    if (protocolValue != null) protocolValue.set(protocol);
    var ci = local.values.getChild("Communication Information");
    if (ci != null) ci.setCollapsed(false);
}

// 同步 Chataigne Serial Module 波特率 (DataBits/Parity/StopBits 由 module.json defaults 决定, 运行时改不了)
function setSerialBaudRate(baud) {
    var p = findParam("baudRate");
    if (p == null) return false;
    p.set(baud);
    return true;
}

// 同步 Chataigne 从站地址参数
function setSerialSlaveAddress(slave) {
    var p = findParam("slaveAddress");
    if (p == null) return false;
    p.set(slave);
    return true;
}

// 启停本模块 (关闭/打开串口, 探测时需关闭避免与 Python pyserial 冲突)
function setModuleEnabled(enabled) {
    if (local.parameters != null) {
        var p = local.parameters.getChild("enabled");
        if (p != null && typeof p.set == "function") {
            p.set(enabled);
            return true;
        }
    }
    return false;
}

// OS Module 探测 (Python pyserial 扫描)
function ensureOSModule() {
    var osMod = root.modules.getItemWithName(OS_NAME);
    if (osMod != null) return osMod;
    script.log(OS_NAME + " not found, auto-creating...");
    osMod = root.modules.addItem("OS");
    if (osMod == null) {
        script.log("Failed to create OS module. Please add one named " + OS_NAME);
        return null;
    }
    osMod.setName(OS_NAME);
    script.refreshEnvironment();
    script.log(OS_NAME + " auto-created");
    return osMod;
}
function removeOSModule() {
    var osMod = root.modules.getItemWithName(OS_NAME);
    if (osMod != null) {
        root.modules.removeItem(osMod);
        script.refreshEnvironment();
    }
}
function writeShellWrapper() {
    var pyPath = script.getScriptDirectory() + "/scripts/probe_servo.py";
    var logPath = "/tmp/probe_ks100.log";
    var content = "#!/bin/bash\n" +
        "export PATH=\"/opt/homebrew/bin:/usr/local/bin:$PATH\"\n" +
        "python3 \"" + pyPath + "\" \"$@\" 2>\"" + logPath + "\"\n";
    util.writeFile(SHELL_PATH, content, true);
}

// 命令: Get Communication - 探测伺服 (固定从站 1)
function getCommunication() {
    if (waiting || probing || opActive || resetPending) return;
    script.log("Probing servo (slave 1~254, all baud)...");
    setModuleEnabled(false);
    var osMod = ensureOSModule();
    if (osMod == null) { setModuleEnabled(true); return; }
    writeShellWrapper();
    util.writeFile(RESULT_PATH, '{"success":false,"status":"probing"}', true);
    if (typeof osMod.launchProcess != "function") {
        script.log("OS module has no launchProcess. Run manually:");
        script.log("  /bin/bash " + SHELL_PATH + " --output " + RESULT_PATH);
        setModuleEnabled(true);
        removeOSModule();
        return;
    }
    script.log("Launching probe process...");
    osMod.launchProcess("/bin/bash " + SHELL_PATH + " --output " + RESULT_PATH, false);
    probing = true;
    probePolls = 0;
}

// 命令: Set BaudRate - 写 FA-72 + 软复位
function setBaudRate(slave, baud) {
    if (waiting || probing || opActive || resetPending) return;
    var slaveNum = parseInt(slave, 10);
    var baudNum = parseInt(baud, 10);
    if (slaveNum < 1 || slaveNum > 254) return;
    if (baudNum < 300 || baudNum > 115200) return;
    opActive = true;
    opType = "set_baud";
    opStage = 1;
    opSlave = slaveNum;
    opBaudVal = baudNum;
    script.log("Set baud rate: slave=" + slaveNum + " baud=" + baudNum);
    sendFrame(makeWrite(slaveNum, REG_FA72, Math.floor(baudNum / 100)));
}

function continueSetBaud(frame) {
    if (frame[1] != 0x06) {
        opActive = false;
        opType = "";
        script.log("Set baud rate failed: unexpected response 0x" + toHexByte(frame[1]));
        return;
    }
    if (opStage == 1) {
        opStage = 2;
        sendFrame(makeWrite(opSlave, REG_FA60, 1));
        return;
    }
    if (opStage == 2) {
        opActive = false;
        script.log("Reset sent. Waiting " + RESET_WAIT_SEC + "s...");
        resetPending = true;
        resetWaitTime = 0;
    }
}

// 命令: Set Slave Address - 写 FA-71
function setSlaveAddress(newSlave) {
    if (waiting || probing || opActive || resetPending) return;
    var newNum = parseInt(newSlave, 10);
    if (newNum < 1 || newNum > 254) return;
    var p = findParam("slaveAddress");
    var currentSlave = (p != null) ? p.get() : 1;
    if (currentSlave == newNum) {
        script.log("Slave address already " + newNum + ", nothing to do");
        return;
    }
    opActive = true;
    opType = "set_slave";
    opStage = 1;
    opSlave = newNum;
    script.log("Set slave address: " + currentSlave + " -> " + newNum);
    sendFrame(makeWrite(currentSlave, REG_FA71, newNum));
}

function continueSetSlave(frame) {
    if (frame[1] != 0x06) {
        opActive = false;
        opType = "";
        script.log("Set slave address failed: unexpected response");
        return;
    }
    opActive = false;
    opType = "";
    setSerialSlaveAddress(opSlave);
    script.log("Slave address updated to " + opSlave);
    util.showMessageBox("Slave Address Updated",
        "Slave address updated to " + opSlave + ".\n" +
        "Module Slave Address parameter synced.\n\n" +
        "从站地址已更新到 " + opSlave + "。\n" +
        "模块 Slave Address 参数已同步。",
        "info", "OK");
}

// 自动强制伺服为 8N1 (在 Get Communication 完成后, 如果 FA-73 != 3)
function startForce8N1(slave, baud, currentProtocol) {
    opActive = true;
    opType = "force_8n1";
    opStage = 1;
    opSlave = slave;
    opBaudVal = baud;
    script.log("Servo is " + currentProtocol + ", forcing 8N1...");
    sendFrame(makeWrite(slave, REG_FA73, 3));
}

function continueForce8N1(frame) {
    if (frame[1] != 0x06) {
        opActive = false;
        opType = "";
        script.log("Force 8N1 failed: unexpected response");
        return;
    }
    if (opStage == 1) {
        opStage = 2;
        sendFrame(makeWrite(opSlave, REG_FA60, 1));
        return;
    }
    if (opStage == 2) {
        opActive = false;
        script.log("Force 8N1: reset sent. Waiting...");
        resetPending = true;
        resetWaitTime = 0;
    }
}

function handleResetComplete() {
    var type = opType;
    opType = "";
    if (type == "set_baud") {
        setSerialBaudRate(opBaudVal);
        updateValues(opSlave, opBaudVal, "8N1");
        script.log("Baud rate set to " + opBaudVal);
        util.showMessageBox("Baud Rate Set",
            "Baud rate set to " + opBaudVal + ".\n" +
            "Module serial port synced.\n\n" +
            "波特率已设置为 " + opBaudVal + "。\n" +
            "模块串口已同步。",
            "info", "OK");
    } else if (type == "force_8n1") {
        setSerialBaudRate(opBaudVal);
        updateValues(opSlave, opBaudVal, "8N1");
        script.log("Servo forced to 8N1, baud=" + opBaudVal);
        util.showMessageBox("Servo Forced to 8N1",
            "Servo protocol set to 8N1.\n" +
            "Communication resumed at " + opBaudVal + " 8N1.\n\n" +
            "伺服协议已设置为 8N1。\n" +
            "通讯已在 " + opBaudVal + " 8N1 下恢复。",
            "info", "OK");
    }
    setModuleEnabled(true);
}

// Modbus 响应帧提取
function tryExtractFrame() {
    if (rxBuffer.length < 5) return null;
    var func = rxBuffer[1];
    var frameLen = 0;
    if (func >= 128) frameLen = 5;
    else if (func == 0x03 || func == 0x04) frameLen = 3 + rxBuffer[2] + 2;
    else if (func == 0x06 || func == 0x05 || func == 0x0F || func == 0x10) frameLen = 8;
    else return null;
    if (rxBuffer.length < frameLen) return null;
    var frame = rxBuffer.slice(0, frameLen);
    rxBuffer = rxBuffer.slice(frameLen);
    return frame;
}

function dataReceived(data) {
    for (var i = 0; i < data.length; i++) rxBuffer.push(data[i]);
    while (true) {
        var frame = tryExtractFrame();
        if (frame == null) break;
        script.log("-RX: " + bytesToStr(frame));
        if (responseValue != null) {
            responseAccumulator = responseAccumulator.length > 0
                ? (responseAccumulator + "\n" + bytesToStr(frame))
                : bytesToStr(frame);
            responseValue.set(responseAccumulator);
        }
        waiting = false;
        if (opActive) {
            if (opType == "set_baud") continueSetBaud(frame);
            else if (opType == "set_slave") continueSetSlave(frame);
            else if (opType == "force_8n1") continueForce8N1(frame);
        }
    }
}

function update(deltaTime) {
    if (waiting) {
        waitTime += deltaTime;
        if (waitTime > 0.5) {
            waiting = false;
            if (opActive) {
                script.log("Timeout: " + opType + " stage " + opStage);
                script.log("TX: " + lastSentHex);
                opActive = false;
                opType = "";
                opStage = 0;
            }
        }
    }
    if (resetPending) {
        resetWaitTime += deltaTime;
        if (resetWaitTime >= RESET_WAIT_SEC) {
            resetPending = false;
            handleResetComplete();
        }
    }
    if (probing) {
        probePolls++;
        if (probePolls >= PROBE_MAX_POLLS) {
            probing = false;
            removeOSModule();
            setModuleEnabled(true);
            script.log("Probe timeout. Check /tmp/probe_result.json and /tmp/probe_ks100.log");
        } else if (probePolls % 5 == 0 && util.fileExists(RESULT_PATH)) {
            var data = util.readFile(RESULT_PATH, true);
            if (data != null && data.status != "probing") {
                probing = false;
                removeOSModule();
                if (data.success) {
                    setSerialBaudRate(data.baud);
                    updateValues(data.slave, data.baud, data.protocol);
                    script.log("Probe: slave=" + data.slave + " baud=" + data.baud + " mode=" + data.protocol);
                    if (data.fa73 != 3) {
                        // 强制改伺服为 8N1 (写 FA-73 + 软复位)
                        startForce8N1(data.slave, data.baud, data.protocol);
                    } else {
                        setModuleEnabled(true);
                    }
                } else {
                    var msg = data.error || "Probe failed";
                    if (data.detail) msg += ": " + data.detail;
                    script.log(msg);
                    setModuleEnabled(true);
                }
            }
        }
    }
}

function init() {
    if (local.values != null) {
        var ci = local.values.getChild("Communication Information");
        if (ci != null) {
            responseValue = ci.getChild("Last Response");
            baudRateValue = ci.getChild("Baud Rate");
            protocolValue = ci.getChild("Protocol");
            slaveAddressValue = ci.getChild("Slave Address");
        }
    }
    script.setUpdateRate(20);
    script.log("Servo RTU KS100 loaded (8N1)");
}

function moduleParameterChanged(param) {
    if (param.niceName == "Get Communication") getCommunication();
}

function moduleCleanedUp() {
    removeOSModule();
}

function sendRaw(hexCommand) {
    if (waiting || probing || opActive) return;
    var body = hexToBytes(hexCommand);
    if (body.length < 2) return;
    responseAccumulator = "";
    sendFrame(body);
}
