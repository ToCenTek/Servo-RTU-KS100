// Servo RTU KS100 - 极简版
// Chataigne Serial Module 硬编码 8N1 (来自 module.json defaults, 运行时改不了).
// 探测: 用 Chataigne 自家串口扫 baud × slave (8N1 fixed).
// 操作: Set BaudRate (写 FA-72 + 软复位) / Set Slave Address (写 FA-71).
// 假设: 伺服默认 8N1 (KS100 出厂默认). 非 8N1 需先用厂家工具改 8N1.

// 状态
var opActive = false;
var opType = "";            // "probe" | "set_baud" | "set_slave" | "force_8n1"
var opStage = 0;
var opSlave = 1;
var opBaudVal = 0;
var opTime = 0;             // 当前操作已用时间

var probeBaudIdx = 0;
var PROBE_BAUDS = [115200, 57600, 38400, 19200, 9600, 4800];
var probeSwitchTime = 0;    // 切换 baud 后等 300ms 再发读
var PROBE_SWITCH_DELAY = 0.3;

var resetPending = false;
var resetTime = 0;
var RESET_WAIT_SEC = 2.5;

var rxBuffer = [];

var responseValue = null;
var baudRateValue = null;
var protocolValue = null;
var slaveAddressValue = null;
var responseAccumulator = "";

// 寄存器
var REG_FA60 = 0x003C;  // 软复位
var REG_FA71 = 0x0047;  // 从站地址
var REG_FA72 = 0x0048;  // 波特率
var REG_FA73 = 0x0049;  // 协议 (3 = 8N1)

// CRC16-Modbus (查表法, 算术实现避开位运算符)
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
    script.log("-TX: " + bytesToStr(pdu));
    if (responseAccumulator.length > 0) responseAccumulator += "\n";
    responseAccumulator += "TX: " + bytesToStr(pdu);
    if (responseValue != null) responseValue.set(responseAccumulator);
    opTime = 0;
    rxBuffer = [];
    local.sendBytes(pdu);
}

function findParam(name) {
    if (local.parameters != null) {
        var p = local.parameters.getChild(name);
        if (p != null) return p;
    }
    return local.getChild(name);
}

function updateValues(slave, baud, protocol) {
    if (slaveAddressValue != null) slaveAddressValue.set("" + slave);
    if (baudRateValue != null) baudRateValue.set("" + baud);
    if (protocolValue != null) protocolValue.set(protocol);
    var ci = local.values.getChild("Communication Information");
    if (ci != null) ci.setCollapsed(false);
}

function setSerialBaudRate(baud) {
    var p = findParam("baudRate");
    if (p == null) return false;
    p.set(baud);
    return true;
}

function setSerialSlaveAddress(slave) {
    var p = findParam("slaveAddress");
    if (p == null) return false;
    p.set(slave);
    return true;
}

// 解析 read 0x03 响应: N 字节数据
function parseReadResponse(frame, count) {
    if (frame[1] != 0x03 || frame[2] != count * 2) return null;
    var regs = [];
    for (var i = 0; i < count; i++) {
        regs.push(frame[3 + i * 2] * 256 + frame[4 + i * 2]);
    }
    return regs;
}

// 命令: Get Communication - 用 Chataigne 串口扫 baud (8N1 fixed, slave 1)
function getCommunication() {
    if (opActive || probeSwitchTime > 0) return;
    probeBaudIdx = 0;
    script.log("Probing servo (8N1, slave 1, baud " + PROBE_BAUDS.join("/") + ")...");
    probeNextBaud();
}

function probeNextBaud() {
    if (probeBaudIdx >= PROBE_BAUDS.length) {
        script.log("Probe failed. No servo at 8N1.");
        script.log("If servo is 8N2/8E1/8O1, use vendor tool to set 8N1 first.");
        return;
    }
    var baud = PROBE_BAUDS[probeBaudIdx];
    script.log("Probe: trying baud " + baud);
    setSerialBaudRate(baud);
    probeSwitchTime = 0.001;  // 非零表示正在等切换
}

function continueProbe(frame) {
    if (frame[1] == 0x03) {
        var regs = parseReadResponse(frame, 3);
        if (regs == null) { probeBaudIdx++; probeNextBaud(); opActive = false; opType = ""; return; }
        opActive = false;
        opType = "";
        var slave = regs[0];
        var baud = regs[1] * 100;
        var fa73 = regs[2];
        var protoLabel = fa73 == 0 ? "8N2" : fa73 == 1 ? "8E1" : fa73 == 2 ? "8O1" : "8N1";
        script.log("Probe OK: slave=" + slave + " baud=" + baud + " protocol=" + protoLabel);
        setSerialBaudRate(baud);
        setSerialSlaveAddress(slave);
        updateValues(slave, baud, protoLabel);
        if (fa73 != 3) {
            startForce8N1(slave, baud);
        } else {
            util.showMessageBox("Probe Complete",
                "Servo found.\n" +
                "  Slave: " + slave + "\n" +
                "  Baud:  " + baud + "\n" +
                "  Protocol: " + protoLabel + "\n\n" +
                "伺服已找到。",
                "info", "OK");
        }
    } else {
        opActive = false;
        opType = "";
        probeBaudIdx++;
        probeNextBaud();
    }
}

// 命令: Set BaudRate - 写 FA-72 + 软复位
function setBaudRate(slave, baud) {
    if (opActive || probeSwitchTime > 0) return;
    var slaveNum = parseInt(slave, 10);
    var baudNum = parseInt(baud, 10);
    if (slaveNum < 1 || slaveNum > 254) return;
    if (baudNum < 300 || baudNum > 115200) return;
    opActive = true;
    opType = "set_baud";
    opStage = 1;
    opSlave = slaveNum;
    opBaudVal = baudNum;
    script.log("Set baud: slave=" + slaveNum + " baud=" + baudNum);
    sendFrame(makeWrite(slaveNum, REG_FA72, Math.floor(baudNum / 100)));
}

function continueSetBaud(frame) {
    if (frame[1] != 0x06) {
        opActive = false; opType = "";
        script.log("Set baud failed");
        return;
    }
    if (opStage == 1) {
        opStage = 2;
        sendFrame(makeWrite(opSlave, REG_FA60, 1));
        return;
    }
    if (opStage == 2) {
        opActive = false;
        script.log("Reset sent, waiting " + RESET_WAIT_SEC + "s...");
        resetPending = true;
        resetTime = 0;
    }
}

// 命令: Set Slave Address - 写 FA-71
function setSlaveAddress(newSlave) {
    if (opActive || probeSwitchTime > 0) return;
    var newNum = parseInt(newSlave, 10);
    if (newNum < 1 || newNum > 254) return;
    var p = findParam("slaveAddress");
    var currentSlave = (p != null) ? p.get() : 1;
    if (currentSlave == newNum) {
        script.log("Slave already " + newNum);
        return;
    }
    opActive = true;
    opType = "set_slave";
    opStage = 1;
    opSlave = newNum;
    script.log("Set slave: " + currentSlave + " -> " + newNum);
    sendFrame(makeWrite(currentSlave, REG_FA71, newNum));
}

function continueSetSlave(frame) {
    if (frame[1] != 0x06) {
        opActive = false; opType = "";
        script.log("Set slave failed");
        return;
    }
    opActive = false;
    opType = "";
    setSerialSlaveAddress(opSlave);
    script.log("Slave updated to " + opSlave);
    util.showMessageBox("Slave Address Updated",
        "Slave address updated to " + opSlave + ".\n\n" +
        "从站地址已更新到 " + opSlave + "。",
        "info", "OK");
}

// 强制伺服为 8N1 (探测成功后, 如果 FA-73 != 3)
function startForce8N1(slave, baud) {
    opActive = true;
    opType = "force_8n1";
    opStage = 1;
    opSlave = slave;
    opBaudVal = baud;
    script.log("Forcing servo to 8N1...");
    sendFrame(makeWrite(slave, REG_FA73, 3));
}

function continueForce8N1(frame) {
    if (frame[1] != 0x06) {
        opActive = false; opType = "";
        script.log("Force 8N1 failed");
        return;
    }
    if (opStage == 1) {
        opStage = 2;
        sendFrame(makeWrite(opSlave, REG_FA60, 1));
        return;
    }
    if (opStage == 2) {
        opActive = false;
        script.log("Force 8N1: reset sent, waiting...");
        resetPending = true;
        resetTime = 0;
    }
}

function handleResetComplete() {
    var type = opType;
    opType = "";
    if (type == "set_baud") {
        setSerialBaudRate(opBaudVal);
        updateValues(opSlave, opBaudVal, "8N1");
        script.log("Baud set to " + opBaudVal);
        util.showMessageBox("Baud Rate Set",
            "Baud rate set to " + opBaudVal + ".\n\n" +
            "波特率已设置为 " + opBaudVal + "。",
            "info", "OK");
    } else if (type == "force_8n1") {
        setSerialBaudRate(opBaudVal);
        updateValues(opSlave, opBaudVal, "8N1");
        script.log("Servo forced to 8N1 at " + opBaudVal);
        util.showMessageBox("Servo Forced to 8N1",
            "Servo protocol set to 8N1.\n" +
            "Communication resumed at " + opBaudVal + ".\n\n" +
            "伺服协议已设置为 8N1。\n" +
            "通讯已在 " + opBaudVal + " 下恢复。",
            "info", "OK");
    }
}

function tryExtractModbusFrame() {
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
        var frame = tryExtractModbusFrame();
        if (frame == null) break;
        script.log("-RX: " + bytesToStr(frame));
        if (responseAccumulator.length > 0) responseAccumulator += "\n";
        responseAccumulator += "RX: " + bytesToStr(frame);
        if (responseValue != null) responseValue.set(responseAccumulator);
        if (!opActive) continue;
        if (opType == "probe") continueProbe(frame);
        else if (opType == "set_baud") continueSetBaud(frame);
        else if (opType == "set_slave") continueSetSlave(frame);
        else if (opType == "force_8n1") continueForce8N1(frame);
    }
}

function update(deltaTime) {
    // 等待 baud 切换完成后发读
    if (probeSwitchTime > 0) {
        probeSwitchTime += deltaTime;
        if (probeSwitchTime >= PROBE_SWITCH_DELAY) {
            probeSwitchTime = 0;
            opActive = true;
            opType = "probe";
            opStage = 1;
            opSlave = 1;
            opBaudVal = PROBE_BAUDS[probeBaudIdx];
            sendFrame(makeRead(1, REG_FA71, 3));
        }
        return;
    }
    if (resetPending) {
        resetTime += deltaTime;
        if (resetTime >= RESET_WAIT_SEC) {
            resetPending = false;
            handleResetComplete();
        }
        return;
    }
    if (opActive) {
        opTime += deltaTime;
        if (opTime > 1.0) {
            var type = opType;
            opActive = false;
            opType = "";
            if (type == "probe") {
                probeBaudIdx++;
                probeNextBaud();
            } else {
                script.log(type + " timeout");
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
    script.log("Servo RTU KS100 loaded (8N1, using Chataigne port for probe)");
}

function moduleParameterChanged(param) {
    if (param.niceName == "Get Communication") getCommunication();
}

function moduleCleanedUp() {}

function sendRaw(hexCommand) {
    if (opActive || probeSwitchTime > 0) return;
    var body = hexToBytes(hexCommand);
    if (body.length < 2) return;
    responseAccumulator = "";
    sendFrame(body);
}
