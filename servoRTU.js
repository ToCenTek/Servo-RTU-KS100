// Servo RTU KS100 - 简洁版
// Chataigne Serial Module 硬编码 8N1 (来自 module.json defaults, 运行时改不了).
// 探测: 关闭 Chataigne 串口 → Python 扫 baud × mode × slave → force 8N1 → 重开 Chataigne.
// 操作: Set BaudRate (写 FA-72 + 软复位) / Set Slave Address (写 FA-71).

// 状态
var opActive = false;
var opType = "";            // "set_baud" | "set_slave" | "force_8n1"
var opStage = 0;
var opSlave = 1;
var opBaudVal = 0;
var opTime = 0;

var probing = false;
var probePolls = 0;
var PROBE_MAX_POLLS = 1200;  // 240s
var probeRestorePort = "";
var probeUsedEnable = false;  // 用 setEnabled(false) 关闭的(不同于 setPortName)

var closeTimer = 0;
var CLOSE_DELAY = 0.3;      // 关闭 Chataigne 串口后等 300ms 让 Python 打开

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

var RESULT_PATH = "/tmp/probe_result.json";
var SHELL_PATH = "/tmp/probe_ks100.sh";
var OS_NAME = "Servo OS";

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

function getPortName() {
    var p = findParam("port");
    if (p == null) return "";
    return "" + p.get();
}

function setPortName(name) {
    var p = findParam("port");
    if (p == null) return false;
    p.set(name);
    return true;
}

// 启停本模块 (关闭/打开串口, 探测时需关闭避免与 Python pyserial 冲突)
// Chataigne Serial Module 的 Enabled 参数在 local 顶层 (不是 local.parameters)
function setModuleEnabled(enabled) {
    var names = ["Enabled", "enabled", "Enable", "enable", "isEnabled"];
    for (var i = 0; i < names.length; i++) {
        var p = local.getChild(names[i]);
        if (p != null && typeof p.set == "function") {
            p.set(enabled);
            return true;
        }
    }
    return false;
}

// OS Module (探测时需要 subprocess)
function ensureOSModule() {
    var osMod = root.modules.getItemWithName(OS_NAME);
    if (osMod != null) return osMod;
    script.log(OS_NAME + " not found, auto-creating...");
    osMod = root.modules.addItem("OS");
    if (osMod == null) { script.log("Failed to create OS module"); return null; }
    osMod.setName(OS_NAME);
    script.refreshEnvironment();
    return osMod;
}
function removeOSModule() {
    var osMod = root.modules.getItemWithName(OS_NAME);
    if (osMod != null) { root.modules.removeItem(osMod); script.refreshEnvironment(); }
}
function writeShellWrapper() {
    var pyPath = script.getScriptDirectory() + "/scripts/probe_servo.py";
    var content = "#!/bin/bash\nexport PATH=\"/opt/homebrew/bin:/usr/local/bin:$PATH\"\npython3 \"" + pyPath + "\" \"$@\" 2>/tmp/probe_ks100.log\n";
    util.writeFile(SHELL_PATH, content, true);
}

// 命令: Get Communication - 关闭 Chataigne 端口 → Python 探测 (含 force 8N1) → 重开 Chataigne
function getCommunication() {
    if (opActive || probing || closeTimer > 0) return;
    var port = getPortName();
    if (port.length == 0) { script.log("No port configured"); return; }
    probeRestorePort = port;
    script.log("Closing Chataigne port for Python probe...");
    probeUsedEnable = setModuleEnabled(false);
    if (probeUsedEnable) {
        script.log("Module disabled via setEnabled");
    } else {
        script.log("setEnabled failed, falling back to setPortName");
        setPortName("");
    }
    closeTimer = 0.001;
}

function startPythonProbe() {
    var osMod = ensureOSModule();
    if (osMod == null) {
        script.log("Probe aborted: no OS module");
        setPortName(probeRestorePort);
        return;
    }
    writeShellWrapper();
    util.writeFile(RESULT_PATH, '{"success":false,"status":"probing"}', true);
    if (typeof osMod.launchProcess != "function") {
        script.log("OS module has no launchProcess");
        setPortName(probeRestorePort);
        removeOSModule();
        return;
    }
    script.log("Launching Python probe (8N2/8E1/8O1/8N1 × baud × slave)...");
    osMod.launchProcess("/bin/bash " + SHELL_PATH + " --output " + RESULT_PATH, false);
    probing = true;
    probePolls = 0;
}

function onProbeResult(data) {
    probing = false;
    removeOSModule();
    if (data.success) {
        var baud = data.baud;
        var slave = data.slave;
        var proto = data.protocol;
        script.log("Probe: slave=" + slave + " baud=" + baud + " protocol=" + proto + (data.forced ? " (forced 8N1)" : ""));
        updateValues(slave, baud, "8N1");
        setSerialSlaveAddress(slave);
        if (!probeUsedEnable) {
            setPortName(probeRestorePort);  // 恢复 port (setPortName 路径需要)
        }
        setSerialBaudRate(baud);
        setModuleEnabled(true);
        var detail = "Slave: " + slave + "\nBaud: " + baud + "\nProtocol: " + proto + " (now 8N1)";
        if (data.forced) {
            detail = "Servo was " + proto + ", forced to 8N1.\n" + detail;
        }
        var detailCN = "从站: " + slave + "\n波特: " + baud + "\n协议: " + proto + " (已强制 8N1)";
        if (data.forced) {
            detailCN = "伺服原为 " + proto + ", 已强制为 8N1。\n" + detailCN;
        }
        util.showMessageBox("Probe Complete", detail + "\n\n" + detailCN, "info", "OK");
    } else {
        var msg = data.error || "Probe failed";
        if (data.detail) msg += ": " + data.detail;
        script.log(msg);
        if (!probeUsedEnable) {
            setPortName(probeRestorePort);
        }
        setModuleEnabled(true);
        util.showMessageBox("Probe Failed", msg + "\n\n" + msg, "warning", "OK");
    }
}

// 命令: Set BaudRate - 写 FA-72 + 软复位 (servo 已在 8N1)
function setBaudRate(slave, baud) {
    if (opActive || probing || closeTimer > 0) return;
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
    if (opActive || probing || closeTimer > 0) return;
    var newNum = parseInt(newSlave, 10);
    if (newNum < 1 || newNum > 254) return;
    var p = findParam("slaveAddress");
    var currentSlave = (p != null) ? p.get() : 1;
    if (currentSlave == newNum) { script.log("Slave already " + newNum); return; }
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
        if (opType == "set_baud") continueSetBaud(frame);
        else if (opType == "set_slave") continueSetSlave(frame);
    }
}

function update(deltaTime) {
    if (closeTimer > 0) {
        closeTimer += deltaTime;
        if (closeTimer >= CLOSE_DELAY) {
            closeTimer = 0;
            startPythonProbe();
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
    if (probing) {
        probePolls++;
        if (probePolls >= PROBE_MAX_POLLS) {
            probing = false;
            removeOSModule();
            setPortName(probeRestorePort);
            script.log("Probe timeout. Check /tmp/probe_result.json and /tmp/probe_ks100.log");
        } else if (probePolls % 10 == 0 && util.fileExists(RESULT_PATH)) {
            var data = util.readFile(RESULT_PATH, true);
            if (data != null && data.status != "probing") {
                onProbeResult(data);
            }
        }
        return;
    }
    if (opActive) {
        opTime += deltaTime;
        if (opTime > 1.0) {
            opActive = false;
            opType = "";
            script.log("Operation timeout");
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
    script.log("Servo RTU KS100 loaded (8N1, Get Comm uses Python probe)");
}

function moduleParameterChanged(param) {
    if (param.niceName == "Get Communication") getCommunication();
}

function moduleCleanedUp() {
    removeOSModule();
}

function sendRaw(hexCommand) {
    if (opActive || probing || closeTimer > 0) return;
    var body = hexToBytes(hexCommand);
    if (body.length < 2) return;
    responseAccumulator = "";
    sendFrame(body);
}
