var responseValue = null;
var baudRateValue = null;
var protocolValue = null;
var slaveAddressValue = null;

var waiting = false;
var waitTime = 0;
var rxBuffer = [];

var probing = false;
var probeSlave = 1;
var probePollCount = 0;
var probeTotalPolls = 0;
var probeMaxPolls = 1200;  // 240s, 给 1~254 全扫留足时间
var probeRestoreBaud = 0;
var probeRestoreMode = -1;

var opActive = false;
var opType = "";
var opStage = 0;
var opSlave = 1;
var opBaudVal = 0;
var opModeVal = 0;

// 软复位后等待驱动器重启的状态
var resetPending = false;
var resetWaitTime = 0;
var RESET_WAIT_SEC = 2.5;

// 最后一次成功验证(读回的伺服实际值)的通讯参数
// 用于 init() 时还原 Communication Information 显示
var confirmedBaud = -1;
var confirmedMode = -1;
var confirmedSlave = -1;

var lastSentHex = "";
var responseAccumulator = "";

var RESULT_PATH = "/tmp/probe_result.json";
var SHELL_PATH = "/tmp/probe_ks100.sh";
var OS_NAME = "Servo OS";

var REG_FA60 = 0x003C;  // 软复位(写 1 触发)
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
// 完全照搬 modbusRTU.js (ToCenTek-modbus-RTU) 的简洁模式: 不用 appendCRC 新建数组, 直接 push 到 pdu
function sendFrame(pdu) {
    var c = crc16(pdu);
    pdu.push(c[0], c[1]);
    script.log("-TX: " + bytesToStr(pdu));
    rxBuffer = [];
    waiting = true;
    waitTime = 0;
    local.sendBytes(pdu);
}

// 十六进制字符转半字节值
function hexNibble(c) {
    var code = c.charCodeAt(0);
    if (code >= 48 && code <= 57) return code - 48;
    if (code >= 65 && code <= 70) return code - 55;
    if (code >= 97 && code <= 102) return code - 87;
    return -1;
}

// 十六进制字符串转字节数组
function hexToBytes(text) {
    var clean = "";
    var i = 0;
    for (i = 0; i < text.length; i++) {
        var c = text.charAt(i);
        if (hexNibble(c) >= 0) clean = clean + c;
    }
    if ((clean.length % 2) != 0) return [];
    var bytes = [];
    for (i = 0; i < clean.length; i = i + 2) {
        bytes.push(hexNibble(clean.charAt(i)) * 16 + hexNibble(clean.charAt(i + 1)));
    }
    return bytes;
}

// 字节转 2 字符十六进制字符串
function toHexByte(v) {
    var s = "0123456789ABCDEF";
    return s.charAt(Math.floor(v / 16) % 16) + s.charAt(v % 16);
}

// 字节数组转空格分隔十六进制字符串
function bytesToStr(bytes) {
    var out = "";
    var i = 0;
    for (i = 0; i < bytes.length; i++) {
        if (i > 0) out = out + " ";
        out = out + toHexByte(bytes[i]);
    }
    return out;
}

// 按多个名称变体搜索模块中的可写参数
function findParam(names) {
    var i = 0;
    var p = null;
    // Chataigne Serial Module 内置参数在 local.parameters 下
    if (local.parameters != null) {
        for (i = 0; i < names.length; i++) {
            p = local.parameters.getChild(names[i]);
            if (p != null) return p;
        }
    }
    // 也可能在 local 下的 serial/Serial 子对象
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

// 尝试同步 Chataigne 串口参数
// Chataigne Serial Module 暴露的 parameters (从 dumpLocal 看到):
//   baudRate, slaveAddress, port, protocol, autoAdd, messageStructure,
//   customSeparator, firstValueIsTheName, pass_through, dtr, rts, isConnected
//   (没有 DataBits/Parity/StopBits -- Chataigne 不暴露, 可能 C++ 层硬编码)
// 所以只能设 baudRate. dataBits/parity/stopBits 假设模块已配好
// 返回 true 表示至少波特率被设置
function setSerialConfig(baud, mode) {
    var parts = modeToSerial(mode);
    // BaudRate -- Chataigne 参数实际叫 baudRate, 但 getChild 大小写不敏感
    var baudP = findParam(["baudRate", "BaudRate", "baudrate", "baud_rate"]);
    if (baudP != null) {
        baudP.set(baud);
    }
    // DataBits/Parity/StopBits 不可改 (Chataigne 不暴露这些参数)
    // 假设: 用户使用时这些参数已通过 GUI / defaults 块配置好
    return (baudP != null);
}

// 手写 indexOf (JUCE 引擎不支持 s.indexOf)
function strIndexOf(s, sub, start) {
    if (sub.length == 0) return start || 0;
    if (start == null) start = 0;
    if (s.length < sub.length + start) return -1;
    var lastStart = s.length - sub.length;
    for (var i = start; i <= lastStart; i++) {
        var match = true;
        for (var j = 0; j < sub.length; j++) {
            if (s.charAt(i + j) != sub.charAt(j)) {
                match = false;
                break;
            }
        }
        if (match) return i;
    }
    return -1;
}

// 在 JSON 文本中替换 "key": <value> 字段值
// key 包含引号 (e.g. '"baudRate"')
// 找不到时返回原 content (不中断, 让调用方决定)
// 不用 RegExp, 不用 String.indexOf, 用 strIndexOf + charAt + substring
function simpleReplaceField(content, key, newValue) {
    var keyIdx = strIndexOf(content, key);
    if (keyIdx < 0) return content;  // 找不到, 返回原内容
    var colonIdx = strIndexOf(content, ':', keyIdx);
    if (colonIdx < 0) return content;
    var valStart = colonIdx + 1;
    while (valStart < content.length) {
        var c = content.charAt(valStart);
        if (c != ' ' && c != '\n' && c != '\r' && c != '\t') break;
        valStart++;
    }
    var valEnd = valStart;
    if (valStart < content.length && content.charAt(valStart) == '"') {
        // 字符串值: 找结束的 "
        valEnd = strIndexOf(content, '"', valStart + 1);
        if (valEnd < 0) return content;
        valEnd = valEnd + 1;  // 包括结尾的引号
    } else {
        // 数字值: 找 , 或 } 或 \n
        while (valEnd < content.length) {
            var c2 = content.charAt(valEnd);
            if (c2 == ',' || c2 == '}' || c2 == '\n') break;
            valEnd++;
        }
    }
    return content.substring(0, valStart) + newValue + content.substring(valEnd);
}

// 从 module.json 的 defaults 块读出当前 4 个值
// 返回: { BaudRate, DataBits, Parity, StopBits } 或 null (失败)
function readCurrentDefaults() {
    var dir = script.getScriptDirectory();
    var modulePath = dir + "/module.json";
    if (!util.fileExists(modulePath)) return null;
    var content = util.readFile(modulePath, false);
    if (content == null || typeof content.substring != "function") return null;
    var defKey = strIndexOf(content, '"defaults"');
    if (defKey < 0) return null;
    var braceStart = strIndexOf(content, '{', defKey);
    if (braceStart < 0) return null;
    var blockEnd = strIndexOf(content, '"scripts"', braceStart);
    if (blockEnd < 0) blockEnd = strIndexOf(content, '"parameters"', braceStart);
    if (blockEnd < 0) return null;
    var block = content.substring(braceStart, blockEnd);
    // 提取单字段值
    function getVal(field) {
        var idx = strIndexOf(block, '"' + field + '"');
        if (idx < 0) return null;
        var colon = strIndexOf(block, ':', idx);
        if (colon < 0) return null;
        var start = colon + 1;
        while (start < block.length) {
            var c = block.charAt(start);
            if (c != ' ' && c != '\n' && c != '\r' && c != '\t') break;
            start++;
        }
        if (start >= block.length) return null;
        var end;
        if (block.charAt(start) == '"') {
            end = strIndexOf(block, '"', start + 1);
            if (end < 0) return null;
            return block.substring(start + 1, end);
        }
        end = start;
        while (end < block.length) {
            var c2 = block.charAt(end);
            if (c2 == ',' || c2 == '}' || c2 == '\n') break;
            end++;
        }
        return block.substring(start, end);
    }
    var baudStr = getVal("baudRate");
    var dataStr = getVal("dataBits");
    var parity = getVal("Parity");
    var stopStr = getVal("stopBits");
    return {
        BaudRate: baudStr == null ? -1 : parseInt(baudStr, 10),
        DataBits: dataStr == null ? -1 : parseInt(dataStr, 10),
        Parity: parity,
        StopBits: stopStr == null ? -1 : parseInt(stopStr, 10)
    };
}

// 检查 mode (DataBits/Parity/StopBits) 是否需要 Reload 才能应用
// BaudRate 改变不需要 Reload (Chataigne 实时改)
// mode 改变需要 Reload (Chataigne 模块创建时读)
function modeNeedsReload(mode) {
    var def = readCurrentDefaults();
    if (def == null) return true;  // 读不到, 保守起见假设需要
    var parts = modeToSerial(mode);
    if (def.DataBits != parts[0]) return true;
    if (def.Parity != null && def.Parity != parts[1]) return true;
    if (def.StopBits != parts[2]) return true;
    return false;
}

// 安全更新 module.json defaults 块 (4 个字段)
// 保留原文件格式 (中文/缩进/换行), 写后用 JSON.parse 验证
// 用 simpleReplaceField (找不到时返回原内容, 不中断)
function safeUpdateDefaults(baud, mode) {
    var dir = script.getScriptDirectory();
    var modulePath = dir + "/module.json";
    if (!util.fileExists(modulePath)) {
        script.logWarning("safeUpdateDefaults: module.json not found");
        return false;
    }
    var content = util.readFile(modulePath, false);
    if (content == null) {
        script.logWarning("safeUpdateDefaults: readFile returned null");
        return false;
    }
    var parts = modeToSerial(mode);
    script.log("safeUpdateDefaults: parts=[" + parts[0] + ",\"" + parts[1] + "\"," + parts[2] + "] baud=" + baud);
    // 直接在整个 content 中找并替换 4 个字段 (Chataigne 用 camelCase)
    var newContent = simpleReplaceField(content, '"baudRate"', "" + baud);
    newContent = simpleReplaceField(newContent, '"dataBits"', "" + parts[0]);
    newContent = simpleReplaceField(newContent, '"Parity"', '"' + parts[1] + '"');
    newContent = simpleReplaceField(newContent, '"stopBits"', "" + parts[2]);
    if (newContent == content) {
        script.logWarning("safeUpdateDefaults: no fields changed (one or more not found)");
        return false;
    }
    script.log("safeUpdateDefaults: content len=" + content.length + " newContent len=" + newContent.length);
    if (typeof JSON.parse == "function") {
        JSON.parse(newContent);
    }
    util.writeFile(modulePath, newContent, true);
    // 验证写入
    var verify = util.readFile(modulePath, false);
    if (verify != null) {
        script.log("safeUpdateDefaults: verify len=" + verify.length + " (orig " + content.length + ")");
    }
    script.log("safeUpdateDefaults: wrote baudRate=" + baud + " dataBits=" + parts[0] + " Parity=\"" + parts[1] + "\" stopBits=" + parts[2]);
    return true;
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
    script.log("Shell wrapper written: " + SHELL_PATH);
    script.log("Python path: " + pyPath);
    // 也直接尝试给 shell wrapper 加执行权限 (macOS/Linux)
    // 注: ES3 没有 chmod API, 只能靠 util.writeFile 保留执行位
}

// 更新探测结果到 Communication Information
function updateDetectedValues(baud, slave, mode) {
    probeRestoreBaud = baud;
    probeRestoreMode = mode;
    confirmedBaud = baud;
    confirmedMode = mode;
    confirmedSlave = slave;
    if (baudRateValue != null) {
        baudRateValue.set("" + baud);
    }
    if (protocolValue != null) {
        protocolValue.set(modeLabel(mode));
    }
    if (slaveAddressValue != null) {
        slaveAddressValue.set("" + slave);
    }
    var ci = local.values.getChild("Communication Information");
    if (ci != null) {
        ci.setCollapsed(false);
    }
    script.log("Servo found: slave " + slave + ", baud " + baud + ", mode " + modeLabel(mode));
}

// 将 module.json 的 defaults 中 DataBits/Parity/StopBits 还原为 FA-73 数值
// (无法识别时返回 -1)
function serialDefaultsToMode(def) {
    if (def == null) return -1;
    var db = def.DataBits;
    var pa = def.Parity;
    var sb = def.StopBits;
    if (db == 8 && pa == "None" && sb == 2) return 0;  // 8N2
    if (db == 8 && pa == "Even" && sb == 1) return 1;  // 8E1
    if (db == 8 && pa == "Odd"  && sb == 1) return 2;  // 8O1
    if (db == 8 && pa == "None" && sb == 1) return 3;  // 8N1
    return -1;
}

// 手写 indexOf (JUCE 引擎不支持 s.indexOf)
// 在 s 中找 sub, 找到返回索引, 找不到返回 -1
// start: 可选起始位置
function strIndexOf(s, sub, start) {
    if (sub.length == 0) return start || 0;
    if (start == null) start = 0;
    if (s.length < sub.length + start) return -1;
    var lastStart = s.length - sub.length;
    for (var i = start; i <= lastStart; i++) {
        var match = true;
        for (var j = 0; j < sub.length; j++) {
            if (s.charAt(i + j) != sub.charAt(j)) {
                match = false;
                break;
            }
        }
        if (match) return i;
    }
    return -1;
}

// 在 JSON 文本中替换 "fieldName": <value> 字段值
// 不用 RegExp, 不用 String.indexOf (JUCE 引擎不支持), 用 strIndexOf + charAt + substring
function replaceJsonField(content, fieldName, newValue) {
    var key = '"' + fieldName + '"';
    var keyIdx = strIndexOf(content, key);
    if (keyIdx < 0) return null;
    var colonIdx = strIndexOf(content, ':', keyIdx);  // 找 key 后的冒号
    if (colonIdx < 0) return null;
    // 找 value 起始: 跳过冒号后的空白
    var valStart = colonIdx + 1;
    while (valStart < content.length) {
        var c = content.charAt(valStart);
        if (c != ' ' && c != '\n' && c != '\r' && c != '\t') break;
        valStart++;
    }
    // 找 value 结束
    var valEnd = valStart;
    if (valStart < content.length && content.charAt(valStart) == '"') {
        valEnd = strIndexOf(content, '"', valStart + 1);
        if (valEnd < 0) return null;
    } else {
        while (valEnd < content.length) {
            var c2 = content.charAt(valEnd);
            if (c2 == ',' || c2 == '}' || c2 == '\n') break;
            valEnd++;
        }
    }
    return content.substring(0, valStart) + newValue + content.substring(valEnd);
}

// 模块初始化：获取值对象引用
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
    // 从 module.json 的 defaults 还原 Communication Information 显示
    // 这样 Set Comm 验证后 Reload Modules 也能看到新值
    var dir = script.getScriptDirectory();
    var modulePath = dir + "/module.json";
    if (util.fileExists(modulePath)) {
        var json = util.readFile(modulePath, true);
        if (json != null && json.defaults != null) {
            var d = json.defaults;
            if (baudRateValue != null && d.BaudRate != null) {
                baudRateValue.set("" + d.BaudRate);
            }
            var m = serialDefaultsToMode(d);
            if (protocolValue != null && m >= 0) {
                protocolValue.set(modeLabel(m));
            }
        }
    }
    script.setUpdateRate(20);
    script.log("Servo RTU KS100 loaded");
}

// 周期调用：处理超时、软复位等待、探测结果轮询
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
                        script.log("Set communication failed at stage 1 (baud " + opBaudVal + ")");
                        script.log("Servo did not respond. Module serial may not match servo.");
                    } else if (timedOutStage == 2) {
                        script.log("Set communication failed at stage 2 (mode " + modeLabel(opModeVal) + ")");
                        script.log("Baud was updated to " + opBaudVal + " but mode change failed.");
                    } else if (timedOutStage == 3) {
                        script.log("Set communication failed at stage 3 (soft reset)");
                    }
                } else if (timedOutOp == "verify_comm") {
                    script.log("Verification read failed. Servo may not have applied new params.");
                } else if (timedOutOp == "set_slave") {
                    script.log("Set slave address failed: no response from slave " + opBaudVal);
                    script.log("The servo is NOT actually at slave " + opBaudVal + ".");
                    script.log("Possible causes:");
                    script.log("  1. Module serial (baud/dataBits/parity/stopBits) does not match servo.");
                    script.log("     After Probe Communication, Reload Custom Modules to apply new defaults.");
                    script.log("  2. Servo is on a different slave address.");
                    script.log("  3. Wiring issue (485+/485- swapped, or no power to servo).");
                    script.log("Run Probe Communication to find the actual configuration.");
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

    if (resetPending) {
        resetWaitTime = resetWaitTime + deltaTime;
        if (resetWaitTime >= RESET_WAIT_SEC) {
            resetPending = false;
            handleResetComplete();
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
                        script.log("Probe complete: " + data.baud + " " + data.protocol + " slave " + data.slave);
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
                script.log("Probe timeout (" + (probeMaxPolls * 2) + "s). Check:");
                script.log("  - /tmp/probe_result.json (should exist after probe)");
                script.log("  - /tmp/probe_ks100.log (Python stderr)");
                script.log("  - OS module has launchProcess");
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

// 命令：获取伺服通信参数
// 从站 1 开始扫描, 到 254 为止, 找到第一个有效响应即停止
// 假设总线上只有 1 个设备
// 实际从站地址从 FA-71 寄存器读出
function getCommunication() {
    if (waiting || probing) return;
    probeSlave = 1;
    probing = false;

    util.showMessageBox("Please wait...",
        "探测伺服通信参数 (从 slave 1 开始扫描, 找到即停)...",
        "info", "OK");

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

    var cmd = "/bin/bash " + SHELL_PATH + " --output " + RESULT_PATH;

    script.log("Probing servo parameters (slave 1~254)");
    script.log("Probe cmd: " + cmd);
    script.log("Probe script dir: " + script.getScriptDirectory());

    util.writeFile(RESULT_PATH, '{"success":false,"status":"probing"}', true);

    if (typeof osMod.launchProcess == "function") {
        script.log("Launching probe process (non-blocking)...");
        // 第二参数 blocking=false: 非阻塞, 不能让 JS 引擎卡住
        osMod.launchProcess(cmd, false);
        script.log("Probe process launched");
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

// 多步操作：写 FA-72 波特率 → 写 FA-73 协议 → 写 FA-60=1 软复位
// 每一步都直接写保存地址(不是 +0x0080 暂存),使改动立即写入 EEPROM
// 软复位后驱动器重启,新参数生效
function continueSetComm(frame) {
    if (frame[1] != 0x06) {
        opActive = false;
        opType = "";
        opStage = 0;
        script.log("Set communication failed: unexpected response 0x" + toHexByte(frame[1]));
        return;
    }
    if (opStage == 1) {
        opStage = 2;
        // Step 2: 写 FA-73 协议模式
        sendFrame(makeWrite(opSlave, REG_FA73, opModeVal));
        return;
    }
    if (opStage == 2) {
        opStage = 3;
        // Step 3: 写 FA-60 = 1 触发软复位,驱动器重启后用新参数
        sendFrame(makeWrite(opSlave, REG_FA60, 1));
        return;
    }
    if (opStage == 3) {
        // 软复位命令已发出,进入等待重启阶段
        opActive = false;
        opType = "";
        opStage = 0;
        script.log("Reset command sent. Waiting " + RESET_WAIT_SEC + "s for servo reboot...");
        resetPending = true;
        resetWaitTime = 0;
        return;
    }
}

// 软复位等待结束: 同步 Chataigne Serial Module 内置参数, 调 init() 重新初始化
// 不再修改 module.json 文件 (避免格式破坏)
// 流程: setSerialConfig() 设置 BaudRate/DataBits/Parity/StopBits
//        Chataigne 听到变化自动让 Serial Module 重连串口
//        init() 重新解析 parameters + 更新 values 显示
// 删除当前模块, 让 Chataigne Reload 后重建 (用新 module.json defaults)
function tryRemoveCurrentModule() {
    if (root == null) return;
    if (root.modules == null) return;
    if (typeof root.modules.getItemWithName != "function") return;
    if (typeof root.modules.removeItem != "function") return;
    var mod = root.modules.getItemWithName("Servo RTU KS100");
    if (mod == null) return;
    root.modules.removeItem(mod);
}

function handleResetComplete() {
    // 1) 改 parameters.baudRate (Chataigne 实时改, 不需 Reload)
    var ok = setSerialConfig(opBaudVal, opModeVal);
    if (ok) {
        script.log("Module baudRate set: " + opBaudVal);
    } else {
        script.logWarning("Module baudRate not found; update manually");
    }
    // 2) 改 module.json defaults (4 字段, 包括 baudRate)
    if (safeUpdateDefaults(opBaudVal, opModeVal)) {
        script.log("module.json defaults updated");
    } else {
        script.logWarning("Failed to update module.json defaults");
    }
    // 3) 判断 mode (DataBits/Parity/StopBits) 是否变化
    //    只有 mode 变了才需要 Reload (Chataigne 模块创建时读 mode)
    //    BaudRate 变不需 Reload (Chataigne 实时改)
    var needsReload = modeNeedsReload(opModeVal);
    if (needsReload) {
        // 弹窗提示需要 Reload
        util.showMessageBox(
            "Servo Communication Updated (Mode Changed)",
            "Servo communication updated.\n" +
            "  Slave: " + opSlave + "\n" +
            "  Baud:  " + opBaudVal + "\n" +
            "  Mode:  " + modeLabel(opModeVal) + "\n" +
            "\n" +
            "Mode (DataBits/Parity/StopBits) changed.\n" +
            "Chataigne needs to Reload to apply new mode.\n" +
            "The current module will be removed now.\n" +
            "Please Reload Custom Modules to re-add it.\n" +
            "  Module menu > Reload Custom Modules\n" +
            "\n" +
            "伺服通讯已更新 (模式已改变)。\n" +
            "  从站:  " + opSlave + "\n" +
            "  波特:  " + opBaudVal + "\n" +
            "  模式:  " + modeLabel(opModeVal) + "\n" +
            "\n" +
            "模式 (DataBits/Parity/StopBits) 已改变, Chataigne 需要 Reload 应用新模式。\n" +
            "当前模块将被删除, 请 Reload Custom Modules 重新加载。\n" +
            "  模块菜单 > 重新加载自定义模块",
            "info",
            "OK"
        );
        // 删除当前模块, 用户手动 Reload
        tryRemoveCurrentModule();
    } else {
        // Mode 没变, 只需要 BaudRate 改变, 不需 Reload
        util.showMessageBox(
            "Servo Communication Updated (BaudRate Only)",
            "Servo communication updated.\n" +
            "  Slave: " + opSlave + "\n" +
            "  Baud:  " + opBaudVal + "\n" +
            "  Mode:  " + modeLabel(opModeVal) + " (unchanged)\n" +
            "\n" +
            "BaudRate applied to module's parameters.\n" +
            "No Reload needed. Module continues to work.\n" +
            "\n" +
            "伺服通讯已更新 (只改波特率)。\n" +
            "  从站:  " + opSlave + "\n" +
            "  波特:  " + opBaudVal + "\n" +
            "  模式:  " + modeLabel(opModeVal) + " (未变)\n" +
            "\n" +
            "波特率已应用到模块参数, 无需 Reload, 模块继续工作。",
            "info",
            "OK"
        );
        // 不删除, 模块继续工作
    }
}

// 多步操作：写 FA-71 从站地址
// 成功后: 同步更新 UI 中的 Slave Address 参数
function continueSetSlave(frame) {
    if (frame[1] != 0x06) {
        opActive = false;
        opType = "";
        opStage = 0;
        script.log("Set slave failed: unexpected response 0x" + toHexByte(frame[1]));
        return;
    }
    opActive = false;
    opType = "";
    opStage = 0;
    // 同步更新 Chataigne Serial Module 内置 slaveAddress 参数 (实际叫小写, 无空格)
    if (local.parameters != null) {
        var slaveParam = local.parameters.getChild("slaveAddress");
        if (slaveParam != null) slaveParam.set(opSlave);
    }
    // 不再修改 module.json, 只更新 UI 参数 (已有, 见 760 行)
    script.log("Slave address updated to " + opSlave);
    util.showMessageBox(
        "Slave Address Updated",
        "Servo slave address updated.\n" +
        "  Old: " + (opSlave > 1 ? "see Parameters" : "1") + "\n" +
        "  New: " + opSlave + "\n" +
        "\n" +
        "The module's Slave Address parameter has been synced.\n" +
        "module.json default has also been updated.\n" +
        "\n" +
        "All future Modbus commands will use the new slave " + opSlave + ".\n" +
        "If you need to talk to slave 1 again, change Slave Address in the Parameters panel.\n" +
        "\n" +
        "---\n" +
        "\n" +
        "伺服从站地址已更新。\n" +
        "  新: " + opSlave + "\n" +
        "\n" +
        "模块的 Slave Address 参数已同步。\n" +
        "module.json 默认值也已更新。\n" +
        "\n" +
        "后续所有 Modbus 命令将使用新从站 " + opSlave + "。\n" +
        "如需重新与从站 1 通信, 请在 Parameters 面板中修改 Slave Address。",
        "info",
        "OK"
    );
}

// 命令：修改伺服通信参数（波特率 + 协议模式 + 软复位 + 验证）
// 流程：写 FA-72 baud → 写 FA-73 mode → 写 FA-60=1 软复位 → 等待重启 → 切模块串口 → 读回验证
// 重要前提：调用前模块串口必须与伺服当前状态一致(否则第一步就超时)
// 若不确定,请先运行 Probe Communication
// 注意: Chataigne Serial Module 在模块创建时读 module.json defaults 块(DataBits/Parity/StopBits),
//       运行时不能改. 要切换模式, 需改 module.json defaults + Reload Modules.
function setCommunication(slave, baud, mode) {
    if (waiting || probing || opActive || resetPending) return;
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
    // Step 1: 写 FA-72 baud (保存地址,直接写入 EEPROM)
    var baudDiv = Math.floor(baudNum / 100);
    sendFrame(makeWrite(slave, REG_FA72, baudDiv));
}

// 命令: Get Communication
// 无参数, 固定从站 1
function probeCommunication() {
    getCommunication();
}

// 命令：设置从站地址
// 注意: 成功修改后,模块将使用新站号与伺服通信
// 失败最常见原因: currentSlave 与伺服实际站号不一致(可能之前已改过但 UI 未更新)
// 此时先用 Probe Communication 找到伺服当前站号
function setSlaveAddress(currentSlave, newSlave) {
    if (waiting || probing || opActive || resetPending) return;
    if (currentSlave < 1 || currentSlave > 254) return;
    if (newSlave < 1 || newSlave > 254) return;
    if (currentSlave == newSlave) {
        script.log("Set slave: new == current (" + currentSlave + "), nothing to do");
        return;
    }

    opActive = true;
    opType = "set_slave";
    opStage = 1;
    opSlave = newSlave;
    opBaudVal = currentSlave;  // 用 opBaudVal 暂存 currentSlave 给错误诊断用

    script.log("Setting slave address from " + currentSlave + " to " + newSlave);
    sendFrame(makeWrite(currentSlave, REG_FA71, newSlave));
}

// Parameters 面板参数变化回调
function moduleParameterChanged(param) {
    // Get Communication Trigger: 固定从站 1, 不依赖 parameters.Slave Address
    if (param.niceName == "Get Communication") {
        getCommunication();
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
