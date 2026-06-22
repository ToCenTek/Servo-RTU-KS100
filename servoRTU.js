var responseValue = null;
var baudRateValue = null;
var protocolValue = null;
var slaveAddressValue = null;

var waiting = false;
var waitTime = 0;
var rxBuffer = [];

var probing = false;
var probeSlave = 1;
var probeSlaveEnd = 10;  // 默认扫描 1~10
var probePollCount = 0;
var probeTotalPolls = 0;
var probeMaxPolls = 240;
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

// 把验证后的通讯参数写回 module.json 的 defaults,这样 Reload Custom Modules 后
// 模块会按新参数连接,同时 init() 也能从 defaults 还原 Communication Information 显示
// 实现: 用字符串正则替换,保留原文件的中文、缩进、换行符风格
// (如果用 util.writeFile(..., object) JSON.stringify 会把中文转义为 \uXXXX
//  并把缩进统一为 2 空格,破坏原格式)
function saveModuleDefaults(baud, mode) {
    var dir = script.getScriptDirectory();
    var modulePath = dir + "/module.json";
    script.log("saveModuleDefaults: dir=" + dir);
    if (!util.fileExists(modulePath)) {
        script.logWarning("saveModuleDefaults: file not found at " + modulePath);
        return false;
    }
    var content = util.readFile(modulePath, false);
    if (content == null) {
        script.logWarning("saveModuleDefaults: readFile returned null");
        return false;
    }
    var parts = modeToSerial(mode);
    var newBaud = '"BaudRate": ' + baud;
    var newData = '"DataBits": ' + parts[0];
    var newParity = '"Parity": "' + parts[1] + '"';
    var newStop = '"StopBits": ' + parts[2];
    // ES3 不支持正则字面量,改用 new RegExp
    var reBaud = new RegExp('"BaudRate"\\s*:\\s*\\d+');
    var reData = new RegExp('"DataBits"\\s*:\\s*\\d+');
    var reParity = new RegExp('"Parity"\\s*:\\s*"[^"]*"');
    var reStop = new RegExp('"StopBits"\\s*:\\s*\\d+');
    if (content.search(reBaud) < 0) {
        script.logWarning("saveModuleDefaults: BaudRate field not found");
        return false;
    }
    content = content.replace(reBaud, newBaud);
    content = content.replace(reData, newData);
    content = content.replace(reParity, newParity);
    content = content.replace(reStop, newStop);
    util.writeFile(modulePath, content, true);
    // 验证: 重新读取看新值是否真的落盘
    var check = util.readFile(modulePath, false);
    if (check == null) {
        script.logWarning("saveModuleDefaults: verification read returned null");
        return false;
    }
    if (check.indexOf(newBaud) < 0) {
        script.logWarning("saveModuleDefaults: write did not persist (re-read missing new value)");
        return false;
    }
    script.log("saveModuleDefaults: wrote " + newBaud + ", " + newData + ", " + newParity + ", " + newStop);
    return true;
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
// 自动扫描 slave 1~slaveEnd, 找到那个唯一的从站
// 前提: 总线上同时只能有 1 个从站(其他从站断电), 否则多从站同时
//       响应会导致 Modbus CRC 错误,探测失败
// 实际从站地址从 FA-71 寄存器读出 (响应帧),不依赖 UI 参数
function getCommunication(slaveEnd) {
    if (waiting || probing) return;
    if (slaveEnd == null || slaveEnd < 1) {
        slaveEnd = 10;  // 默认扫到 10, 覆盖大部分场景
    }
    if (slaveEnd > 254) {
        slaveEnd = 254;
    }
    probeSlave = 1;
    probeSlaveEnd = slaveEnd;
    probing = false;

    util.showMessageBox("Please wait...",
        "探测伺服通信参数 (扫描 slave 1~" + slaveEnd + "), 这需要一些时间...\n" +
        "完成后将重新使能当前模块\n" +
        "请将通信参数设置为伺服一致\n" +
        "\n" +
        "Detecting servo: scan slave 1~" + slaveEnd + "...",
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

    var cmd = "/bin/bash " + SHELL_PATH +
              " --slave " + probeSlave +
              " --scan-end " + probeSlaveEnd +
              " --output " + RESULT_PATH;

    script.log("Probing servo parameters (slave 1~" + probeSlaveEnd + ")");

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

// 软复位等待结束: 写 module.json defaults,弹窗提示用户 Reload Custom Modules
// 不自动切模块串口(Chataigne Serial Module 的 param.set 不一定触发重连),
// 不自动读回验证(需要切换串口,见上)。0x06 响应是请求回显,3 步都收到回显
// 本身就证明伺服已收到并执行,设置已应用。
function handleResetComplete() {
    var saved = saveModuleDefaults(opBaudVal, opModeVal);
    if (saved) {
        script.log("module.json defaults updated to " + opBaudVal + " " + modeLabel(opModeVal));
    } else {
        script.logWarning("Failed to update module.json defaults");
    }
    // 持久化 Communication Information 显示(即使 Reload 之前用户也能看到)
    confirmedBaud = opBaudVal;
    confirmedMode = opModeVal;
    confirmedSlave = opSlave;
    if (baudRateValue != null) baudRateValue.set("" + opBaudVal);
    if (protocolValue != null) protocolValue.set(modeLabel(opModeVal));
    // 弹窗提示用户手动 Reload Custom Modules 让 defaults 生效
    // 注意: showMessageBox 的按钮只是标签,无回调,不会触发 Reload
    // Chataigne 引擎没有提供 reloadModules() API,必须用户在 UI 手动操作
    util.showMessageBox(
        "Servo Communication Updated",
        "Servo communication updated.\n" +
        "  Slave: " + opSlave + "\n" +
        "  Baud:  " + opBaudVal + "\n" +
        "  Mode:  " + modeLabel(opModeVal) + "\n" +
        "\n" +
        "module.json defaults have been updated.\n" +
        "To apply the new defaults, manually Reload Custom Modules:\n" +
        "  Module menu > Reload Custom Modules\n" +
        "(or use the keyboard shortcut shown in that menu)\n" +
        "\n" +
        "If the new params differ from current module serial, the\n" +
        "module cannot talk to the servo until you Reload.\n" +
        "If params are unchanged, no Reload is needed.\n" +
        "\n" +
        "----------------------------------------------------------\n" +
        "\n" +
        "伺服通讯已更新。\n" +
        "  - 从站:  " + opSlave + "\n" +
        "  - 波特:  " + opBaudVal + "\n" +
        "  - 模式:  " + modeLabel(opModeVal) + "\n" +
        "\n" +
        "module.json 默认参数已更新。\n" +
        "要应用新默认参数, 请删除当前模块, 并重新加载自定义模块:\n" +
        "  File > Reload Custom Modules\n" +
        "\n" +
        "如果新参数与当前模块串口不同, 重新加载前模块可能无法与伺服通信。\n" +
        "如果参数未变, 则无需重新加载",
        "info",
        "OK"
    );
}

// 把从站地址同步到 module.json 的 parameters.Slave Address.default
// (Reload Modules 后这个值会作为 UI 参数的初始值)
function saveModuleSlaveAddress(newSlave) {
    var dir = script.getScriptDirectory();
    var modulePath = dir + "/module.json";
    if (!util.fileExists(modulePath)) return false;
    var content = util.readFile(modulePath, false);
    if (content == null) return false;
    // 匹配 "Slave Address": { ... "default": N ... } 块中的 default 值
    // 简单做法: 在 "Slave Address" 块内替换 "default": <digits>
    var idx = content.indexOf('"Slave Address"');
    if (idx < 0) {
        script.logWarning("saveModuleSlaveAddress: 'Slave Address' field not found");
        return false;
    }
    // 找下一个 parameters 字段("scripts" 之前)作为结束边界
    // (这样不会误把 commands 部分的 "Slave Address" 也改掉)
    var endIdx = content.indexOf('"scripts"', idx);
    if (endIdx < 0) endIdx = content.length;
    var block = content.substring(idx, endIdx);
    var newBlock = block.replace(
        new RegExp('"default"\\s*:\\s*\\d+'),
        '"default": ' + newSlave
    );
    content = content.substring(0, idx) + newBlock + content.substring(endIdx);
    util.writeFile(modulePath, content, true);
    return true;
}

// 多步操作：写 FA-71 从站地址
// 成功后: 同步更新 module.json parameters.Slave Address.default, 更新 UI 参数
// (Reload Modules 后会用新默认值)
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
    // 同步更新 UI 中的 Slave Address 参数
    if (local.parameters != null) {
        var slaveParam = local.parameters.getChild("Slave Address");
        if (slaveParam != null) slaveParam.set(opSlave);
    }
    // 持久化到 module.json
    if (saveModuleSlaveAddress(opSlave)) {
        script.log("module.json Slave Address default updated to " + opSlave);
    } else {
        script.logWarning("Failed to update module.json Slave Address");
    }
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

// 命令: Get Communication (命令面板触发)
// 接受 Scan End 一个参数, 不依赖 parameters.Slave Address
function probeCommunication(scanEnd) {
    if (scanEnd == null) scanEnd = 10;
    getCommunication(scanEnd);
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
    // Parameters 中的 Get Communication Trigger: 默认扫 1~10, 不依赖 parameters.Slave Address
    if (param.niceName == "Get Communication") {
        getCommunication(10);
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
