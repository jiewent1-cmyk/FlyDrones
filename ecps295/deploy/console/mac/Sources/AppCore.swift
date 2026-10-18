// ECPS Console - glue: settings, the two links, safety, recording, and the bridge to the web UI.
//
// Web UI <- binary [1][ECL type][payload] for every companion message, text JSON {"k": "state" | "ack" | "log" | ...}
// Web UI -> text JSON {"c": "connect" | "cmd" | "stick" | "estop" | "land" | "estop_reset" | "estop_stop" | "cam" |
//                       "topo" | "rec" | "streams" | "lang" | "settings", ...}

import Foundation

final class AppCore {
    let net = DispatchQueue(label: "ecps.net")
    let fc: FCLink
    let ecl: ECLLink
    let safety: Safety
    let rec = Recorder()
    let web: WebServer
    private var timer: DispatchSourceTimer?
    private var k = 0
    private var nextTick = 0.0
    private var lag: [Double] = []  // how late the 20 Hz timer fired (ms): the network queue's backlog
    private var logTail: [String] = []
    private let defaults = UserDefaults.standard
    var onLang: (() -> Void)?  // the UI language changed (called on main): menus and window title

    var host: String { defaults.string(forKey: "host") ?? "192.168.86.20" }
    var eclPort: UInt16 { UInt16(defaults.integer(forKey: "eclPort") == 0 ? 14700 : defaults.integer(forKey: "eclPort")) }
    var mavPort: UInt16 { UInt16(defaults.integer(forKey: "mavPort") == 0 ? 14550 : defaults.integer(forKey: "mavPort")) }

    init(webRoot: URL) {
        fc = FCLink(queue: net)
        ecl = ECLLink(queue: net)
        safety = Safety(fc: fc, ecl: ecl)
        web = WebServer(root: webRoot)
        if defaults.object(forKey: "extraStreams") != nil { fc.extraStreams = defaults.bool(forKey: "extraStreams") }
        if defaults.object(forKey: "recAuto") != nil { rec.auto = defaults.bool(forKey: "recAuto") }
        if let cam = defaults.string(forKey: "cam") { ecl.prefs["cam"] = cam }
        if defaults.double(forKey: "camHz") > 0 { ecl.prefs["cam_hz"] = defaults.double(forKey: "camHz") }
        Log.sink = { [weak self] line in self?.net.async { self?.pushLog(line) } }
        safety.onEvent = { [weak self] level, text in
            Log.info("[safety] \(text)")
            self?.rec.event(level, text)
            self?.web.broadcast(text: jsonString(["k": "event", "level": level, "src": L("控制台", "Console"), "text": text]))
        }
        fc.onFrame = { [weak self] f in self?.rec.frame(f.raw) }
        fc.onAck = { [weak self] cmd, res in self?.safety.ack(command: cmd, result: res) }
        if defaults.object(forKey: "bwLevel") != nil, defaults.integer(forKey: "bwLevel") >= 0 { ecl.userLevel = defaults.integer(forKey: "bwLevel") }
        fc.onText = { [weak self] t in self?.rec.event("fc", "\(t[1]): \(t[3])") }
        ecl.onMessage = { [weak self] typ, body in self?.fromCompanion(typ, body) }
        web.onCommand = { [weak self] obj in self?.net.async { self?.command(obj) } }
        web.onClient = { [weak self] in self?.net.async { self?.welcome() } }
    }

    func start() throws {
        try web.start(base: 8770)
        Log.info("UI \(web.pageURL.absoluteString)")
        net.async {
            self.fc.connect(host: self.host, port: self.mavPort)
            self.ecl.connect(host: self.host, port: self.eclPort)
        }
        let t = DispatchSource.makeTimerSource(queue: net)
        t.schedule(deadline: .now() + 0.05, repeating: 0.05)
        t.setEventHandler { [weak self] in self?.tick() }
        t.resume()
        timer = t
    }

    // ------------------------------------------------------------------ periodic (20 Hz, on net)
    private func tick() {
        let now = mono()
        if nextTick > 0 { lag.append(max(0, (now - nextTick) * 1000)); if lag.count > 100 { lag.removeFirst(lag.count - 100) } }
        nextTick = now + 0.05
        k += 1
        ecl.tick()
        if k % 2 == 0 {
            fc.tick()
            safety.tick()
            rec.autoUpdate(armed: fc.armed, fcFresh: fc.hbAge < 2)
            web.broadcast(text: jsonString(state()))
        }
    }

    private func state() -> [String: Any] {
        [
            "k": "state",
            "fc": fc.snapshot(),
            "ecl": ecl.snapshot(),
            "safety": safety.snapshot(),
            "rec": rec.snapshot(),
            "app": ["lag_ms": jnum(lag.max(), 1)],
            "settings": ["host": host, "ecl": Int(eclPort), "mav": Int(mavPort), "streams": fc.extraStreams,
                         "cam": ecl.prefs["cam"] ?? "brain", "cam_hz": ecl.prefs["cam_hz"] ?? 5, "rec_auto": rec.auto,
                         "bw": ecl.userLevel ?? -1, "lang": Lang.current],
        ]
    }

    private func welcome() {
        web.broadcast(text: jsonString(["k": "log_tail", "lines": logTail]))
        if let t = ecl.topo {
            var d = Data([1, ECLLink.T_TOPO])
            d.append(t)
            web.broadcast(binary: d)
        }
        web.broadcast(text: jsonString(state()))
    }

    private func pushLog(_ line: String) {
        logTail.append(line)
        if logTail.count > 200 { logTail.removeFirst(logTail.count - 200) }
        web.broadcast(text: jsonString(["k": "log", "line": line]))
    }

    private func fromCompanion(_ typ: UInt8, _ body: Data) {
        var d = Data([1, typ])
        d.append(body)
        let droppable = typ == ECLLink.T_CAM || typ == ECLLink.T_MAP || typ == ECLLink.T_BRAIN || typ == ECLLink.T_TOF
        web.broadcast(binary: d, droppable: droppable)
        if typ == ECLLink.T_TICK { rec.ecpsMessage("tick", body) }
        if typ == ECLLink.T_STATUS { rec.ecpsMessage("status", body) }
    }

    // ------------------------------------------------------------------ UI commands (on net)
    private func reply(_ obj: [String: Any], _ ok: Bool, _ msg: String) {
        web.broadcast(text: jsonString(["k": "ack", "req": obj["req"] ?? NSNull(), "c": obj["c"] ?? "", "op": obj["op"] ?? "",
                                        "ok": ok, "msg": msg]))
    }

    private func command(_ obj: [String: Any]) {
        let c = (obj["c"] as? String) ?? ""
        switch c {
        case "connect":
            if let h = obj["host"] as? String, !h.trimmingCharacters(in: .whitespaces).isEmpty {
                defaults.set(h.trimmingCharacters(in: .whitespaces), forKey: "host")
            }
            if let p = obj["ecl"] as? Int, p > 0, p < 65536 { defaults.set(p, forKey: "eclPort") }
            if let p = obj["mav"] as? Int, p > 0, p < 65536 { defaults.set(p, forKey: "mavPort") }
            fc.connect(host: host, port: mavPort)
            ecl.connect(host: host, port: eclPort)
            rec.event("console", L("连接 \(host) ECPS \(eclPort) / MAVLink \(mavPort)", "connect \(host) ECPS \(eclPort) / MAVLink \(mavPort)"))
            reply(obj, true, "")
        case "cmd":
            guard let op = obj["op"] as? String else { return reply(obj, false, "no op") }
            var args = obj
            for key in ["c", "op", "req"] { args[key] = nil }
            rec.event("operator", L("指令 \(op) \(jsonString(args))", "command \(op) \(jsonString(args))"))
            ecl.sendCmd(op, args) { [weak self] ok, msg in
                self?.reply(obj, ok, msg)
                self?.rec.event(ok ? "info" : "warning", L("指令 \(op)：\(ok ? "已确认" : "失败") \(msg)", "command \(op): \(ok ? "acknowledged" : "failed") \(msg)"))
            }
        case "stick":
            ecl.sendStick((obj["f"] as? Double) ?? 0, (obj["l"] as? Double) ?? 0, (obj["y"] as? Double) ?? 0, (obj["z"] as? Double) ?? 0)
        case "estop":
            if ((obj["level"] as? Int) ?? 1) >= 2 { safety.kill(source: L("界面", "UI")) } else { safety.emergencyLand(source: L("界面", "UI")) }
            reply(obj, true, "")
        case "land":
            safety.land(source: L("界面", "UI"))
            reply(obj, true, "")
        case "estop_reset":
            safety.reset(force: (obj["force"] as? Bool) ?? false)
            reply(obj, true, "")
        case "estop_stop":
            safety.stopSending()
            reply(obj, true, "")
        case "cam":
            if let m = obj["mode"] as? String { ecl.prefs["cam"] = m; defaults.set(m, forKey: "cam") }
            if let hz = obj["hz"] as? Double { ecl.prefs["cam_hz"] = hz; defaults.set(hz, forKey: "camHz") }
            if let w = obj["color_w"] as? Int, w >= 160 { ecl.prefs["color_w"] = w }
            ecl.hello()
            reply(obj, true, "")
        case "topo":
            ecl.requestTopo()
        case "rec":
            if let a = obj["auto"] as? Bool { rec.auto = a; defaults.set(a, forKey: "recAuto") }
            if let on = obj["on"] as? Bool {
                if on {
                    let msg = rec.start(reason: L("手动", "manual"))
                    reply(obj, msg.isEmpty, msg)
                } else {
                    rec.event("record", L("停止记录（手动）", "recording stopped (manual)"))
                    rec.stop()
                    reply(obj, true, "")
                }
            }
        case "bw":  // bandwidth level: -1 automatic, 0-3 fixed
            let l = (obj["lvl"] as? Int) ?? -1
            ecl.userLevel = (0...3).contains(l) ? l : nil
            defaults.set(l, forKey: "bwLevel")
            reply(obj, true, "")
        case "streams":
            fc.extraStreams = (obj["on"] as? Bool) ?? true
            defaults.set(fc.extraStreams, forKey: "extraStreams")
            reply(obj, true, "")
        case "lang":
            setLang((obj["lang"] as? String) ?? "zh")
            reply(obj, true, "")
        case "ui_event":  // operator actions worth keeping in the recording (e.g. goal picked, view changes)
            rec.event("ui", (obj["text"] as? String) ?? "")
        default:
            reply(obj, false, "unknown command \(c)")
        }
    }

    // ------------------------------------------------------------------ native entry points (menu / keys)
    func emergencyLand(source: String) { net.async { self.safety.emergencyLand(source: source) } }
    func kill(source: String) { net.async { self.safety.kill(source: source) } }
    func land(source: String) { net.async { self.safety.land(source: source) } }
    func resetEstop(force: Bool) { net.async { self.safety.reset(force: force) } }

    /// UI language (menu or page): saved; the menus follow at once, the pages with the next state (sent right away).
    func setLang(_ l: String) {
        Lang.set(l)
        DispatchQueue.main.async { self.onLang?() }
        net.async { self.web.broadcast(text: jsonString(self.state())) }
    }

    /// For the quit confirmation: the FC reports armed (and is heard).
    var flying: Bool { net.sync { fc.armed && fc.hbAge < 3 } }
}
