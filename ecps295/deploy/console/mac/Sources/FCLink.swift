// ECPS Console - MAVLink link to the flight controller through mavlink-router's ground-station endpoint on the Pi
// (UDP <pi>:14550, Mode=Server: the router answers whoever sends to it, so we send a 1 Hz heartbeat first).
//
// The console is a ground station with sysid 250: outside MAV_GCS_SYSID..MAV_GCS_SYSID_HI (254 on the drone, 254-255
// in SITL), so its heartbeat never keeps the FC's companion-loss failsafe (FS_GCS) quiet. It never requests the streams
// the companion depends on (RANGEFINDER is its route-B height source): it only listens to them and, optionally,
// asks for a few extra display-only messages (optical flow, vibration, HUD, battery) that the companion does not use.

import Foundation
import Network

final class FCLink {
    let q: DispatchQueue
    private var conn: NWConnection?
    private(set) var host = ""
    private(set) var port: UInt16 = 14550
    private(set) var state = "idle"
    let enc = MavEncoder(sysid: 250, compid: 190)
    let parser = MavParser()
    var fcSys: UInt8 = 1
    var companionSys: UInt8 = 254
    var extraStreams = true

    // latest state
    private(set) var msgs: [String: [String: Any]] = [:]  // FC messages by name
    private(set) var msgT: [String: Double] = [:]
    private(set) var hbT = 0.0
    private(set) var mode: UInt32?
    private(set) var armed = false
    private(set) var compHbT = 0.0
    private(set) var viso: [String: Any]?
    private(set) var visoT = 0.0
    private(set) var rxFrames = 0
    private(set) var rxBytes = 0
    private var seqs: [UInt16: (last: UInt8, lost: Int, got: Int)] = [:]
    private var newTexts: [[Any]] = []
    private var lastHbSent = 0.0
    private var lastStreamReq = -100.0
    private var reconnectAt = 0.0
    private var rateWin: [Double] = []

    var onFrame: ((MavFrame) -> Void)?
    var onAck: ((UInt16, UInt8) -> Void)?
    var onText: (([Any]) -> Void)?

    init(queue: DispatchQueue) { q = queue }

    func connect(host: String, port: UInt16) {
        self.host = host
        self.port = port
        conn?.cancel()
        conn = nil
        msgs = [:]; msgT = [:]; hbT = 0; mode = nil; armed = false; compHbT = 0; seqs = [:]
        guard !host.isEmpty, let p = NWEndpoint.Port(rawValue: port) else { state = "no host"; return }
        let params = NWParameters.udp
        params.serviceClass = .interactiveVoice  // WMM AC_VO: the LAND / disarm commands jump the AP's queues
        let c = NWConnection(host: NWEndpoint.Host(host), port: p, using: params)
        conn = c
        c.stateUpdateHandler = { [weak self] st in
            guard let self = self else { return }
            switch st {
            case .ready: self.state = "ready"; self.send(self.enc.heartbeat())
            case .waiting(let e): self.state = "waiting: \(e.localizedDescription)"
            case .failed(let e): self.state = "failed: \(e.localizedDescription)"; self.reconnectAt = mono() + 1.0
            case .cancelled: break
            default: self.state = "\(st)"
            }
        }
        c.start(queue: q)
        receive(c)
        Log.info("MAVLink -> \(host):\(port) (sysid \(enc.sysid))")
    }

    private func receive(_ c: NWConnection) {
        c.receiveMessage { [weak self] data, _, _, err in
            guard let self = self, c === self.conn else { return }
            if let d = data, !d.isEmpty { self.handle(d) }
            if err == nil { self.receive(c) }
        }
    }

    func send(_ d: Data) {
        guard let c = conn, !d.isEmpty else { return }
        c.send(content: d, completion: .contentProcessed { _ in })
    }

    /// 10 Hz from AppCore (on q).
    func tick() {
        let now = mono()
        if reconnectAt > 0, now >= reconnectAt {
            reconnectAt = 0
            connect(host: host, port: port)
        }
        if now - lastHbSent >= 1.0 {
            lastHbSent = now
            send(enc.heartbeat())
        }
        if extraStreams, now - hbT < 3, now - lastStreamReq > 10 {
            lastStreamReq = now
            // display-only messages the companion never requests: OPTICAL_FLOW, VIBRATION, VFR_HUD, BATTERY_STATUS
            for (mid, hz) in [(100, 5.0), (241, 2.0), (74, 4.0), (147, 1.0)] as [(Int, Double)] {
                send(enc.commandLong(target: fcSys, comp: 1, command: 511, [Float(mid), Float(1e6 / hz)]))
            }
        }
        rateWin = rateWin.filter { now - $0 < 2.0 }
    }

    private func handle(_ d: Data) {
        rxBytes += d.count
        let now = mono()
        for f in parser.push(d) {
            rxFrames += 1
            rateWin.append(now)
            onFrame?(f)
            let key = UInt16(f.sysid) << 8 | UInt16(f.compid)
            if var s = seqs[key] {
                let gap = Int(f.seq &- s.last &- 1)
                s.lost += gap < 128 ? gap : 0
                s.got += 1
                s.last = f.seq
                seqs[key] = s
            } else {
                seqs[key] = (f.seq, 0, 1)
            }
            guard f.known, let spec = mavMessages[f.msgid] else { continue }
            let m = mavDecode(f)
            if f.sysid == fcSys {
                if spec.name == "HEARTBEAT" {
                    let ap = (m["autopilot"] as? Double) ?? 0
                    let ty = (m["type"] as? Double) ?? 0
                    if ap == 8 || ty == 6 || ty == 18 { continue }  // not the vehicle (GCS / onboard controller)
                    hbT = now
                    mode = UInt32((m["custom_mode"] as? Double) ?? 0)
                    armed = (UInt8((m["base_mode"] as? Double) ?? 0) & 128) != 0
                }
                msgs[spec.name] = m
                msgT[spec.name] = now
                if spec.name == "COMMAND_ACK" {
                    let tgt = UInt8((m["target_system"] as? Double) ?? 0)
                    if tgt == 0 || tgt == enc.sysid {
                        onAck?(UInt16((m["command"] as? Double) ?? 0), UInt8((m["result"] as? Double) ?? 0))
                    }
                }
                if spec.name == "STATUSTEXT" { text(m, src: L("飞控", "FC")) }
            } else if f.sysid == companionSys {
                if spec.name == "HEARTBEAT" { compHbT = now }
                if spec.name == "VISION_POSITION_ESTIMATE" { viso = m; visoT = now }
                if spec.name == "STATUSTEXT" { text(m, src: L("机载", "Companion")) }
            }
        }
    }

    private func text(_ m: [String: Any], src: String) {
        let t: [Any] = [Date().timeIntervalSince1970, src, Int((m["severity"] as? Double) ?? 6), (m["text"] as? String) ?? ""]
        newTexts.append(t)
        onText?(t)
    }

    // ------------------------------------------------------------------ commands
    func setMode(_ m: UInt32) {
        // MAV_CMD_DO_SET_MODE (176) with an ACK, and the plain SET_MODE message as a second path
        send(enc.commandLong(target: fcSys, comp: 1, command: 176, [1, Float(m)]))
        send(enc.setMode(target: fcSys, customMode: m))
    }

    /// MAV_CMD_COMPONENT_ARM_DISARM 0 with the force magic 21196: disarms in flight (motors stop at once).
    func forceDisarm() {
        send(enc.commandLong(target: fcSys, comp: 1, command: 400, [0, 21196]))
    }

    var hbAge: Double { hbT > 0 ? mono() - hbT : .infinity }
    var modeName: String { mode.map { copterModes[$0] ?? "MODE \($0)" } ?? "?" }

    func age(_ name: String) -> Double? { msgT[name].map { mono() - $0 } }

    private func num(_ name: String, _ field: String) -> Double? { msgs[name]?[field] as? Double }

    func snapshot() -> [String: Any] {
        let now = mono()
        var lost = 0, got = 0
        for (k, v) in seqs where UInt8(k >> 8) == fcSys { lost += v.lost; got += v.got }
        var s: [String: Any] = [
            "link": ["host": host, "port": Int(port), "state": state, "fps": rateWin.count / 2,
                     "loss_pct": jnum(got + lost > 0 ? 100.0 * Double(lost) / Double(got + lost) : nil, 1),
                     "crc_err": parser.crcErrors, "kB": rxBytes / 1024],
            "hb_age": jnum(hbT > 0 ? now - hbT : nil, 2),
            "mode": modeName,
            "mode_n": mode.map { Int($0) } ?? NSNull(),
            "armed": armed,
            "comp_hb_age": jnum(compHbT > 0 ? now - compHbT : nil, 2),
        ]
        if let a = msgs["ATTITUDE"] {
            s["att"] = [jnum(((a["roll"] as? Double) ?? 0) * 180 / .pi, 1), jnum(((a["pitch"] as? Double) ?? 0) * 180 / .pi, 1),
                        jnum(((a["yaw"] as? Double) ?? 0) * 180 / .pi, 1), jnum(((a["yawspeed"] as? Double) ?? 0) * 180 / .pi, 1)]
            s["att_age"] = jnum(age("ATTITUDE"), 2)
        }
        if let p = msgs["LOCAL_POSITION_NED"] {
            s["lpn"] = ["x", "y", "z", "vx", "vy", "vz"].map { jnum(p[$0] as? Double) }
            s["lpn_age"] = jnum(age("LOCAL_POSITION_NED"), 2)
        }
        if let r = num("RANGEFINDER", "distance") { s["rng"] = jnum(r); s["rng_age"] = jnum(age("RANGEFINDER"), 2) }
        if let ds = msgs["DISTANCE_SENSOR"] { s["dist"] = jnum(((ds["current_distance"] as? Double) ?? 0) / 100) }
        if let v = viso { s["viso_z"] = jnum(-((v["z"] as? Double) ?? 0)); s["viso_age"] = jnum(now - visoT, 2) }
        if let st = msgs["SYS_STATUS"] {
            let mv = (st["voltage_battery"] as? Double) ?? 0
            let ca = (st["current_battery"] as? Double) ?? -1
            s["bat"] = [jnum(mv / 1000, 2), jnum(ca >= 0 ? ca / 100 : nil, 2), (st["battery_remaining"] as? Double).map { Int($0) } ?? NSNull()]
            let present: Double = (st["onboard_control_sensors_present"] as? Double) ?? 0
            let enabled: Double = (st["onboard_control_sensors_enabled"] as? Double) ?? 0
            let health: Double = (st["onboard_control_sensors_health"] as? Double) ?? 0
            s["sensors"] = [present, enabled, health]
            s["load"] = jnum(((st["load"] as? Double) ?? 0) / 10, 1)
        }
        if let b = msgs["BATTERY_STATUS"] {
            s["bat2"] = [jnum(((b["current_consumed"] as? Double) ?? 0), 0), jnum(((b["temperature"] as? Double) ?? 32767) / 100, 1)]
        }
        if let e = msgs["EKF_STATUS_REPORT"] {
            s["ekf"] = [Int((e["flags"] as? Double) ?? 0), jnum(e["velocity_variance"] as? Double, 2), jnum(e["pos_horiz_variance"] as? Double, 2),
                        jnum(e["pos_vert_variance"] as? Double, 2), jnum(e["compass_variance"] as? Double, 2), jnum(e["terrain_alt_variance"] as? Double, 2)]
            s["ekf_age"] = jnum(age("EKF_STATUS_REPORT"), 2)
        }
        if let f = msgs["OPTICAL_FLOW"] {
            s["flow"] = [Int((f["quality"] as? Double) ?? 0), jnum(f["flow_comp_m_x"] as? Double, 3), jnum(f["flow_comp_m_y"] as? Double, 3),
                         jnum(f["ground_distance"] as? Double, 3)]
            s["flow_age"] = jnum(age("OPTICAL_FLOW"), 2)
        }
        if let v = msgs["VIBRATION"] {
            s["vib"] = ["vibration_x", "vibration_y", "vibration_z"].map { jnum(v[$0] as? Double, 1) }
                + ["clipping_0", "clipping_1", "clipping_2"].map { Int((v[$0] as? Double) ?? 0) }
        }
        if let h = msgs["VFR_HUD"] {
            s["hud"] = ["groundspeed", "alt", "climb", "throttle", "heading"].map { jnum(h[$0] as? Double, 2) }
        }
        if let rc = msgs["RC_CHANNELS"] {
            s["rc"] = (1...8).map { Int((rc["chan\($0)_raw"] as? Double) ?? 0) } + [Int((rc["rssi"] as? Double) ?? 0)]
            s["rc_age"] = jnum(age("RC_CHANNELS"), 2)
        }
        if let sp = msgs["SCALED_PRESSURE"] { s["press"] = jnum(sp["press_abs"] as? Double, 2) }
        if let sim = msgs["SIM_STATE"] {
            s["sim"] = [jnum(((sim["lat_int"] as? Double) ?? 0) / 1e7, 7), jnum(((sim["lon_int"] as? Double) ?? 0) / 1e7, 7),
                        jnum(sim["alt"] as? Double, 2), jnum(sim["yaw"] as? Double, 3)]
        }
        if !newTexts.isEmpty {
            s["texts"] = newTexts
            newTexts.removeAll()
        }
        return s
    }
}
