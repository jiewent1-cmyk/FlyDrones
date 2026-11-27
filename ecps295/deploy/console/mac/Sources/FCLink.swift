// ECPS Console - MAVLink link to the flight controller through mavlink-router's ground-station endpoint on the Pi
// Two ways in, both always on:
//   - client: UDP to <pi>:14550 for a router endpoint in Mode=Server (SITL / Gazebo rigs): the router answers whoever
//     sends to it, so we send a 1 Hz heartbeat first;
//   - listener: UDP on local port 14550 for a router endpoint in Mode=Normal pointed at this Mac (the Pi's production
//     config since the 10-31 security change: Address = ECPS_GCS_IP, Port = 14550, no server port open on the Pi).
//     Only flows from the configured host are accepted; replies go back on that flow (the router's own source port).
// Sends go to whichever path delivered FC frames most recently (the client path until then).
//
// The console is a ground station with sysid 250: outside MAV_GCS_SYSID..MAV_GCS_SYSID_HI (254 on the drone, 254-255
// in SITL), so its heartbeat never keeps the FC's companion-loss failsafe (FS_GCS) quiet. It never requests the streams
// the companion depends on (RANGEFINDER is its route-B height source): it only listens to them and, optionally,
// asks for a few extra display-only messages (optical flow, vibration, HUD, battery) that the companion does not use.

import Foundation
import Network

final class FCLink {
    let q: DispatchQueue
    private var conn: NWConnection?         // client path (router Mode=Server)
    private var listener: NWListener?       // listener path (router Mode=Normal -> this Mac)
    private var inConn: NWConnection?       // the accepted flow from the router
    private var inRxT = 0.0                 // last frame received on inConn
    private var listenPort: UInt16 = 0
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
    private var lastGpsReq = -100.0
    private var lastMsgReq = -100.0
    private var reconnectAt = 0.0
    private var rateWin: [Double] = []

    // the mission, read only (2026-11-25, GPS map page): the console asks the FC for its mission with the MAVLink
    // mission protocol and draws it. It never uploads a mission, never changes the mode or anything else.
    private var missionItems: [Int: [String: Any]] = [:]
    private var missionCount = 0
    private var missionCurrent = -1
    private var missionState = 0     // 0 idle, 1 waiting for the count, 2 getting the items, 3 done
    private var missionT = 0.0        // when the last request went out
    private var missionTries = 0
    private var missionNext = 0       // the item we ask for next
    private var missionItemTries = 0
    private var missionT0 = 0.0       // when the mission arrived
    private var missionAsked = false  // ask once after connecting
    private var lastFix = -1          // the last GPS fix type we logged
    private var homeSeen = false
    private var originSeen = false

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
        missionItems = [:]; missionCount = 0; missionCurrent = -1; missionState = 0; missionAsked = false
        lastFix = -1; homeSeen = false; originSeen = false
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
        listen(port: port)
    }

    /// Listener on local UDP `port` for a router that pushes to this Mac (Mode=Normal). Restarted only on a port change.
    private func listen(port: UInt16) {
        inConn?.cancel(); inConn = nil; inRxT = 0
        if listener != nil, listenPort == port { return }
        listener?.cancel(); listener = nil
        guard let p = NWEndpoint.Port(rawValue: port) else { return }
        let params = NWParameters.udp
        params.serviceClass = .interactiveVoice
        params.allowLocalEndpointReuse = true
        do {
            let l = try NWListener(using: params, on: p)
            l.newConnectionHandler = { [weak self] c in
                guard let self = self else { c.cancel(); return }
                guard case let .hostPort(h, _) = c.endpoint, self.sameHost(h) else {
                    Log.info("MAVLink listener: ignored flow from \(c.endpoint)")
                    c.cancel(); return
                }
                self.inConn?.cancel()
                self.inConn = c
                c.start(queue: self.q)
                self.receiveIn(c)
                Log.info("MAVLink <- \(c.endpoint) on local :\(port)")
            }
            l.stateUpdateHandler = { st in
                if case .failed(let e) = st { Log.info("MAVLink listener :\(port) failed: \(e.localizedDescription)") }
            }
            l.start(queue: q)
            listener = l
            listenPort = port
        } catch {
            Log.info("MAVLink listener :\(port) not started: \(error.localizedDescription)")
        }
    }

    private func sameHost(_ h: NWEndpoint.Host) -> Bool {
        var s = "\(h)"
        if let i = s.firstIndex(of: "%") { s = String(s[..<i]) }  // drop an interface suffix
        return s == host || s == "::ffff:\(host)"
    }

    private func receiveIn(_ c: NWConnection) {
        c.receiveMessage { [weak self] data, _, _, err in
            guard let self = self, c === self.inConn else { return }
            if let d = data, !d.isEmpty { self.inRxT = mono(); self.handle(d) }
            if err == nil { self.receiveIn(c) }
        }
    }

    private func receive(_ c: NWConnection) {
        c.receiveMessage { [weak self] data, _, _, err in
            guard let self = self, c === self.conn else { return }
            if let d = data, !d.isEmpty { self.handle(d) }
            if err == nil { self.receive(c) }
        }
    }

    func send(_ d: Data) {
        guard !d.isEmpty else { return }
        if let ic = inConn, mono() - inRxT < 3.0 {  // the router pushes to us: answer on its flow
            ic.send(content: d, completion: .contentProcessed { _ in })
            return
        }
        guard let c = conn else { return }
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
        // the GPS map page (2026-11-25): the GPS fix and the FC's global position. Always on (the page needs them),
        // display-only: the companion uses LOCAL_POSITION_NED, not these.
        if now - hbT < 3, now - lastGpsReq > 10 {
            lastGpsReq = now
            for (mid, hz) in [(24, 5.0), (33, 5.0)] as [(Int, Double)] {
                send(enc.commandLong(target: fcSys, comp: 1, command: 511, [Float(mid), Float(1e6 / hz)]))
            }
        }
        // the messages the FC does not stream by itself: the home point and the EKF origin (both on request)
        if now - hbT < 3, now - lastMsgReq > 5 {
            lastMsgReq = now
            for mid in [242, 49] { send(enc.commandLong(target: fcSys, comp: 1, command: 512, [Float(mid)])) }
        }
        missionTick(now)
        rateWin = rateWin.filter { now - $0 < 2.0 }
    }

    // ------------------------------------------------------------------ mission (read only)
    /// Ask the FC for its mission (MAVLink mission protocol). Called on demand and once after connecting.
    func requestMission() {
        missionItems = [:]; missionCount = 0; missionState = 1; missionTries = 1; missionNext = 0
        missionT = mono()
        send(enc.missionRequestList(target: fcSys, comp: 1))
        Log.info("mission: list requested")
    }

    private func requestItem(_ seq: Int) {
        missionNext = seq
        missionItemTries = 1
        missionT = mono()
        send(enc.missionRequestInt(target: fcSys, comp: 1, seq: UInt16(seq)))
    }

    private func finishMission() {
        send(enc.missionAck(target: fcSys, comp: 1))  // to the FC: closes its mission-download transaction
        missionState = 3
        missionT0 = mono()
        Log.info("mission: \(missionItems.count)/\(missionCount) items received")
    }

    private func missionTick(_ now: Double) {
        if missionState == 1, now - missionT > 2.0 {
            if missionTries < 4 {
                missionTries += 1; missionT = now
                send(enc.missionRequestList(target: fcSys, comp: 1))
            } else {
                missionState = 0
                Log.info("mission: the FC did not answer the list request")
            }
        } else if missionState == 2, now - missionT > 1.5 {
            if missionItemTries < 3 {
                missionItemTries += 1; missionT = now
                // retry with MISSION_REQUEST_INT; the MAVLink 1 form (some FCs answer only this one) only on the last
                // try: ArduPilot answers it but logs "got MISSION_REQUEST; use MISSION_REQUEST_INT!" (2026-11-26 SITL)
                if missionItemTries < 3 {
                    send(enc.missionRequestInt(target: fcSys, comp: 1, seq: UInt16(missionNext)))
                } else {
                    send(enc.missionRequest(target: fcSys, comp: 1, seq: UInt16(missionNext)))
                }
            } else {
                missionNext += 1; missionItemTries = 0; missionT = now   // this item does not come: skip it
                if missionNext >= missionCount { finishMission() }
            }
        }
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
                    if !missionAsked { missionAsked = true; requestMission() }   // once after connecting
                }
                msgs[spec.name] = m
                msgT[spec.name] = now
                // the mission protocol: the FC's answers to our read-only request
                if spec.name == "MISSION_COUNT" {
                    let c = Int((m["count"] as? Double) ?? 0)
                    if missionState == 1 {
                        missionCount = c
                        missionState = 2
                        Log.info("mission: the FC has \(c) items")
                        if c > 0 { requestItem(0) } else { finishMission() }
                    }
                } else if spec.name == "MISSION_ITEM" {
                    let seq = Int((m["seq"] as? Double) ?? 0)
                    missionItems[seq] = m
                    if missionState == 2 {
                        missionNext = seq + 1
                        missionItemTries = 0
                        missionT = now
                        if missionNext < missionCount { requestItem(missionNext) } else { finishMission() }
                    }
                } else if spec.name == "MISSION_CURRENT" {
                    missionCurrent = Int((m["seq"] as? Double) ?? 0)
                } else if spec.name == "GPS_RAW_INT" {
                    let fix = fi("GPS_RAW_INT", "fix_type")
                    if fix != lastFix {
                        lastFix = fix
                        Log.info(String(format: "GPS: fix %d, %d sats, hdop %.2f, h_acc %.2f m, %.7f %.7f",
                                         fix, fi("GPS_RAW_INT", "satellites_visible"),
                                         u16v("GPS_RAW_INT", "eph") / 100, fv("GPS_RAW_INT", "h_acc") / 1000,
                                         fv("GPS_RAW_INT", "lat") / 1e7, fv("GPS_RAW_INT", "lon") / 1e7))
                    }
                } else if spec.name == "HOME_POSITION", !homeSeen {
                    homeSeen = true
                    Log.info(String(format: "home: %.7f %.7f", fv("HOME_POSITION", "latitude") / 1e7,
                                     fv("HOME_POSITION", "longitude") / 1e7))
                } else if spec.name == "GPS_GLOBAL_ORIGIN", !originSeen {
                    originSeen = true
                    Log.info(String(format: "EKF origin: %.7f %.7f", fv("GPS_GLOBAL_ORIGIN", "latitude") / 1e7,
                                     fv("GPS_GLOBAL_ORIGIN", "longitude") / 1e7))
                }
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

    /// A field of the last message of that name (NaN when it is missing).
    private func fv(_ name: String, _ field: String) -> Double { (msgs[name]?[field] as? Double) ?? .nan }
    /// The same as an integer (0 when missing).
    private func fi(_ name: String, _ field: String) -> Int { let v = fv(name, field); return v.isFinite ? Int(v) : 0 }
    /// A uint16 field where 65535 means "unknown" (NaN then).
    private func u16v(_ name: String, _ field: String) -> Double { let v = fv(name, field); return v >= 65535 ? .nan : v }

    func snapshot() -> [String: Any] {
        let now = mono()
        var lost = 0, got = 0
        for (k, v) in seqs where UInt8(k >> 8) == fcSys { lost += v.lost; got += v.got }
        var s: [String: Any] = [
            "link": ["host": host, "port": Int(port), "state": (inConn != nil && mono() - inRxT < 3.0) ? "ready (listen :\(listenPort))" : state, "fps": rateWin.count / 2,
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
        // ---- GPS map page (2026-11-25). Two different positions: the raw GPS fix and the FC's own (flow) position.
        if msgs["GPS_RAW_INT"] != nil {
            s["gps"] = ["fix": fi("GPS_RAW_INT", "fix_type"),
                        "lat": jnum(fv("GPS_RAW_INT", "lat") / 1e7, 7),
                        "lon": jnum(fv("GPS_RAW_INT", "lon") / 1e7, 7),
                        "alt": jnum(fv("GPS_RAW_INT", "alt") / 1000, 2),
                        "sats": fi("GPS_RAW_INT", "satellites_visible"),
                        "hdop": jnum(u16v("GPS_RAW_INT", "eph") / 100, 2),
                        "epv": jnum(u16v("GPS_RAW_INT", "epv") / 100, 2),
                        "hacc": jnum(fv("GPS_RAW_INT", "h_acc") / 1000, 2),
                        "vacc": jnum(fv("GPS_RAW_INT", "v_acc") / 1000, 2),
                        "vel": jnum(u16v("GPS_RAW_INT", "vel") / 100, 2),
                        "cog": jnum(u16v("GPS_RAW_INT", "cog") / 100, 1),
                        "age": jnum(age("GPS_RAW_INT"), 2)]
        }
        if msgs["HOME_POSITION"] != nil {
            s["home"] = ["lat": jnum(fv("HOME_POSITION", "latitude") / 1e7, 7),
                         "lon": jnum(fv("HOME_POSITION", "longitude") / 1e7, 7),
                         "alt": jnum(fv("HOME_POSITION", "altitude") / 1000, 2),
                         "age": jnum(age("HOME_POSITION"), 2)]
        }
        if msgs["GPS_GLOBAL_ORIGIN"] != nil {
            s["origin"] = ["lat": jnum(fv("GPS_GLOBAL_ORIGIN", "latitude") / 1e7, 7),
                           "lon": jnum(fv("GPS_GLOBAL_ORIGIN", "longitude") / 1e7, 7),
                           "alt": jnum(fv("GPS_GLOBAL_ORIGIN", "altitude") / 1000, 2),
                           "age": jnum(age("GPS_GLOBAL_ORIGIN"), 2)]
        }
        if msgs["GLOBAL_POSITION_INT"] != nil {
            s["gpos"] = ["lat": jnum(fv("GLOBAL_POSITION_INT", "lat") / 1e7, 7),
                         "lon": jnum(fv("GLOBAL_POSITION_INT", "lon") / 1e7, 7),
                         "alt": jnum(fv("GLOBAL_POSITION_INT", "alt") / 1000, 2),
                         "rel": jnum(fv("GLOBAL_POSITION_INT", "relative_alt") / 1000, 2),
                         "hdg": jnum(u16v("GLOBAL_POSITION_INT", "hdg") / 100, 1),
                         "age": jnum(age("GLOBAL_POSITION_INT"), 2)]
        }
        // the FC's own position: the EKF origin (the companion sets it) plus the local NED offset, i.e. the
        // dead-reckoned flow position. The GPS is not fused into the EKF, so this is NOT the GPS position.
        var fcLat = Double.nan, fcLon = Double.nan
        let olat = fv("GPS_GLOBAL_ORIGIN", "latitude") / 1e7, olon = fv("GPS_GLOBAL_ORIGIN", "longitude") / 1e7
        let n = fv("LOCAL_POSITION_NED", "x"), e = fv("LOCAL_POSITION_NED", "y")
        if olat.isFinite && olon.isFinite && n.isFinite && e.isFinite {
            fcLat = olat + n / 111319.49
            fcLon = olon + e / (111319.49 * cos(olat * .pi / 180))
        } else {
            fcLat = fv("GLOBAL_POSITION_INT", "lat") / 1e7    // no origin yet: the FC's global estimate
            fcLon = fv("GLOBAL_POSITION_INT", "lon") / 1e7
        }
        if fcLat.isFinite && fcLon.isFinite {
            s["fcpos"] = ["lat": jnum(fcLat, 7), "lon": jnum(fcLon, 7),
                          "n": jnum(n, 2), "e": jnum(e, 2),
                          "age": jnum(age("LOCAL_POSITION_NED"), 2)]
        }
        // the mission (read only): the items the FC has, and the one it is flying
        if missionCount > 0 || !missionItems.isEmpty {
            var items: [[String: Any]] = []
            for seq in missionItems.keys.sorted() {
                guard let it = missionItems[seq] else { continue }
                items.append(["seq": seq,
                              "lat": jnum((it["x"] as? Double) ?? .nan, 7),
                              "lon": jnum((it["y"] as? Double) ?? .nan, 7),
                              "alt": jnum((it["z"] as? Double) ?? .nan, 2),
                              "frame": Int((it["frame"] as? Double) ?? 0),
                              "cmd": Int((it["command"] as? Double) ?? 0),
                              "current": Int((it["current"] as? Double) ?? 0)])
            }
            s["mission"] = ["count": missionCount, "current": missionCurrent, "items": items,
                            "age": jnum(missionT0 > 0 ? now - missionT0 : nil, 2)]
        }
        if !newTexts.isEmpty {
            s["texts"] = newTexts
            newTexts.removeAll()
        }
        return s
    }
}
