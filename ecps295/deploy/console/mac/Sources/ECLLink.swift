// ECPS Console - the companion link (ECL1 over UDP, companion/console_link.py on the Pi, default port 14700).
//
// Datagram: <4sBBHHHI magic 'ECL1', type, flags (bit 0 zlib), msg, chunk, nchunk, stamp ms> + <= 1200 payload bytes.
// Everything that arrives compressed is inflated here, so the web UI gets plain payloads:
//   TICK / ACK / STATUS / TOPO   JSON
//   BRAIN   <IHBBB tick, n, rows, cols, nfeat> + uint8 counts[n] + uint8 grids[2][nfeat][rows*cols]
//   MAP     <H json length> + JSON + uint32 delta-coded voxel indices[nvox] + uint8 confidence[nvox]
//   CAM     <BHHI kind, w, h, frame> + JPEG;   TOF  unchanged
// Commands carry an id and are re-sent until the companion acknowledges them (8 tries, 300 ms apart).
// Bandwidth: the Pi's Wi-Fi stalls for seconds above ~40 kB/s, so the console asks for a level 0-3 (HELLO "lvl") and
// adapts it from the PING round trips (4 Hz): down at once when a reply takes > 400 ms or several are lost, up again
// after 6 s of a clean link. An emergency stop drops to level 0 for 15 s to free the air for the stop and its
// confirmation. The link is marked as interactive voice (WMM AC_VO) so the AP queues it ahead of bulk traffic.

import Foundation
import Network

final class ECLLink {
    static let MAGIC: [UInt8] = Array("ECL1".utf8)
    static let T_TICK: UInt8 = 1, T_BRAIN: UInt8 = 2, T_CAM: UInt8 = 3, T_MAP: UInt8 = 4, T_TOF: UInt8 = 5, T_TOPO: UInt8 = 6
    static let T_ACK: UInt8 = 7, T_STATUS: UInt8 = 8, T_PONG: UInt8 = 9
    static let T_HELLO: UInt8 = 64, T_PING: UInt8 = 65, T_CMD: UInt8 = 66, T_ESTOP: UInt8 = 67, T_TOPO_REQ: UInt8 = 68

    let q: DispatchQueue
    private var conn: NWConnection?
    private(set) var host = ""
    private(set) var port: UInt16 = 14700
    private(set) var state = "idle"
    var prefs: [String: Any] = ["name": "ECPS Console (Mac)", "cam": "brain", "cam_hz": 5, "color_w": 480, "color_q": 55]

    private var msg: UInt16 = 0
    private var parts: [UInt32: (t: Double, n: Int, chunks: [Int: Data])] = [:]
    private(set) var lastRx = 0.0
    private(set) var rtt: Double?
    private(set) var rxBytes = 0
    private(set) var counts: [UInt8: Int] = [:]
    private(set) var lastTick: [String: Any]?
    private(set) var lastTickT = 0.0
    private(set) var lastStatus: [String: Any]?
    private(set) var topo: Data?
    private var lastHello = 0.0
    private var lastPing = 0.0
    private var reconnectAt = 0.0
    private var cid = 0
    private struct Pending {
        let data: [Data]
        var tries: Int
        var last: Double
        let op: String
        let done: ((Bool, String) -> Void)?
    }
    private var pending: [Int: Pending] = [:]
    // bandwidth level
    var userLevel: Int? = nil  // nil = automatic
    private(set) var level = 2
    private var levelChanged = 0.0
    private var goodSince = 0.0
    private var forcedUntil = 0.0
    private var pings: [Double: Double] = [:]  // stamp -> sent (mono), until answered or lost
    private var rtts: [(t: Double, ms: Double)] = []
    private var lost: [Double] = []
    private var lastEval = 0.0
    private var holdoff: [Int: Double] = [:]  // level -> no return to it before this time (exponential back-off)
    private var backoff: [Int: Double] = [:]  // level -> current back-off (s)
    private var established = false  // a PONG came back on this connection: only then do lost pings count

    var onMessage: ((UInt8, Data) -> Void)?

    init(queue: DispatchQueue) { q = queue }

    var connected: Bool { lastRx > 0 && mono() - lastRx < 1.5 }

    func connect(host: String, port: UInt16) {
        self.host = host
        self.port = port
        conn?.cancel()
        conn = nil
        parts = [:]
        pings = [:]
        rtts = []
        lost = []
        established = false
        lastTick = nil
        lastStatus = nil
        topo = nil
        guard !host.isEmpty, let p = NWEndpoint.Port(rawValue: port) else { state = "no host"; return }
        let params = NWParameters.udp
        params.serviceClass = .interactiveVoice
        let c = NWConnection(host: NWEndpoint.Host(host), port: p, using: params)
        conn = c
        c.stateUpdateHandler = { [weak self] st in
            guard let self = self else { return }
            switch st {
            case .ready: self.state = "ready"; self.hello(); self.requestTopo()
            case .waiting(let e): self.state = "waiting: \(e.localizedDescription)"
            case .failed(let e): self.state = "failed: \(e.localizedDescription)"; self.reconnectAt = mono() + 1.0
            case .cancelled: break
            default: self.state = "\(st)"
            }
        }
        c.start(queue: q)
        receive(c)
        Log.info("ECPS link -> \(host):\(port)")
    }

    private func receive(_ c: NWConnection) {
        c.receiveMessage { [weak self] data, _, _, err in
            guard let self = self, c === self.conn else { return }
            if let d = data, !d.isEmpty { self.handle(d) }
            if err == nil { self.receive(c) }
        }
    }

    // ------------------------------------------------------------------ send
    private func datagrams(_ typ: UInt8, _ payload: Data) -> [Data] {
        msg &+= 1
        let n = max(1, (payload.count + 1199) / 1200)
        let st = UInt32(truncatingIfNeeded: Int64(mono() * 1000))
        var out: [Data] = []
        for i in 0..<n {
            var d = Data(ECLLink.MAGIC)
            d.append(contentsOf: [typ, 0])
            d.appendLE(msg)
            d.appendLE(UInt16(i))
            d.appendLE(UInt16(n))
            d.appendLE(st)
            let a = payload.startIndex + i * 1200
            d.append(payload.subdata(in: a..<min(payload.endIndex, a + 1200)))
            out.append(d)
        }
        return out
    }

    private func sendRaw(_ dgrams: [Data]) {
        guard let c = conn else { return }
        for d in dgrams { c.send(content: d, completion: .contentProcessed { _ in }) }
    }

    private func sendJSON(_ typ: UInt8, _ obj: [String: Any]) {
        guard let d = jsonData(obj) else { return }
        sendRaw(datagrams(typ, d))
    }

    func hello() {
        var h = prefs
        h["lvl"] = level
        sendJSON(ECLLink.T_HELLO, h)
    }

    /// Emergency: lowest bandwidth for a while, whatever the automatic level says.
    func forceLevel(_ l: Int, seconds: Double) {
        forcedUntil = mono() + seconds
        if l != level { level = l; levelChanged = mono(); hello() }
    }

    private func evaluateLevel(_ now: Double) {
        rtts = rtts.filter { now - $0.t < 3 }
        lost = lost.filter { now - $0 < 3 }
        let unanswered = pings.filter { now - $0.value > 1.5 }.map { $0.key }
        for st in unanswered {
            pings[st] = nil
            if established { lost.append(now) }
        }
        var want = level
        if now < forcedUntil {
            want = 0
        } else if let u = userLevel {
            want = u
        } else {
            let mx = rtts.map { $0.ms }.max() ?? 0
            let sorted = rtts.map { $0.ms }.sorted()
            let p50 = sorted.isEmpty ? 0 : sorted[sorted.count / 2]
            let bad = mx > 400 || lost.count >= 3
            let clean = rtts.count >= 6 && p50 < 80 && mx < 200 && lost.isEmpty
            goodSince = clean ? (goodSince == 0 ? now : goodSince) : 0
            if bad && level > 0 && now - levelChanged > 2 {
                want = level - 1
                // the level we leave was too much: wait longer each time before trying it again (30, 60, 120 s)
                let b = min(120, (backoff[level] ?? 15) * 2)
                backoff[level] = b
                holdoff[level] = now + b
            } else if clean && level < 3 && now - goodSince > 6 && now - levelChanged > 6 && now >= (holdoff[level + 1] ?? 0) {
                want = level + 1
            }
            if clean && now - levelChanged > 60 { backoff[level] = nil }  // stable here for a minute: forget the back-off
        }
        if want != level {
            Log.info("ECPS link level \(level) -> \(want)")
            level = want
            levelChanged = now
            rtts = []  // judge the new level on its own round trips (a spike before the change must not count twice)
            lost = []
            hello()
        }
    }

    func requestTopo() { sendRaw(datagrams(ECLLink.T_TOPO_REQ, Data())) }

    @discardableResult
    func sendCmd(_ op: String, _ args: [String: Any] = [:], done: ((Bool, String) -> Void)? = nil) -> Int {
        cid += 1
        var obj = args
        obj["id"] = cid
        obj["op"] = op
        guard let d = jsonData(obj) else { done?(false, "bad args"); return cid }
        let dg = datagrams(ECLLink.T_CMD, d)
        pending[cid] = Pending(data: dg, tries: 1, last: mono(), op: op, done: done)
        sendRaw(dg)
        return cid
    }

    /// Operator sticks (MANUAL): fire and forget, ~10 Hz while a key is held.
    func sendStick(_ f: Double, _ l: Double, _ y: Double, _ z: Double) {
        sendJSON(ECLLink.T_CMD, ["op": "stick", "f": f, "l": l, "y": y, "z": z])
    }

    /// Emergency stop (1 land, 2 kill); Safety repeats it. `trig` numbers the operator's presses: a new press makes the
    /// companion act again even after it saw an earlier stop confirmed (and the pilot took over).
    func sendEstop(_ level: Int, trig: Int) {
        cid += 1
        sendJSON(ECLLink.T_ESTOP, ["id": cid, "level": level, "trig": trig])
    }

    /// 20 Hz from AppCore (on q).
    func tick() {
        let now = mono()
        if reconnectAt > 0, now >= reconnectAt {
            reconnectAt = 0
            connect(host: host, port: port)
        }
        if now - lastHello >= 0.2 { lastHello = now; hello() }
        if now - lastPing >= 0.25 {
            lastPing = now
            var d = Data()
            d.appendLE(now.bitPattern)
            pings[now] = now
            sendRaw(datagrams(ECLLink.T_PING, d))
        }
        if now - lastEval >= 0.5 {
            lastEval = now
            evaluateLevel(now)
        }
        if connected, topo == nil, Int(now * 2) % 4 == 0 { requestTopo() }
        for (id, p) in pending where now - p.last >= 0.3 {
            if p.tries >= 8 {  // 2.4 s: a congested Wi-Fi link (commands are idempotent, a repeat does no harm)
                pending[id] = nil
                p.done?(false, L("机载程序无应答", "no answer from the companion"))
            } else {
                pending[id]?.tries += 1
                pending[id]?.last = now
                sendRaw(p.data)
            }
        }
        let stale = parts.filter { now - $0.value.t > 1.0 }.map { $0.key }
        for k in stale { parts[k] = nil }
    }

    // ------------------------------------------------------------------ receive
    private func handle(_ d: Data) {
        guard d.count >= 16, Array(d.prefix(4)) == ECLLink.MAGIC else { return }
        let typ = d.u8(4), flags = d.u8(5), m = d.u16(6), chunk = Int(d.u16(8)), n = Int(d.u16(10))
        guard n >= 1, chunk < n else { return }
        rxBytes += d.count
        lastRx = mono()
        var body = d.subdata(in: (d.startIndex + 16)..<d.endIndex)
        if n > 1 {
            let key = UInt32(typ) << 16 | UInt32(m)
            var e = parts[key] ?? (mono(), n, [:])
            e.chunks[chunk] = body
            if e.chunks.count < n { parts[key] = e; return }
            parts[key] = nil
            var all = Data()
            for i in 0..<n { all.append(e.chunks[i] ?? Data()) }
            body = all
        }
        if flags & 1 != 0 {
            guard let b = zlibInflate(body) else { return }
            body = b
        }
        counts[typ, default: 0] += 1
        switch typ {
        case ECLLink.T_TICK:
            lastTick = jsonObject(body)
            lastTickT = mono()
        case ECLLink.T_STATUS:
            lastStatus = jsonObject(body)
        case ECLLink.T_TOPO:
            topo = body
        case ECLLink.T_ACK:
            if let a = jsonObject(body), let id = a["id"] as? Int, let p = pending[id] {
                pending[id] = nil
                p.done?((a["ok"] as? Bool) ?? false, (a["msg"] as? String) ?? "")
            }
        case ECLLink.T_PONG:
            if body.count >= 8 {
                let st = Double(bitPattern: body.u64(0))
                let ms = (mono() - st) * 1000
                rtt = ms
                if pings.removeValue(forKey: st) != nil { rtts.append((mono(), ms)) }
                established = true
            }
            return
        case ECLLink.T_BRAIN:  // inflate the inner zlib block
            guard body.count > 9, let raw = zlibInflate(body.subdata(in: (body.startIndex + 9)..<body.endIndex)) else { return }
            body = body.prefix(9) + raw
        case ECLLink.T_MAP:
            guard body.count > 2 else { return }
            let hl = Int(body.u16(0))
            guard body.count >= 2 + hl, let raw = zlibInflate(body.subdata(in: (body.startIndex + 2 + hl)..<body.endIndex)) else { return }
            body = body.prefix(2 + hl) + raw
        default:
            break
        }
        onMessage?(typ, body)
    }

    func snapshot() -> [String: Any] {
        [
            "host": host, "port": Int(port), "state": state, "connected": connected,
            "age": jnum(lastRx > 0 ? mono() - lastRx : nil, 2), "rtt": jnum(rtt, 1), "kB": rxBytes / 1024,
            "ticks": counts[ECLLink.T_TICK] ?? 0, "topo": topo != nil,
            "level": level, "auto": userLevel == nil, "forced": mono() < forcedUntil,
            "rtt_max": jnum(rtts.map { $0.ms }.max(), 0), "lost": lost.count,
        ]
    }
}
