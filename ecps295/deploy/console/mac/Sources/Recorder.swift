// ECPS Console - session recording: every MAVLink frame as a .tlog (8-byte big-endian microseconds + the raw frame,
// readable by Mission Planner / pymavlink), the companion's TICK / STATUS messages and the console's own events as JSON
// lines, in ~/Library/Logs/ECPS Console/<start time>/ (not a synced folder). Camera frames and the map are not recorded.
//
// The network queue only appends to memory buffers; a separate queue writes them out every 0.5 s (2026-10-17: writing
// each of ~130 frames/s straight into a synced ~/Documents folder backed the network queue up by seconds).

import Foundation

final class Recorder {
    private let io = DispatchQueue(label: "ecps.rec")
    private var timer: DispatchSourceTimer?
    private var files: [String: FileHandle] = [:]  // used on io only
    private var buf: [String: Data] = [:]  // filled on the caller's queue (net), swapped out under the lock
    private let lock = NSLock()
    private(set) var dir: URL?
    private(set) var bytes = 0
    private(set) var started = 0.0
    var auto = true  // start when the FC arms, stop 10 s after it disarms
    private var disarmedSince = 0.0
    private(set) var active = false

    static var base: URL {
        FileManager.default.homeDirectoryForCurrentUser.appendingPathComponent("Library/Logs/ECPS Console")
    }

    static func freeBytes() -> Int64 {
        let v = try? FileManager.default.homeDirectoryForCurrentUser.resourceValues(forKeys: [.volumeAvailableCapacityForImportantUsageKey])
        return v?.volumeAvailableCapacityForImportantUsage ?? 0
    }

    @discardableResult
    func start(reason: String) -> String {
        if active { return L("已在记录", "already recording") }
        if Recorder.freeBytes() < 300 * 1024 * 1024 { return L("磁盘剩余空间不足 300 MB，未开始记录", "less than 300 MB of free disk space: not recording") }
        let f = DateFormatter()
        f.dateFormat = "yyyyMMdd_HHmmss"
        let d = Recorder.base.appendingPathComponent(f.string(from: Date()))
        do {
            try FileManager.default.createDirectory(at: d, withIntermediateDirectories: true)
            var hs: [String: FileHandle] = [:]
            for name in ["fc.tlog", "ecps.jsonl", "events.jsonl"] {
                let p = d.appendingPathComponent(name)
                FileManager.default.createFile(atPath: p.path, contents: nil)
                hs[name] = try FileHandle(forWritingTo: p)
            }
            io.sync { files = hs }
        } catch {
            return L("无法创建记录目录：", "cannot create the recording folder: ") + error.localizedDescription
        }
        dir = d
        bytes = 0
        started = mono()
        active = true
        let t = DispatchSource.makeTimerSource(queue: io)
        t.schedule(deadline: .now() + 0.5, repeating: 0.5)
        t.setEventHandler { [weak self] in self?.flush() }
        t.resume()
        timer = t
        event("record", L("开始记录（\(reason)）", "recording started (\(reason))"))
        Log.info("recording -> \(d.path)")
        return ""
    }

    func stop() {
        guard active else { return }
        active = false
        timer?.cancel()
        timer = nil
        io.async {
            self.flush()
            for h in self.files.values { try? h.close() }
            self.files = [:]
        }
    }

    private func append(_ name: String, _ d: Data) {
        guard active else { return }
        lock.lock()
        buf[name, default: Data()].append(d)
        lock.unlock()
        bytes += d.count
    }

    private func flush() {
        lock.lock()
        let b = buf
        buf = [:]
        lock.unlock()
        for (name, d) in b where !d.isEmpty { files[name]?.write(d) }
    }

    func frame(_ raw: Data) {
        guard active else { return }
        var d = Data()
        d.appendLE(UInt64(Date().timeIntervalSince1970 * 1e6).byteSwapped)  // big-endian microseconds
        d.append(raw)
        append("fc.tlog", d)
    }

    func ecpsMessage(_ kind: String, _ json: Data) {
        guard active else { return }
        var d = Data("{\"t\":\(String(format: "%.3f", Date().timeIntervalSince1970)),\"k\":\"\(kind)\",\"m\":".utf8)
        d.append(json)
        d.append(Data("}\n".utf8))
        append("ecps.jsonl", d)
    }

    func event(_ level: String, _ text: String) {
        guard active, let d = jsonData(["t": Date().timeIntervalSince1970, "level": level, "text": text]) else { return }
        append("events.jsonl", d + Data("\n".utf8))
    }

    /// Auto start / stop with the FC's arming state (10 Hz from AppCore).
    func autoUpdate(armed: Bool, fcFresh: Bool) {
        guard auto else { return }
        if armed && fcFresh && !active { start(reason: L("飞控已解锁", "FC armed")) }
        if active {
            if armed { disarmedSince = 0 } else if disarmedSince == 0 { disarmedSince = mono() }
            if disarmedSince > 0 && mono() - disarmedSince > 10 && mono() - started > 10 {
                event("record", L("停止记录（上锁 10 s）", "recording stopped (disarmed for 10 s)"))
                stop()
                disarmedSince = 0
            }
        }
    }

    func snapshot() -> [String: Any] {
        ["active": active, "auto": auto, "dir": dir?.path ?? "", "MB": jnum(Double(bytes) / 1048576, 2)]
    }
}
