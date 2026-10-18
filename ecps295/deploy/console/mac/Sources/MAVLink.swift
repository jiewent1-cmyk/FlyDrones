// ECPS Console - minimal MAVLink v1/v2 codec, table-driven from MavMeta.swift (generated from pymavlink by
// tools/gen_mavmeta.py). Only the messages the console shows or sends are known; frames of other ids are kept raw
// (for the .tlog recording) but not CRC-checked or decoded.

import Foundation

enum MavType { case u8, i8, u16, i16, u32, i32, u64, i64, f32, f64, char }

struct MavField {
    let name: String
    let type: MavType
    let offset: Int
    let count: Int  // array length, 0 = scalar
    init(_ name: String, _ type: MavType, _ offset: Int, _ count: Int) {
        self.name = name; self.type = type; self.offset = offset; self.count = count
    }
}

struct MavMessageSpec {
    let id: UInt32
    let name: String
    let crcExtra: UInt8
    let length: Int
    let fields: [MavField]
}

struct MavFrame {
    let msgid: UInt32
    let sysid: UInt8
    let compid: UInt8
    let seq: UInt8
    let payload: Data  // zero-extended to the spec length when the id is known (MAVLink 2 trims trailing zeros)
    let raw: Data
    let known: Bool
}

@inline(__always) func crcAccumulate(_ b: UInt8, _ crc: inout UInt16) {
    var tmp = b ^ UInt8(truncatingIfNeeded: crc & 0xFF)
    tmp ^= tmp << 4
    crc = (crc >> 8) ^ (UInt16(tmp) << 8) ^ (UInt16(tmp) << 3) ^ (UInt16(tmp) >> 4)
}

func mavCRC(_ data: Data, from: Int, to: Int, extra: UInt8) -> UInt16 {
    var crc: UInt16 = 0xFFFF
    for i in from..<to { crcAccumulate(data.u8(i), &crc) }
    crcAccumulate(extra, &crc)
    return crc
}

final class MavParser {
    private(set) var crcErrors = 0
    private(set) var unknown = 0
    private(set) var frames = 0

    /// One UDP datagram may carry several frames (mavlink-router packs them); garbage is skipped byte by byte.
    func push(_ d: Data) -> [MavFrame] {
        var out: [MavFrame] = []
        var i = 0
        let n = d.count
        while i < n {
            let stx = d.u8(i)
            if stx == 0xFD, i + 12 <= n {
                let len = Int(d.u8(i + 1))
                let incompat = d.u8(i + 2)
                let total = 12 + len + ((incompat & 1) != 0 ? 13 : 0)
                if i + total > n { break }
                let msgid = UInt32(d.u8(i + 7)) | UInt32(d.u8(i + 8)) << 8 | UInt32(d.u8(i + 9)) << 16
                let raw = d.subdata(in: (d.startIndex + i)..<(d.startIndex + i + total))
                if let spec = mavMessages[msgid] {
                    let crc = UInt16(d.u8(i + 10 + len)) | UInt16(d.u8(i + 11 + len)) << 8
                    if crc != mavCRC(d, from: i + 1, to: i + 10 + len, extra: spec.crcExtra) {
                        crcErrors += 1
                        i += 1
                        continue
                    }
                    var p = d.subdata(in: (d.startIndex + i + 10)..<(d.startIndex + i + 10 + len))
                    if p.count < spec.length { p.append(Data(count: spec.length - p.count)) }
                    out.append(MavFrame(msgid: msgid, sysid: d.u8(i + 5), compid: d.u8(i + 6), seq: d.u8(i + 4), payload: p, raw: raw, known: true))
                } else {
                    unknown += 1
                    out.append(MavFrame(msgid: msgid, sysid: d.u8(i + 5), compid: d.u8(i + 6), seq: d.u8(i + 4), payload: Data(), raw: raw, known: false))
                }
                frames += 1
                i += total
            } else if stx == 0xFE, i + 8 <= n {
                let len = Int(d.u8(i + 1))
                let total = 8 + len
                if i + total > n { break }
                let msgid = UInt32(d.u8(i + 5))
                let raw = d.subdata(in: (d.startIndex + i)..<(d.startIndex + i + total))
                if let spec = mavMessages[msgid] {
                    let crc = UInt16(d.u8(i + 6 + len)) | UInt16(d.u8(i + 7 + len)) << 8
                    if crc != mavCRC(d, from: i + 1, to: i + 6 + len, extra: spec.crcExtra) {
                        crcErrors += 1
                        i += 1
                        continue
                    }
                    var p = d.subdata(in: (d.startIndex + i + 6)..<(d.startIndex + i + 6 + len))
                    if p.count < spec.length { p.append(Data(count: spec.length - p.count)) }
                    out.append(MavFrame(msgid: msgid, sysid: d.u8(i + 3), compid: d.u8(i + 4), seq: d.u8(i + 2), payload: p, raw: raw, known: true))
                } else {
                    unknown += 1
                    out.append(MavFrame(msgid: msgid, sysid: d.u8(i + 3), compid: d.u8(i + 4), seq: d.u8(i + 2), payload: Data(), raw: raw, known: false))
                }
                frames += 1
                i += total
            } else {
                i += 1
            }
        }
        return out
    }
}

/// Decoded fields of a known frame: numbers as Double (UInt64 as Double too), char arrays as String, other arrays as [Double].
func mavDecode(_ f: MavFrame) -> [String: Any] {
    guard f.known, let spec = mavMessages[f.msgid] else { return [:] }
    var d: [String: Any] = [:]
    let p = f.payload
    func scalar(_ t: MavType, _ o: Int) -> Double {
        switch t {
        case .u8, .char: return Double(p.u8(o))
        case .i8: return Double(Int8(bitPattern: p.u8(o)))
        case .u16: return Double(p.u16(o))
        case .i16: return Double(Int16(bitPattern: p.u16(o)))
        case .u32: return Double(p.u32(o))
        case .i32: return Double(Int32(bitPattern: p.u32(o)))
        case .u64: return Double(p.u64(o))
        case .i64: return Double(Int64(bitPattern: p.u64(o)))
        case .f32: return Double(Float(bitPattern: p.u32(o)))
        case .f64: return Double(bitPattern: p.u64(o))
        }
    }
    func size(_ t: MavType) -> Int {
        switch t {
        case .u8, .i8, .char: return 1
        case .u16, .i16: return 2
        case .u32, .i32, .f32: return 4
        case .u64, .i64, .f64: return 8
        }
    }
    for fd in spec.fields {
        if fd.type == .char && fd.count > 0 {
            var bytes = [UInt8]()
            for k in 0..<fd.count {
                let b = p.u8(fd.offset + k)
                if b == 0 { break }
                bytes.append(b)
            }
            d[fd.name] = String(decoding: bytes, as: UTF8.self)
        } else if fd.count > 0 {
            d[fd.name] = (0..<fd.count).map { scalar(fd.type, fd.offset + $0 * size(fd.type)) }
        } else {
            d[fd.name] = scalar(fd.type, fd.offset)
        }
    }
    return d
}

final class MavEncoder {
    var sysid: UInt8
    var compid: UInt8
    private var seq: UInt8 = 0

    init(sysid: UInt8, compid: UInt8) {
        self.sysid = sysid
        self.compid = compid
    }

    /// MAVLink 2 frame (no signing, payload not trimmed: receivers accept both).
    func frame(_ msgid: UInt32, _ payload: Data) -> Data {
        guard let spec = mavMessages[msgid] else { return Data() }
        var f = Data([0xFD, UInt8(payload.count), 0, 0, seq, sysid, compid,
                      UInt8(msgid & 0xFF), UInt8((msgid >> 8) & 0xFF), UInt8((msgid >> 16) & 0xFF)])
        f.append(payload)
        let crc = mavCRC(f, from: 1, to: f.count, extra: spec.crcExtra)
        f.appendLE(crc)
        seq &+= 1
        return f
    }

    /// HEARTBEAT as a ground station (MAV_TYPE_GCS 6, MAV_AUTOPILOT_INVALID 8).
    func heartbeat() -> Data {
        var p = Data()
        p.appendLE(UInt32(0))
        p.append(contentsOf: [6, 8, 0, 4, 3])  // type, autopilot, base_mode, system_status ACTIVE, mavlink_version
        return frame(0, p)
    }

    func commandLong(target: UInt8, comp: UInt8, command: UInt16, _ params: [Float]) -> Data {
        var p = Data()
        for k in 0..<7 { p.appendF32(k < params.count ? params[k] : 0) }
        p.appendLE(command)
        p.append(contentsOf: [target, comp, 0])
        return frame(76, p)
    }

    /// SET_MODE (deprecated but still honoured by ArduPilot): base_mode MAV_MODE_FLAG_CUSTOM_MODE_ENABLED.
    func setMode(target: UInt8, customMode: UInt32) -> Data {
        var p = Data()
        p.appendLE(customMode)
        p.append(contentsOf: [target, 1])
        return frame(11, p)
    }
}

/// ArduCopter custom_mode numbers.
let copterModes: [UInt32: String] = [
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND",
    11: "DRIFT", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
    20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE",
    27: "AUTO_RTL", 28: "TURTLE",
]
let MODE_LAND: UInt32 = 9
