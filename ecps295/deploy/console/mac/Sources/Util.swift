// ECPS Console - small helpers: UI language, logging, monotonic clock, zlib, JSON.

import Compression
import Foundation
import Security

enum Log {
    /// Every log line also goes to the web UI's log panel (set by AppCore).
    static var sink: ((String) -> Void)?
    private static let fmt: DateFormatter = {
        let f = DateFormatter()
        f.dateFormat = "HH:mm:ss.SSS"
        return f
    }()

    static func info(_ s: String) {
        let line = "\(fmt.string(from: Date())) \(s)"
        print(line)
        fflush(stdout)
        sink?(line)
    }
}

/// UI language, "zh" or "en": UserDefaults "lang" (menu 显示 → 语言, or the page's 中 / EN switch); unset = the
/// system's first preferred language. Native texts (menus, dialogs, safety events, recorded events) take the language
/// at the moment they are made; the web page follows the state's settings.lang.
enum Lang {
    static var current: String {
        if let l = UserDefaults.standard.string(forKey: "lang"), l == "zh" || l == "en" { return l }
        return (Locale.preferredLanguages.first ?? "zh").hasPrefix("zh") ? "zh" : "en"
    }

    static func set(_ l: String) { UserDefaults.standard.set(l == "en" ? "en" : "zh", forKey: "lang") }
}

/// The text in the current UI language: L("中文", "English").
func L(_ zh: String, _ en: String) -> String { Lang.current == "en" ? en : zh }

private let eclKeyService = "edu.ecps295.console.ecl1"
func loadECLKey() -> Data? {
    let q: [String: Any] = [kSecClass as String: kSecClassGenericPassword,
                            kSecAttrService as String: eclKeyService,
                            kSecAttrAccount as String: "shared-key",
                            kSecReturnData as String: true, kSecMatchLimit as String: kSecMatchLimitOne]
    var result: CFTypeRef?
    guard SecItemCopyMatching(q as CFDictionary, &result) == errSecSuccess else { return nil }
    return result as? Data
}

@discardableResult func saveECLKey(_ data: Data) -> Bool {
    let q: [String: Any] = [kSecClass as String: kSecClassGenericPassword,
                            kSecAttrService as String: eclKeyService,
                            kSecAttrAccount as String: "shared-key"]
    SecItemDelete(q as CFDictionary)
    var item = q
    item[kSecValueData as String] = data
    return SecItemAdd(item as CFDictionary, nil) == errSecSuccess
}

/// Monotonic seconds (not affected by wall-clock changes).
@inline(__always) func mono() -> Double { ProcessInfo.processInfo.systemUptime }

/// zlib (RFC 1950, what Python's zlib.compress writes) -> bytes. Apple's COMPRESSION_ZLIB is raw DEFLATE, so the
/// 2-byte header and the 4-byte Adler-32 trailer are stripped first.
func zlibInflate(_ data: Data) -> Data? {
    guard data.count >= 6 else { return nil }
    return inflateRaw(data.subdata(in: (data.startIndex + 2)..<(data.endIndex - 4)))
}

func inflateRaw(_ input: Data, maximumOutput: Int = 8 * 1024 * 1024) -> Data? {
    let stream = UnsafeMutablePointer<compression_stream>.allocate(capacity: 1)
    defer { stream.deallocate() }
    guard compression_stream_init(stream, COMPRESSION_STREAM_DECODE, COMPRESSION_ZLIB) == COMPRESSION_STATUS_OK else { return nil }
    defer { compression_stream_destroy(stream) }
    let cap = 64 * 1024
    let buf = UnsafeMutablePointer<UInt8>.allocate(capacity: cap)
    defer { buf.deallocate() }
    var out = Data()
    let ok: Bool = input.withUnsafeBytes { (src: UnsafeRawBufferPointer) -> Bool in
        let base = src.bindMemory(to: UInt8.self).baseAddress
        stream.pointee.src_ptr = base ?? UnsafePointer<UInt8>(buf)
        stream.pointee.src_size = input.count
        while true {
            stream.pointee.dst_ptr = buf
            let dstCapacity = min(cap, maximumOutput + 1 - out.count)
            stream.pointee.dst_size = dstCapacity
            let st = compression_stream_process(stream, Int32(COMPRESSION_STREAM_FINALIZE.rawValue))
            out.append(buf, count: dstCapacity - stream.pointee.dst_size)
            if out.count > maximumOutput { return false }
            if st == COMPRESSION_STATUS_END { return true }
            if st != COMPRESSION_STATUS_OK { return false }
            if stream.pointee.src_size == 0 && stream.pointee.dst_size == cap { return true }
        }
    }
    return ok ? out : nil
}

func jsonData(_ obj: Any) -> Data? {
    guard JSONSerialization.isValidJSONObject(obj) else { return nil }
    return try? JSONSerialization.data(withJSONObject: obj, options: [])
}

func jsonObject(_ data: Data) -> [String: Any]? {
    (try? JSONSerialization.jsonObject(with: data, options: [])) as? [String: Any]
}

func jsonString(_ obj: Any) -> String {
    guard let d = jsonData(obj) else { return "{}" }
    return String(data: d, encoding: .utf8) ?? "{}"
}

/// Rounded double for JSON (NaN / inf -> NSNull so the payload stays valid JSON).
func jnum(_ v: Double?, _ digits: Int = 3) -> Any {
    guard let v = v, v.isFinite else { return NSNull() }
    let p = pow(10.0, Double(digits))
    return (v * p).rounded() / p
}

extension Data {
    func u8(_ o: Int) -> UInt8 { self[startIndex + o] }
    func u16(_ o: Int) -> UInt16 { UInt16(u8(o)) | UInt16(u8(o + 1)) << 8 }
    func u32(_ o: Int) -> UInt32 { UInt32(u16(o)) | UInt32(u16(o + 2)) << 16 }
    func u64(_ o: Int) -> UInt64 { UInt64(u32(o)) | UInt64(u32(o + 4)) << 32 }
    mutating func appendLE<T: FixedWidthInteger>(_ v: T) {
        var x = v.littleEndian
        Swift.withUnsafeBytes(of: &x) { append(contentsOf: $0) }
    }
    mutating func appendF32(_ v: Float) { appendLE(v.bitPattern) }
}
