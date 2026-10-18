// ECPS Console - local web server for the UI (127.0.0.1 only): static files of Resources/web over HTTP, live data and
// UI commands over a WebSocket on the next port. The page (in the app's WKWebView, or any browser on this Mac for
// development) must send {"auth": <token>} first; the token is in the URL the app opens.

import Foundation
import Network

final class WebServer {
    let root: URL
    let token: String
    private(set) var httpPort: UInt16 = 0
    private(set) var wsPort: UInt16 = 0
    private let q = DispatchQueue(label: "ecps.web")
    private var http: NWListener?
    private var ws: NWListener?
    private final class Client {
        let conn: NWConnection
        var authed = false
        var queued = 0  // bytes handed to the connection and not yet sent
        init(_ c: NWConnection) { conn = c }
    }
    private var clients: [ObjectIdentifier: Client] = [:]
    var onCommand: (([String: Any]) -> Void)?
    var onClient: (() -> Void)?  // a client authenticated (send it the current state)

    init(root: URL) {
        self.root = root
        token = UUID().uuidString.replacingOccurrences(of: "-", with: "").lowercased()
    }

    /// Binds HTTP on the first free port from `base` and the WebSocket on the port after it.
    func start(base: UInt16) throws {
        var p = base
        while p < base + 40 {
            if let h = try? listener(port: p, websocket: false), let w = try? listener(port: p + 1, websocket: true) {
                http = h
                ws = w
                httpPort = p
                wsPort = p + 1
                h.newConnectionHandler = { [weak self] c in self?.serveHTTP(c) }
                w.newConnectionHandler = { [weak self] c in self?.acceptWS(c) }
                let sem = DispatchSemaphore(value: 0)
                var ready = 0, failed = false
                for l in [h, w] {
                    l.stateUpdateHandler = { st in
                        switch st {
                        case .ready: ready += 1; if ready == 2 { sem.signal() }
                        case .failed: failed = true; sem.signal()
                        default: break
                        }
                    }
                    l.start(queue: q)
                }
                if sem.wait(timeout: .now() + 2) == .success && !failed && ready == 2 { return }
                h.cancel(); w.cancel()
            }
            p += 2
        }
        throw NSError(domain: "ecps.web", code: 1, userInfo: [NSLocalizedDescriptionKey: "no free local port from \(base)"])
    }

    private func listener(port: UInt16, websocket: Bool) throws -> NWListener {
        let params = NWParameters.tcp
        params.allowLocalEndpointReuse = true
        params.requiredLocalEndpoint = NWEndpoint.hostPort(host: "127.0.0.1", port: NWEndpoint.Port(rawValue: port)!)
        if websocket {
            let o = NWProtocolWebSocket.Options()
            o.autoReplyPing = true
            o.maximumMessageSize = 1 << 20
            params.defaultProtocolStack.applicationProtocols.insert(o, at: 0)
        }
        return try NWListener(using: params)
    }

    // ------------------------------------------------------------------ HTTP (GET only, one request per connection)
    private func serveHTTP(_ c: NWConnection) {
        c.start(queue: q)
        var buf = Data()
        func readMore() {
            c.receive(minimumIncompleteLength: 1, maximumLength: 16384) { [weak self] data, _, done, err in
                guard let self = self else { return }
                if let d = data { buf.append(d) }
                if let r = buf.range(of: Data("\r\n\r\n".utf8)) {
                    self.respond(c, String(decoding: buf[buf.startIndex..<r.lowerBound], as: UTF8.self))
                } else if done || err != nil || buf.count > 65536 {
                    c.cancel()
                } else {
                    readMore()
                }
            }
        }
        readMore()
    }

    private static let mime: [String: String] = [
        "html": "text/html; charset=utf-8", "js": "text/javascript; charset=utf-8", "css": "text/css; charset=utf-8",
        "json": "application/json", "png": "image/png", "svg": "image/svg+xml", "ico": "image/x-icon", "jpg": "image/jpeg",
        "woff2": "font/woff2",
    ]

    private func respond(_ c: NWConnection, _ head: String) {
        let first = head.split(separator: "\r\n").first.map(String.init) ?? ""
        let parts = first.split(separator: " ")
        var status = "404 Not Found"
        var body = Data("not found".utf8)
        var type = "text/plain; charset=utf-8"
        if parts.count >= 2, parts[0] == "GET" {
            var path = String(parts[1].split(separator: "?").first ?? "/")
            path = path.removingPercentEncoding ?? path
            if path == "/" { path = "/index.html" }
            let url = root.appendingPathComponent(String(path.dropFirst())).standardizedFileURL
            if url.path.hasPrefix(root.standardizedFileURL.path + "/"), let d = try? Data(contentsOf: url) {
                status = "200 OK"
                body = d
                type = WebServer.mime[url.pathExtension.lowercased()] ?? "application/octet-stream"
            }
        } else {
            status = "405 Method Not Allowed"
        }
        var resp = Data("HTTP/1.1 \(status)\r\nContent-Type: \(type)\r\nContent-Length: \(body.count)\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n".utf8)
        resp.append(body)
        c.send(content: resp, completion: .contentProcessed { _ in c.cancel() })
    }

    // ------------------------------------------------------------------ WebSocket
    private func acceptWS(_ c: NWConnection) {
        let cl = Client(c)
        clients[ObjectIdentifier(c)] = cl
        c.stateUpdateHandler = { [weak self] st in
            switch st {
            case .failed, .cancelled: self?.clients[ObjectIdentifier(c)] = nil
            default: break
            }
        }
        c.start(queue: q)
        receiveWS(cl)
    }

    private func receiveWS(_ cl: Client) {
        cl.conn.receiveMessage { [weak self] data, ctx, _, err in
            guard let self = self else { return }
            if err != nil { cl.conn.cancel(); self.clients[ObjectIdentifier(cl.conn)] = nil; return }
            let md = ctx?.protocolMetadata(definition: NWProtocolWebSocket.definition) as? NWProtocolWebSocket.Metadata
            if md?.opcode == .close { cl.conn.cancel(); self.clients[ObjectIdentifier(cl.conn)] = nil; return }
            if let d = data, md?.opcode == .text, let obj = jsonObject(d) {
                if !cl.authed {
                    if (obj["auth"] as? String) == self.token {
                        cl.authed = true
                        self.onClient?()
                    } else {
                        cl.conn.cancel()
                        return
                    }
                } else {
                    self.onCommand?(obj)
                }
            }
            self.receiveWS(cl)
        }
    }

    private func send(_ cl: Client, _ data: Data, text: Bool, droppable: Bool) {
        if droppable && cl.queued > 4 << 20 { return }  // a stalled page: drop video / map, keep state and acks
        let md = NWProtocolWebSocket.Metadata(opcode: text ? .text : .binary)
        let ctx = NWConnection.ContentContext(identifier: "m", metadata: [md])
        cl.queued += data.count
        let n = data.count
        cl.conn.send(content: data, contentContext: ctx, isComplete: true, completion: .contentProcessed { [weak self] _ in
            self?.q.async { cl.queued -= n }
        })
    }

    /// Thread-safe broadcast to the authenticated pages.
    func broadcast(binary: Data, droppable: Bool = false) {
        q.async { for cl in self.clients.values where cl.authed { self.send(cl, binary, text: false, droppable: droppable) } }
    }

    func broadcast(text: String) {
        let d = Data(text.utf8)
        q.async { for cl in self.clients.values where cl.authed { self.send(cl, d, text: true, droppable: false) } }
    }

    var pageURL: URL { URL(string: "http://127.0.0.1:\(httpPort)/index.html?ws=\(wsPort)&t=\(token)&lang=\(Lang.current)")! }
}
