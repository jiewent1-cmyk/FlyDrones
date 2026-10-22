// ECPS Console - emergency stop and landing (runs natively, independent of the web UI: the Esc key and the menu reach
// it even if the page hangs).
//
// Emergency LAND (big red button, Esc): sent on BOTH paths every 200 ms until the FC's own heartbeat reports LAND or
//   disarmed - (1) straight to the FC through mavlink-router (MAV_CMD_DO_SET_MODE + SET_MODE, mode 9), which works even
//   if the companion process hung; (2) ESTOP to the companion, which latches, releases control at once and sends LAND
//   itself. Once the FC confirms, the console stops sending: if the pilot then flips the mode switch, the pilot wins.
// Kill (hold 1.5 s): MAV_CMD_COMPONENT_ARM_DISARM 0 + force 21196 on both paths every 150 ms until the FC reports
//   disarmed. The motors stop in the air: the drone falls.
// Normal land: DO_SET_MODE LAND until the FC reports it (5 s), plus the companion's "land" command.

import Foundation

final class Safety {
    private let fc: FCLink
    private let ecl: ECLLink
    var onEvent: ((String, String) -> Void)?  // (level, text) for the UI log + recorder

    private(set) var landActive = false
    private(set) var landStart = 0.0
    private(set) var landConfirmed = 0.0
    private(set) var killActive = false
    private(set) var killStart = 0.0
    private(set) var killConfirmed = 0.0
    private(set) var landAbandoned = false
    private(set) var killAbandoned = false
    private(set) var normalLand = 0.0
    private(set) var source = ""
    private var lastLandSend = 0.0
    private var lastKillSend = 0.0
    private var lastNormalSend = 0.0
    private var sends = 0
    private var trig = 0  // operator presses (ECL ESTOP "trig")

    init(fc: FCLink, ecl: ECLLink) {
        self.fc = fc
        self.ecl = ecl
    }

    private func fcFresh() -> Bool { fc.hbAge < 2.0 }

    func emergencyLand(source: String) {
        trig += 1
        if !landActive || landConfirmed > 0 || landAbandoned {
            landActive = true
            landStart = mono()
            landConfirmed = 0
            landAbandoned = false
            self.source = source
            onEvent?("critical", L("紧急降落（\(source)）：向飞控和机载程序同时发送 LAND", "Emergency land (\(source)): LAND sent to the FC and the companion"))
        }
        ecl.forceLevel(0, seconds: 15)  // free the Wi-Fi for the stop and its confirmation
        lastLandSend = 0
        tick()
    }

    func kill(source: String) {
        trig += 1
        if !killActive || killConfirmed > 0 || killAbandoned {
            killActive = true
            killStart = mono()
            killConfirmed = 0
            killAbandoned = false
            self.source = source
            onEvent?("critical", L("停桨（\(source)）：强制上锁，电机立即停转", "Kill (\(source)): force disarm, the motors stop at once"))
        }
        ecl.forceLevel(0, seconds: 15)
        lastKillSend = 0
        tick()
    }

    func land(source: String) {
        normalLand = mono()
        lastNormalSend = 0
        onEvent?("warning", L("降落（\(source)）", "Land (\(source))"))
        ecl.sendCmd("land") { [weak self] ok, msg in
            self?.onEvent?(ok ? "info" : "warning", L("机载程序 land：\(ok ? "已确认" : "未确认") \(msg)", "Companion land: \(ok ? "acknowledged" : "not acknowledged") \(msg)"))
        }
        tick()
    }

    /// Clear the console side; the companion latch is reset by its own command (estop_reset).
    func reset(force: Bool) {
        guard fcFresh(), (!fc.armed || force) else {
            onEvent?("warning", L("飞控状态过期或仍解锁，拒绝解除急停", "Cannot reset E-stop: FC state is stale or still armed"))
            return
        }
        ecl.sendCmd("estop_reset", ["force": force]) { [weak self] ok, msg in
            self?.onEvent?(ok ? "info" : "warning", ok ? L("机载程序急停锁已解除", "Companion E-stop latch reset")
                                                        : L("机载程序拒绝解除急停：\(msg)", "The companion refused the E-stop reset: \(msg)"))
            if ok {
                self?.landActive = false
                self?.killActive = false
                self?.landAbandoned = false
                self?.killAbandoned = false
            }
        }
    }

    /// Cancel the console's own repetition (e.g. the pilot took over): the companion latch stays until reset.
    func stopSending() {
        if landActive && landConfirmed == 0 {
            landAbandoned = true
            onEvent?("warning", L("已停止重发紧急降落；飞控未确认", "Stopped resending the emergency land; the FC has not confirmed it"))
        }
        if killActive && killConfirmed == 0 {
            killAbandoned = true
            onEvent?("warning", L("已停止重发停桨；飞控未确认", "Stopped resending motor kill; the FC has not confirmed it"))
        }
        normalLand = 0
    }

    /// COMMAND_ACK from the FC (FCLink.onAck): DO_SET_MODE / ARM_DISARM accepted -> confirmed without waiting for the
    /// next 1 Hz heartbeat.
    func ack(command: UInt16, result: UInt8) {
        guard result == 0 else {
            onEvent?("warning", L("飞控拒绝指令 \(command)（结果 \(result)）", "The FC rejected command \(command) (result \(result))"))
            return
        }
        let now = mono()
        if command == 176 && landActive && landConfirmed == 0 {
            landConfirmed = now
            onEvent?("info", String(format: L("飞控已接受 LAND（紧急降落后 %.2f s，COMMAND_ACK）", "The FC accepted LAND (%.2f s after the emergency land, COMMAND_ACK)"), now - landStart))
        }
        if command == 176 && normalLand > 0 {
            onEvent?("info", L("飞控已接受 LAND", "The FC accepted LAND"))
            normalLand = 0
        }
        if command == 400 && killActive && killConfirmed == 0 {
            killConfirmed = now
            onEvent?("info", String(format: L("飞控已接受强制上锁（停桨后 %.2f s，COMMAND_ACK）", "The FC accepted the force disarm (%.2f s after the kill, COMMAND_ACK)"), now - killStart))
        }
    }

    /// 10 Hz from AppCore (on the network queue), and right away on a trigger.
    func tick() {
        let now = mono()
        if killActive && killConfirmed == 0 && !killAbandoned {
            if fcFresh() && !fc.armed {
                killConfirmed = now
                onEvent?("info", String(format: L("飞控已上锁（停桨后 %.2f s）", "The FC disarmed (%.2f s after the kill)"), now - killStart))
            } else if now - lastKillSend >= 0.15 {
                lastKillSend = now
                fc.forceDisarm()
                ecl.sendEstop(2, trig: trig)
                sends += 1
            }
        }
        if killActive && killConfirmed > 0 && now - killConfirmed < 2.0 && now - lastKillSend >= 0.3 {
            lastKillSend = now
            ecl.sendEstop(2, trig: trig)  // make sure the companion latched too
        }
        if landActive && landConfirmed == 0 && !landAbandoned && !(killActive && killConfirmed == 0) {
            if fcFresh() && (fc.mode == MODE_LAND || !fc.armed) {
                landConfirmed = now
                onEvent?("info", String(format: fc.armed ? L("飞控已进入 LAND（紧急降落后 %.2f s）", "The FC is in LAND (%.2f s after the emergency land)")
                                                         : L("飞控已上锁（紧急降落后 %.2f s）", "The FC disarmed (%.2f s after the emergency land)"), now - landStart))
            } else if now - lastLandSend >= 0.2 {
                lastLandSend = now
                fc.setMode(MODE_LAND)
                ecl.sendEstop(1, trig: trig)
                sends += 1
            }
        }
        if landActive && landConfirmed > 0 && now - landConfirmed < 3.0 && now - lastLandSend >= 0.5 {
            lastLandSend = now
            ecl.sendEstop(1, trig: trig)  // the companion latch, a few more times (no more FC mode commands: the pilot may override)
        }
        if normalLand > 0 {
            if fcFresh() && (fc.mode == MODE_LAND || !fc.armed) {
                onEvent?("info", L("飞控已进入 LAND", "The FC is in LAND"))
                normalLand = 0
            } else if now - normalLand > 5.0 {
                onEvent?("warning", L("降落：5 s 内未收到飞控 LAND 确认", "Land: no LAND confirmation from the FC within 5 s"))
                normalLand = 0
            } else if now - lastNormalSend >= 0.3 {
                lastNormalSend = now
                fc.setMode(MODE_LAND)
            }
        }
    }

    func snapshot() -> [String: Any] {
        let now = mono()
        var s: [String: Any] = ["sends": sends, "source": source]
        if landActive {
            s["land"] = ["t": jnum(now - landStart, 1), "confirmed": landConfirmed > 0, "abandoned": landAbandoned,
                         "after": jnum(landConfirmed > 0 ? landConfirmed - landStart : nil, 2)]
        }
        if killActive {
            s["kill"] = ["t": jnum(now - killStart, 1), "confirmed": killConfirmed > 0, "abandoned": killAbandoned,
                         "after": jnum(killConfirmed > 0 ? killConfirmed - killStart : nil, 2)]
        }
        if normalLand > 0 { s["normal_land"] = jnum(now - normalLand, 1) }
        return s
    }
}
