// ECPS Console - macOS app shell: one window with the web UI (WKWebView), native menus and the emergency keys.
// Chinese or English (menu 显示 → 语言 / View → Language, or the page's 中 / EN switch): see Lang in Util.swift.
//
// Esc = emergency LAND, handled natively (an NSEvent monitor, before the page sees the key): it works even when the
// page is busy or hung. Kill (motor stop) from the menu asks for confirmation; in the page it needs a 1.5 s hold.

import AppKit
import WebKit

final class AppDelegate: NSObject, NSApplicationDelegate, WKNavigationDelegate, WKUIDelegate {
    var window: NSWindow!
    var web: WKWebView!
    var core: AppCore!
    var keyMonitor: Any?

    func applicationDidFinishLaunching(_ notification: Notification) {
        let res = Bundle.main.resourceURL ?? URL(fileURLWithPath: CommandLine.arguments[0]).deletingLastPathComponent()
        var root = res.appendingPathComponent("web")
        if let dev = ProcessInfo.processInfo.environment["ECPS_WEB_ROOT"] { root = URL(fileURLWithPath: dev) }  // edit-reload during development
        core = AppCore(webRoot: root)
        core.onLang = { [weak self] in self?.applyLang() }
        do {
            try core.start()
        } catch {
            let a = NSAlert()
            a.messageText = L("无法启动本地界面服务", "Cannot start the local UI server")
            a.informativeText = error.localizedDescription
            a.runModal()
            NSApp.terminate(nil)
            return
        }
        buildMenu()

        let cfg = WKWebViewConfiguration()
        cfg.preferences.setValue(true, forKey: "developerExtrasEnabled")
        cfg.mediaTypesRequiringUserActionForPlayback = []
        web = WKWebView(frame: NSRect(x: 0, y: 0, width: 1520, height: 940), configuration: cfg)
        web.navigationDelegate = self
        web.uiDelegate = self
        if #available(macOS 13.3, *) { web.isInspectable = true }
        web.setValue(false, forKey: "drawsBackground")

        window = NSWindow(contentRect: NSRect(x: 0, y: 0, width: 1520, height: 940),
                          styleMask: [.titled, .closable, .miniaturizable, .resizable], backing: .buffered, defer: false)
        window.title = L("ECPS 控制台", "ECPS Console")
        window.appearance = NSAppearance(named: .darkAqua)
        window.backgroundColor = NSColor(red: 0.02, green: 0.027, blue: 0.043, alpha: 1)
        window.minSize = NSSize(width: 1180, height: 760)
        window.tabbingMode = .disallowed
        window.contentView = web
        window.setFrameAutosaveName("ECPSConsoleMain")
        if !window.setFrameUsingName("ECPSConsoleMain") { window.center() }
        window.makeKeyAndOrderFront(nil)
        web.load(URLRequest(url: core.web.pageURL))

        // Esc: emergency LAND, whatever has the focus inside the window
        keyMonitor = NSEvent.addLocalMonitorForEvents(matching: .keyDown) { [weak self] ev in
            guard let self = self else { return ev }
            if ev.keyCode == 53 && ev.window === self.window && !ev.isARepeat {
                self.core.emergencyLand(source: L("Esc 键", "Esc key"))
                return nil
            }
            return ev
        }
        NSApp.activate(ignoringOtherApps: true)

        // debugging / automated checks: ECPS_SNAPSHOT=<file.png>, then `kill -USR1 <pid>` saves the page as a PNG
        if let snap = ProcessInfo.processInfo.environment["ECPS_SNAPSHOT"] {
            signal(SIGUSR1, SIG_IGN)
            let src = DispatchSource.makeSignalSource(signal: SIGUSR1, queue: .main)
            src.setEventHandler { [weak self] in self?.snapshot(to: snap) }
            src.resume()
            snapSource = src
        }
    }

    var snapSource: DispatchSourceSignal?

    func snapshot(to path: String) {
        web.takeSnapshot(with: nil) { img, err in
            guard let img = img, let tiff = img.tiffRepresentation, let rep = NSBitmapImageRep(data: tiff),
                  let png = rep.representation(using: .png, properties: [:]) else {
                Log.info("snapshot failed: \(err?.localizedDescription ?? "?")")
                return
            }
            try? png.write(to: URL(fileURLWithPath: path))
            let menus = NSApp.mainMenu?.items.compactMap { $0.submenu?.title }.filter { !$0.isEmpty } ?? []
            Log.info("snapshot -> \(path) · menus: \(menus.joined(separator: " / ")) · window: \(self.window.title)")
        }
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }

    func applicationShouldTerminate(_ sender: NSApplication) -> NSApplication.TerminateReply {
        guard core != nil, core.flying else { return .terminateNow }
        let a = NSAlert()
        a.messageText = L("无人机仍处于解锁状态", "The drone is still armed")
        a.informativeText = L("退出后控制台不再显示遥测，也无法从这里急停（遥控器与机载失联保护不受影响）。确定退出？",
                              "After quitting, the console shows no telemetry and cannot stop the drone from here (the RC transmitter and the companion-loss failsafe are not affected). Quit anyway?")
        a.addButton(withTitle: L("取消", "Cancel"))
        a.addButton(withTitle: L("退出", "Quit"))
        a.alertStyle = .critical
        return a.runModal() == .alertSecondButtonReturn ? .terminateNow : .terminateCancel
    }

    // a page that crashed (WebContent process) is reloaded; the links and the safety logic keep running natively
    func webViewWebContentProcessDidTerminate(_ webView: WKWebView) {
        Log.info("web content process ended: reloading the UI")
        webView.load(URLRequest(url: core.web.pageURL))
    }

    func webView(_ webView: WKWebView, runJavaScriptAlertPanelWithMessage message: String, initiatedByFrame frame: WKFrameInfo,
                 completionHandler: @escaping () -> Void) {
        let a = NSAlert()
        a.messageText = message
        a.runModal()
        completionHandler()
    }

    func webView(_ webView: WKWebView, runJavaScriptConfirmPanelWithMessage message: String, initiatedByFrame frame: WKFrameInfo,
                 completionHandler: @escaping (Bool) -> Void) {
        let a = NSAlert()
        a.messageText = message
        a.addButton(withTitle: L("确定", "OK"))
        a.addButton(withTitle: L("取消", "Cancel"))
        completionHandler(a.runModal() == .alertFirstButtonReturn)
    }

    // ------------------------------------------------------------------ menus
    private func buildMenu() {
        let main = NSMenu()
        let appItem = NSMenuItem()
        main.addItem(appItem)
        let appMenu = NSMenu()
        appMenu.addItem(withTitle: L("关于 ECPS 控制台", "About ECPS Console"), action: #selector(about), keyEquivalent: "").target = self
        appMenu.addItem(.separator())
        appMenu.addItem(withTitle: L("隐藏", "Hide ECPS Console"), action: #selector(NSApplication.hide(_:)), keyEquivalent: "h")
        appMenu.addItem(withTitle: L("退出 ECPS 控制台", "Quit ECPS Console"), action: #selector(NSApplication.terminate(_:)), keyEquivalent: "q")
        appItem.submenu = appMenu

        let editItem = NSMenuItem()
        main.addItem(editItem)
        let edit = NSMenu(title: L("编辑", "Edit"))
        edit.addItem(withTitle: L("撤销", "Undo"), action: Selector(("undo:")), keyEquivalent: "z")
        edit.addItem(.separator())
        edit.addItem(withTitle: L("剪切", "Cut"), action: #selector(NSText.cut(_:)), keyEquivalent: "x")
        edit.addItem(withTitle: L("拷贝", "Copy"), action: #selector(NSText.copy(_:)), keyEquivalent: "c")
        edit.addItem(withTitle: L("粘贴", "Paste"), action: #selector(NSText.paste(_:)), keyEquivalent: "v")
        edit.addItem(withTitle: L("全选", "Select All"), action: #selector(NSText.selectAll(_:)), keyEquivalent: "a")
        editItem.submenu = edit

        let flyItem = NSMenuItem()
        main.addItem(flyItem)
        let fly = NSMenu(title: L("飞行", "Flight"))
        let e = fly.addItem(withTitle: L("紧急降落（Esc）", "Emergency Land (Esc)"), action: #selector(menuEmergencyLand), keyEquivalent: "")
        e.target = self
        fly.addItem(withTitle: L("停桨（强制上锁）…", "Kill Motors (Force Disarm)…"), action: #selector(menuKill), keyEquivalent: "").target = self
        fly.addItem(.separator())
        fly.addItem(withTitle: L("降落", "Land"), action: #selector(menuLand), keyEquivalent: "l").target = self
        fly.addItem(withTitle: L("解除急停锁…", "Reset E-stop Latch…"), action: #selector(menuReset), keyEquivalent: "").target = self
        flyItem.submenu = fly

        let viewItem = NSMenuItem()
        main.addItem(viewItem)
        let view = NSMenu(title: L("显示", "View"))
        view.addItem(withTitle: L("重新载入界面", "Reload UI"), action: #selector(reloadUI), keyEquivalent: "r").target = self
        view.addItem(withTitle: L("在浏览器中打开界面（调试）", "Open UI in Browser (Debug)"), action: #selector(openInBrowser), keyEquivalent: "").target = self
        view.addItem(withTitle: L("打开记录文件夹", "Open Recordings Folder"), action: #selector(openLogs), keyEquivalent: "").target = self
        view.addItem(.separator())
        let langItem = view.addItem(withTitle: L("语言", "Language"), action: nil, keyEquivalent: "")
        let langMenu = NSMenu(title: L("语言", "Language"))
        for (code, name) in [("zh", "中文"), ("en", "English")] {  // each language named in itself
            let it = langMenu.addItem(withTitle: name, action: #selector(menuLang(_:)), keyEquivalent: "")
            it.target = self
            it.representedObject = code
            it.state = Lang.current == code ? .on : .off
        }
        langItem.submenu = langMenu
        viewItem.submenu = view

        let winItem = NSMenuItem()
        main.addItem(winItem)
        let win = NSMenu(title: L("窗口", "Window"))
        win.addItem(withTitle: L("最小化", "Minimize"), action: #selector(NSWindow.performMiniaturize(_:)), keyEquivalent: "m")
        win.addItem(withTitle: L("缩放", "Zoom"), action: #selector(NSWindow.performZoom(_:)), keyEquivalent: "")
        winItem.submenu = win
        NSApp.windowsMenu = win
        NSApp.mainMenu = main
    }

    @objc func menuLang(_ sender: NSMenuItem) {
        if let code = sender.representedObject as? String { core.setLang(code) }
    }

    /// The language changed (menu or the page's switch): menus and the window title follow; the page follows the state.
    func applyLang() {
        buildMenu()
        window?.title = L("ECPS 控制台", "ECPS Console")
    }

    @objc func about() {
        let a = NSAlert()
        a.messageText = L("ECPS 控制台", "ECPS Console")
        a.informativeText = L("ECPS295 MiniFly 无人机地面控制台：大脑实时状态、3D 建图、机载相机、飞行参数、指引与急停。\n界面 ",
                              "Ground console for the ECPS295 MiniFly drone: live brain, 3D map, onboard camera, flight data, guidance and emergency stop.\nUI ")
            + core.web.pageURL.absoluteString
        a.runModal()
    }

    @objc func menuEmergencyLand() { core.emergencyLand(source: L("菜单", "menu")) }

    @objc func menuKill() {
        let a = NSAlert()
        a.messageText = L("停桨：强制上锁？", "Kill motors: force disarm?")
        a.informativeText = L("电机会立即停转，无人机将从当前高度落下。只在紧急降落来不及时使用。",
                              "The motors stop at once and the drone falls from its current height. Use it only when an emergency land would be too slow.")
        a.alertStyle = .critical
        a.addButton(withTitle: L("取消", "Cancel"))
        a.addButton(withTitle: L("停桨", "Kill Motors"))
        if a.runModal() == .alertSecondButtonReturn { core.kill(source: L("菜单", "menu")) }
    }

    @objc func menuLand() { core.land(source: L("菜单", "menu")) }

    @objc func menuReset() {
        let a = NSAlert()
        a.messageText = L("解除急停锁？", "Reset the E-stop latch?")
        a.informativeText = L("解除后机载程序可以再次接管控制（飞手需要重新切到 GUIDED）。飞控仍解锁时需强制解除。",
                              "After the reset the companion can take control again (the pilot has to switch to GUIDED again). While the FC is still armed, only a forced reset works.")
        a.addButton(withTitle: L("取消", "Cancel"))
        a.addButton(withTitle: L("解除", "Reset"))
        a.addButton(withTitle: L("强制解除（仍解锁）", "Force Reset (Still Armed)"))
        let r = a.runModal()
        if r == .alertSecondButtonReturn { core.resetEstop(force: false) }
        if r == .alertThirdButtonReturn { core.resetEstop(force: true) }
    }

    @objc func reloadUI() { web.load(URLRequest(url: core.web.pageURL)) }
    @objc func openInBrowser() { NSWorkspace.shared.open(core.web.pageURL) }
    @objc func openLogs() {
        let d = Recorder.base
        try? FileManager.default.createDirectory(at: d, withIntermediateDirectories: true)
        NSWorkspace.shared.open(d)
    }
}

let app = NSApplication.shared
app.setActivationPolicy(.regular)
let delegate = AppDelegate()
app.delegate = delegate
app.run()
