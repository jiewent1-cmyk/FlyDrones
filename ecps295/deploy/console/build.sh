#!/bin/bash
# Build "ECPS Console.app" with the Command Line Tools only (no Xcode): swiftc + a hand-made bundle, ad-hoc signed.
# The bundle is built outside ~/Documents (a synced folder adds Finder/file-provider attributes that codesign rejects).
# Right after an edit the synced folder sometimes hands out a stale or partial file, so the sources are copied to a
# scratch folder first and the build is retried (up to 3 times) when the copy does not compile.
#   ./build.sh            -> ~/Library/Caches/ECPSConsole/ECPS Console.app
#   ./build.sh --install  -> and copy it to /Applications
set -uo pipefail
cd "$(dirname "$0")"
OUT="${ECPS_BUILD_DIR:-$HOME/Library/Caches/ECPSConsole}"
APP="$OUT/ECPS Console.app"
SRC="$OUT/src"
mkdir -p "$OUT"
ok=0
for try in 1 2 3; do
  rm -rf "$SRC" "$APP"
  mkdir -p "$SRC" "$APP/Contents/MacOS" "$APP/Contents/Resources"
  cp -R mac/Sources mac/web mac/Info.plist "$SRC"/ && [ ! -f mac/AppIcon.icns ] || cp mac/AppIcon.icns "$SRC"/
  if swiftc -O -swift-version 5 -target "$(uname -m)-apple-macos13.0" \
       -framework AppKit -framework WebKit -framework Network \
       "$SRC"/Sources/*.swift -o "$APP/Contents/MacOS/ECPSConsole" 2> "$OUT/swiftc.log"; then
    js_ok=1
    if command -v node >/dev/null; then
      for f in "$SRC"/web/js/*.js; do node --experimental-default-type=module --check "$f" 2>>"$OUT/swiftc.log" || js_ok=0; done
    fi
    if [ $js_ok = 1 ]; then ok=1; break; fi
  fi
  cp "$OUT/swiftc.log" "$OUT/swiftc.attempt$try.log"
  echo "build attempt $try failed (see $OUT/swiftc.attempt$try.log), retrying..." >&2
  sleep 2
done
if [ $ok != 1 ]; then cat "$OUT/swiftc.log" >&2; exit 1; fi
grep -E "warning" "$OUT/swiftc.log" >&2 || true
cp "$SRC/Info.plist" "$APP/Contents/Info.plist"
cp -R "$SRC/web" "$APP/Contents/Resources/web"
find "$APP/Contents/Resources/web" -name .DS_Store -delete
[ -f "$SRC/AppIcon.icns" ] && cp "$SRC/AppIcon.icns" "$APP/Contents/Resources/AppIcon.icns"
xattr -cr "$APP"
codesign --force --deep --sign - "$APP" || exit 1
echo "built: $APP ($(du -sh "$APP" | cut -f1))"
if [ "${1:-}" = "--install" ]; then
  rm -rf "/Applications/ECPS Console.app"
  cp -R "$APP" /Applications/
  xattr -cr "/Applications/ECPS Console.app"
  echo "installed: /Applications/ECPS Console.app"
fi
