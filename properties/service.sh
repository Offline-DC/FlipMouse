#!/system/bin/sh

BIN_DIR="${0%/*}"
BIN="$BIN_DIR/mouse"

# Kill any existing instances (ignore errors)
pkill -f "$BIN" 2>/dev/null
pkill -f "FlipMouse" 2>/dev/null
killall mouse 2>/dev/null
killall FlipMouse 2>/dev/null

# Small pause so the old process fully exits and releases /dev/input grabs
sleep 0.2

# Start exactly one instance
"$BIN" >/dev/null 2>&1 &
