#!/usr/bin/env bash
# Launch the DroneDeck ground station.
#   ./run.sh            -- start the GCS (listens on UDP 14550)
#   ./run.sh demo       -- also start the test telemetry source, so you see live
#                          data with no hardware
#   ./run.sh 14551      -- start the GCS on a different UDP port
set -euo pipefail
cd "$(dirname "$0")"

# Build the optimized native core if it's missing. The app still runs without
# it (pure-Python fallback), so a build failure is non-fatal.
if [ ! -f core/libdronecore.so ]; then
    echo "native core not built -- building (the app would otherwise use the slower Python fallback)"
    ./build.sh || echo "WARNING: core build failed; continuing with pure-Python fallback"
fi

# Launch the GCS window. Under Hyprland (a tiling WM) a big ground-station window
# gets crushed into a tile, squashing its attitude/compass instruments; so there
# we float + size + centre it. On any other WM/DE this is skipped and the window
# opens normally, keeping run.sh portable (Ubuntu, KDE, GNOME, ...).
launch_gcs() {
    if command -v hyprctl >/dev/null 2>&1 && [ -n "${HYPRLAND_INSTANCE_SIGNATURE:-}" ]; then
        python3 app/main.py "$@" &
        local app=$!
        local addr=""
        local i
        for i in $(seq 1 60); do
            addr=$(hyprctl clients -j 2>/dev/null | python3 -c "import sys, json
try: cs = json.load(sys.stdin)
except Exception: cs = []
print(next((c['address'] for c in cs if c.get('class') == 'DroneDeck'), ''))" 2>/dev/null)
            [ -n "$addr" ] && break
            sleep 0.1
        done
        if [ -n "$addr" ]; then
            # a ground station wants the whole screen -- open it maximized. Hyprland
            # fullscreen mode 1 fills the monitor while keeping the top bar visible.
            hyprctl dispatch focuswindow "address:$addr" >/dev/null 2>&1
            hyprctl dispatch fullscreen 1 >/dev/null 2>&1
        fi
        wait "$app"
    else
        exec python3 app/main.py "$@"
    fi
}

if [ "${1:-}" = "demo" ]; then
    shift || true
    echo "starting TEST telemetry source on UDP 14550 (not a real drone)..."
    python3 sim/simulator.py --target 127.0.0.1:14550 &
    SIM_PID=$!
    trap 'kill "$SIM_PID" 2>/dev/null || true' EXIT INT TERM
    launch_gcs "$@"
else
    launch_gcs "$@"
fi
