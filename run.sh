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

if [ "${1:-}" = "demo" ]; then
    shift || true
    echo "starting TEST telemetry source on UDP 14550 (not a real drone)..."
    python3 sim/simulator.py --target 127.0.0.1:14550 &
    SIM_PID=$!
    trap 'kill "$SIM_PID" 2>/dev/null || true' EXIT INT TERM
    python3 app/main.py "$@"
else
    exec python3 app/main.py "$@"
fi
