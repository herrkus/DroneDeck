#!/usr/bin/env bash
# Build the native core and run the ENTIRE DroneDeck test suite (native + Python,
# both parser backends). Exits non-zero if anything fails. Relative paths only.
set -uo pipefail
cd "$(dirname "$0")/.."
export QT_QPA_PLATFORM=offscreen
export SCRATCH="${SCRATCH:-$(mktemp -d)}"

pass=0
fail=0
ok()   { echo "  PASS  $1"; pass=$((pass + 1)); }
bad()  { echo "  FAIL  $1"; fail=$((fail + 1)); }

# check <name> <env-prefix-or-empty> <script>
check() {
    local name="$1" env="$2" script="$3"
    local full
    full=$(env ${env} timeout 60 python3 "$script" 2>&1)
    local last
    last=$(echo "$full" | tail -1)
    if echo "$full" | grep -qE "PASSED|VALIDATED" && ! echo "$full" | grep -qE "FAILED|BROKEN"; then
        ok "$name  ($last)"
    else
        bad "$name  ($last)"
    fi
}

echo "== native core =="
if ./build.sh >/dev/null 2>&1; then ok "build + C++ selftest"; else bad "build/selftest"; fi

echo "== python suite =="
check "crc_extra_calc"      ""                        tests/crc_extra_calc.py
check "parity (native)"     ""                        tests/test_parity.py
check "parity (python)"     "DRONEDECK_FORCE_PYTHON=1" tests/test_parity.py
check "links"               ""                        tests/test_links.py
check "mission"             ""                        tests/test_mission.py
check "mission_gui"         ""                        tests/test_mission_gui.py
check "mission_edit"        ""                        tests/test_mission_edit.py
check "fence_rally"         ""                        tests/test_fence_rally.py
check "fence_shapes"        ""                        tests/test_fence_shapes.py
check "params"              ""                        tests/test_params.py
check "tlog"                ""                        tests/test_tlog.py
check "manual"              ""                        tests/test_manual.py
check "camera"              ""                        tests/test_camera.py
check "logs"                ""                        tests/test_logs.py
check "video"               ""                        tests/test_video.py
check "adsb"                ""                        tests/test_adsb.py
check "multivehicle"        ""                        tests/test_multivehicle.py
check "settings"            ""                        tests/test_settings.py
check "links_manager"       ""                        tests/test_links_manager.py
check "smoke (native)"      ""                        tests/smoke_gui.py
check "smoke (python)"      "DRONEDECK_FORCE_PYTHON=1" tests/smoke_gui.py

echo
echo "==== $pass passed, $fail failed ===="
exit $((fail > 0 ? 1 : 0))
