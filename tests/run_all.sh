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
check "analyze"             ""                        tests/test_analyze.py
check "planfile"            ""                        tests/test_planfile.py
check "mission_validate"    ""                        tests/test_mission_validate.py
check "upload_progress"     ""                        tests/test_upload_progress.py
check "wpprogress"          ""                        tests/test_wpprogress.py
check "spline"              ""                        tests/test_spline.py
check "terrain"             ""                        tests/test_terrain.py
check "odometer"            ""                        tests/test_odometer.py
check "ekf"                 ""                        tests/test_ekf.py
check "ekf (python)"        "DRONEDECK_FORCE_PYTHON=1" tests/test_ekf.py
check "wind"                ""                        tests/test_wind.py
check "wind (python)"       "DRONEDECK_FORCE_PYTHON=1" tests/test_wind.py
check "rc"                  ""                        tests/test_rc.py
check "rc (python)"         "DRONEDECK_FORCE_PYTHON=1" tests/test_rc.py
check "gimbal"              ""                        tests/test_gimbal.py
check "gimbal (python)"     "DRONEDECK_FORCE_PYTHON=1" tests/test_gimbal.py
check "servo"               ""                        tests/test_servo.py
check "servo (python)"      "DRONEDECK_FORCE_PYTHON=1" tests/test_servo.py
check "home"                ""                        tests/test_home.py
check "home (python)"       "DRONEDECK_FORCE_PYTHON=1" tests/test_home.py
check "esc"                 ""                        tests/test_esc.py
check "esc (python)"        "DRONEDECK_FORCE_PYTHON=1" tests/test_esc.py
check "extstate"            ""                        tests/test_extstate.py
check "extstate (python)"   "DRONEDECK_FORCE_PYTHON=1" tests/test_extstate.py
check "tett"                ""                        tests/test_tett.py
check "tett (python)"       "DRONEDECK_FORCE_PYTHON=1" tests/test_tett.py
check "gnss"                ""                        tests/test_gnss.py
check "gnss (python)"       "DRONEDECK_FORCE_PYTHON=1" tests/test_gnss.py
check "autopilot_version"   ""                        tests/test_autopilot_version.py
check "autopilot_v (py)"    "DRONEDECK_FORCE_PYTHON=1" tests/test_autopilot_version.py
check "flightchip"          ""                        tests/test_flightchip.py
check "mapcenter"           ""                        tests/test_mapcenter.py
check "confirm"             ""                        tests/test_confirm.py
check "windcompass"         ""                        tests/test_windcompass.py
check "manual"              ""                        tests/test_manual.py
check "camera"              ""                        tests/test_camera.py
check "logs"                ""                        tests/test_logs.py
check "video"               ""                        tests/test_video.py
check "adsb"                ""                        tests/test_adsb.py
check "multivehicle"        ""                        tests/test_multivehicle.py
check "rc_calibration"      ""                        tests/test_rc_calibration.py
check "sensor_calibration"  ""                        tests/test_sensor_calibration.py
check "cellbars"            ""                        tests/test_cellbars.py
check "systems"             ""                        tests/test_systems.py
check "failsafe"            ""                        tests/test_failsafe.py
check "geofence"            ""                        tests/test_geofence.py
check "guided"              ""                        tests/test_guided.py
check "settings"            ""                        tests/test_settings.py
check "links_manager"       ""                        tests/test_links_manager.py
check "smoke (native)"      ""                        tests/smoke_gui.py
check "smoke (python)"      "DRONEDECK_FORCE_PYTHON=1" tests/smoke_gui.py

echo
echo "==== $pass passed, $fail failed ===="
exit $((fail > 0 ? 1 : 0))
