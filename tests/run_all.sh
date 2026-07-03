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
# Orphaned test simulators are prevented at the source: sim/simulator.py registers PR_SET_PDEATHSIG
# so the kernel SIGTERMs it if its parent test dies (even on a timeout SIGKILL). --kill-after=5
# guarantees a test ignoring SIGTERM is still hard-killed, which in turn triggers that PDEATHSIG.
# (An earlier `pkill -f sim/simulator.py` reaper was removed: -f matches full command lines, so it
# could kill unrelated processes -- including this script's own subshells -- and broke sim tests.)
check() {
    local name="$1" env="$2" script="$3"
    local full
    full=$(env ${env} timeout --kill-after=5 60 python3 "$script" 2>&1)
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
check "parserfuzz (native)" ""                        tests/test_parserfuzz.py
check "parserfuzz (python)" "DRONEDECK_FORCE_PYTHON=1" tests/test_parserfuzz.py
check "parserfuzz2 (native)" ""                        tests/test_parserfuzz2.py
check "parserfuzz2 (python)" "DRONEDECK_FORCE_PYTHON=1" tests/test_parserfuzz2.py
check "links"               ""                        tests/test_links.py
check "mission"             ""                        tests/test_mission.py
check "mission_gui"         ""                        tests/test_mission_gui.py
check "mission_edit"        ""                        tests/test_mission_edit.py
check "waypointedit"         ""                        tests/test_waypointedit.py
check "setcurrentwp"        ""                        tests/test_setcurrentwp.py
check "fence_rally"         ""                        tests/test_fence_rally.py
check "fence_shapes"        ""                        tests/test_fence_shapes.py
check "params"              ""                        tests/test_params.py
check "tlog"                ""                        tests/test_tlog.py
check "tlogreplay"          ""                        tests/test_tlogreplay.py
check "analyze"             ""                        tests/test_analyze.py
check "planfile"            ""                        tests/test_planfile.py
check "planload"            ""                        tests/test_planload.py
check "mission_validate"    ""                        tests/test_mission_validate.py
check "upload_progress"     ""                        tests/test_upload_progress.py
check "wpprogress"          ""                        tests/test_wpprogress.py
check "missionreached"      ""                        tests/test_missionreached.py
check "missionrobust"       ""                        tests/test_missionrobust.py
check "scalemission"        ""                        tests/test_scalemission.py
check "paramset"            ""                        tests/test_paramset.py
check "reboot"              ""                        tests/test_reboot.py
check "parachute"           ""                        tests/test_parachute.py
check "linklifecycle"       ""                        tests/test_linklifecycle.py
check "sysidfilter"         ""                        tests/test_sysidfilter.py
check "spline"              ""                        tests/test_spline.py
check "terrain"             ""                        tests/test_terrain.py
check "odometer"            ""                        tests/test_odometer.py
check "ekf"                 ""                        tests/test_ekf.py
check "ekf (python)"        "DRONEDECK_FORCE_PYTHON=1" tests/test_ekf.py
check "wind"                ""                        tests/test_wind.py
check "vibration"           ""                        tests/test_vibration.py
check "telemfuzz"           ""                        tests/test_telemfuzz.py
check "widgetfuzz"          ""                        tests/test_widgetfuzz.py
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
check "mapfuzz"             ""                        tests/test_mapfuzz.py
check "cleartrail"          ""                        tests/test_cleartrail.py
check "scalebar"            ""                        tests/test_scalebar.py
check "fitmap"              ""                        tests/test_fitmap.py
check "ruler"               ""                        tests/test_ruler.py
check "battwarn"            ""                        tests/test_battwarn.py
check "msgunread"           ""                        tests/test_msgunread.py
check "msgclear"            ""                        tests/test_msgclear.py
check "takeoffalt"          ""                        tests/test_takeoffalt.py
check "mavversion"          ""                        tests/test_mavversion.py
check "paramfile"           ""                        tests/test_paramfile.py
check "paramdiff"           ""                        tests/test_paramdiff.py
check "paramrobust"         ""                        tests/test_paramrobust.py
check "cmdack"              ""                        tests/test_cmdack.py
check "streamreq"           ""                        tests/test_streamreq.py
check "packetloss"          ""                        tests/test_packetloss.py
check "missionack"          ""                        tests/test_missionack.py
check "px4warn"             ""                        tests/test_px4warn.py
check "about"               ""                        tests/test_about.py
check "gpxexport"           ""                        tests/test_gpxexport.py
check "exportfuzz"          ""                        tests/test_exportfuzz.py
check "copycoords"          ""                        tests/test_copycoords.py
check "confirm"             ""                        tests/test_confirm.py
check "windcompass"         ""                        tests/test_windcompass.py
check "homebug"             ""                        tests/test_homebug.py
check "manual"              ""                        tests/test_manual.py
check "joystick_hw"         ""                        tests/test_joystick_hw.py
check "command_retry"       ""                        tests/test_command_retry.py
check "telemetry_msgs"      ""                        tests/test_telemetry_msgs.py
check "gimbal_v2"           ""                        tests/test_gimbal_v2.py
check "fence_enable"        ""                        tests/test_fence_enable.py
check "camera_status"       ""                        tests/test_camera_status.py
check "joystick_buttons"    ""                        tests/test_joystick_buttons.py
check "accel_cal"           ""                        tests/test_accel_cal.py
check "mag_cal"             ""                        tests/test_mag_cal.py
check "gps_rtk"             ""                        tests/test_gps_rtk.py
check "rtcm_inject"         ""                        tests/test_rtcm_inject.py
check "terrain_report"      ""                        tests/test_terrain_report.py
check "camera_protocol"     ""                        tests/test_camera_protocol.py
check "ftp"                 ""                        tests/test_ftp.py
check "ftpclient"           ""                        tests/test_ftpclient.py
check "ftpbrowser"          ""                        tests/test_ftpbrowser.py
check "components"          ""                        tests/test_components.py
check "forward"             ""                        tests/test_forward.py
check "preflight"           ""                        tests/test_preflight.py
check "geotag"              ""                        tests/test_geotag.py
check "geotagdialog"        ""                        tests/test_geotagdialog.py
check "parammeta"           ""                        tests/test_parammeta.py
check "newmsgfuzz"          ""                        tests/test_newmsgfuzz.py
check "ftpcaps"             ""                        tests/test_ftpcaps.py
check "serial_link"         ""                        tests/test_serial_link.py
check "ardupilot_dialect"   ""                        tests/test_ardupilot_dialect.py
check "camera"              ""                        tests/test_camera.py
check "logs"                ""                        tests/test_logs.py
check "video"               ""                        tests/test_video.py
check "adsb"                ""                        tests/test_adsb.py
check "multivehicle"        ""                        tests/test_multivehicle.py
check "rc_calibration"      ""                        tests/test_rc_calibration.py
check "motortest"           ""                        tests/test_motortest.py
check "cameractl"           ""                        tests/test_cameractl.py
check "logrobust"           ""                        tests/test_logrobust.py
check "sensor_calibration"  ""                        tests/test_sensor_calibration.py
check "cellbars"            ""                        tests/test_cellbars.py
check "systems"             ""                        tests/test_systems.py
check "failsafe"            ""                        tests/test_failsafe.py
check "geofence"            ""                        tests/test_geofence.py
check "guided"              ""                        tests/test_guided.py
check "changeheading"       ""                        tests/test_changeheading.py
check "gripper"             ""                        tests/test_gripper.py
check "settings"            ""                        tests/test_settings.py
check "links_manager"       ""                        tests/test_links_manager.py
check "reconnect"           ""                        tests/test_reconnect.py
check "boundedgrowth"       ""                        tests/test_boundedgrowth.py
check "settingsload"        ""                        tests/test_settingsload.py
check "inputrobust"         ""                        tests/test_inputrobust.py
check "numericedge"         ""                        tests/test_numericedge.py
check "smoke (native)"      ""                        tests/smoke_gui.py
check "smoke (python)"      "DRONEDECK_FORCE_PYTHON=1" tests/smoke_gui.py

echo
echo "==== $pass passed, $fail failed ===="
exit $((fail > 0 ? 1 : 0))
