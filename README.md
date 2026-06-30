# DroneDeck

A QGroundControl-style **MAVLink ground control station**: live map, attitude
and heading instruments, a telemetry sidebar, flight commands and mission
planning -- over UDP, TCP or serial -- fed by a hand-optimized native protocol
core.

Built deliberately across three languages, each where it earns its place:

| Layer | Language | Job |
|-------|----------|-----|
| Hot path | **x86-64 assembly** (`core/crc_x25.S`) | The CRC-16/MCRF4XX checksum MAVLink runs over every byte of every frame. Slicing-by-8 breaks the serial dependency the compiler can't, beating `-O3` C by ~4x. |
| Engine | **C++** (`core/dronecore.cpp`) | Streaming MAVLink v1/v2 parser with CRC-gated resync, message decoders and encoders, exposed through a flat `extern "C"` ABI. |
| App | **Python / PySide6** (`app/`, `sim/`) | GUI, UDP/TCP/serial links, vehicle model, mission protocol and the test telemetry source. Binds the native core via `ctypes`. |

> No 3D/vehicle model is included by design -- the map uses a simple heading
> marker. Connect your own drone (or model) when ready.

---

## Quick start

```bash
./build.sh        # build + self-test the native core (asm + C++)
./run.sh demo     # launch the GCS AND a test telemetry source -> live data, no hardware
```

Just the ground station (to connect a real drone or your own simulator):

```bash
./run.sh          # listens for MAVLink on UDP :14550
./run.sh 14551    # ... on a different port
```

Requires `python3`, `PySide6` (`sudo pacman -S pyside6` on Arch), and `g++` for
the native core.

---

## What you get

- **Map** -- OpenStreetMap tiles (disk-cached; offline falls back to a lat/lon
  graticule), with the vehicle marker, flight trail, home point and the planned
  mission path. Wheel to zoom, drag to pan, double-click to re-center, "Follow"
  to track.
- **Attitude indicator** -- artificial horizon with pitch ladder and bank scale.
- **Compass** -- heading card with digital readout.
- **Telemetry** -- link health + message rate, arm state, flight mode, battery,
  GPS fix/sats, position, altitude, speeds, throttle, heading.
- **Links** -- connect over **UDP**, **TCP** or **serial** (USB / SiK radio),
  chosen from the toolbar; a 1 Hz GCS heartbeat goes back on all of them.
- **Safety feedback** -- live flight mode, a colour-coded `STATUSTEXT` console,
  and per-command `COMMAND_ACK` results.
- **Flying controls** -- set flight mode (Loiter/Auto/Guided/RTL/...), plus
  Takeoff, Land, Return-to-Launch, Pause, and **click-on-map to fly there**
  (guided goto).
- **Mission planning** -- plan-mode map clicks drop numbered waypoints; upload /
  download missions over the standard MAVLink mission protocol; one-click
  **survey** grid over a planned area; Clear.

Supported messages: `HEARTBEAT`, `SYS_STATUS`, `GPS_RAW_INT`, `ATTITUDE`,
`GLOBAL_POSITION_INT`, `VFR_HUD`, `COMMAND_LONG`, `COMMAND_ACK`, `STATUSTEXT`,
`SET_POSITION_TARGET_GLOBAL_INT`, and the mission set (`MISSION_COUNT`,
`MISSION_ITEM_INT`, `MISSION_REQUEST_INT`, `MISSION_REQUEST_LIST`,
`MISSION_ACK`, `MISSION_CLEAR_ALL`, `MISSION_CURRENT`, `MISSION_ITEM_REACHED`).

---

## Layout

```
core/    crc_x25.S        x86-64 assembly CRC (slicing-by-8)
         crc_portable.c   portable-C CRC (used on non-x86-64 builds)
         dronecore.cpp    C++ parser/encoder + extern "C" ABI
app/     main.py          GCS entry point
         core.py          ctypes binding (+ pure-Python fallback selector)
         mavlink.py       message catalogue, encoders, pure-Python parser
         link.py          UDP / TCP / serial links (common Link base)
         mission.py       mission upload/download protocol + survey grids
         vehicle.py       live state model
         instruments.py   attitude + compass widgets
         mapview.py       tile map widget (+ mission path, click-to-plan)
         panels.py        telemetry readouts + message console
sim/     simulator.py     TEST telemetry source (flies + obeys commands; not a drone model)
tests/   selftest.cpp     native CRC/parser checks + benchmark
         crc_extra_calc.py CRC_EXTRA derivation, validated vs known messages
         test_parity.py   Python-encode -> C++-decode parity (incl. missions)
         test_links.py    TCP + serial transports end to end
         test_mission.py  mission upload/download round-trip vs the simulator
         test_mission_gui.py  plan -> upload -> download through the real window
         smoke_gui.py     headless end-to-end + screenshot
build.sh   run.sh
```

---

## Connecting a real drone

Pick the transport in the toolbar's **Link** selector:

- **UDP** (default, port `14550`): the MAVLink GCS convention -- a SiK radio
  bridged with `mavlink-routerd`, an ESP/UDP link, or ArduPilot/PX4 SITL's UDP
  out. The link learns the drone's address from its first packet.
- **TCP** (`host:port`, e.g. `127.0.0.1:5760`): SITL's TCP server, or a bridge.
- **Serial** (`port:baud`, e.g. `/dev/ttyACM0:57600`): a USB autopilot or SiK
  radio directly -- available ports are pre-filled.

A 1 Hz GCS heartbeat is sent back on whichever link is active. Modern
ArduPilot/PX4 use the `_INT` mission messages this GCS speaks.

---

## Sharing this with others / other operating systems

The GUI is pure Python + Qt, so it runs on **Linux, Windows and macOS**. The
catch is the *optimized native core*, which is compiled and CPU/OS-specific.
The app handles this automatically:

- **It always runs.** If `libdronecore.so` can't be loaded (wrong OS, wrong CPU,
  or not built yet), the app prints one line and switches to an identical
  **pure-Python parser**. The status bar shows which core is live. So anyone
  with Python + PySide6 can run it with zero compilation.
- **For the fast core on their machine**, they rebuild it once:
  - **Linux / macOS, x86-64**: `./build.sh` -> uses the hand-written assembly.
  - **ARM (Apple Silicon, Raspberry Pi)**: `./build.sh` -> automatically uses
    `crc_portable.c` instead (the x86-64 asm doesn't apply); same results.
  - **Windows**: rebuild `dronecore.cpp` (+ `crc_portable.c`) into a
    `dronecore.dll` with MSVC/MinGW. The x86-64 assembly uses the System V
    calling convention, so it is **not** used on Windows -- the portable C path
    is. (`core.py`'s loader would need the `.dll` name added.)
- **Single-file bundle**: `pyinstaller --onefile app/main.py` (bundle the built
  core alongside, or rely on the Python fallback) produces a self-contained
  executable per OS.

In short: **yes -- anyone can run it.** They get the assembly-accelerated core on
Linux x86-64 out of the box, an automatic portable-C core on other Unix CPUs
after `./build.sh`, and the pure-Python fallback everywhere else with no build at
all.

---

## Tests

```bash
./build.sh                      # native self-test: known CRC vector, asm==C, parser, benchmark
python3 tests/crc_extra_calc.py # derive + validate every CRC_EXTRA seed
python3 tests/test_parity.py    # cross-language: Python-encode -> C++/asm-decode parity
python3 tests/test_links.py     # TCP + serial transports end to end
python3 tests/test_mission.py   # mission upload/download round-trip vs the simulator
python3 tests/test_mission_gui.py  # plan -> upload -> download through the real window
python3 tests/smoke_gui.py      # headless end-to-end with the simulator + screenshot
DRONEDECK_FORCE_PYTHON=1 python3 tests/smoke_gui.py   # same, exercising the fallback
```

The assembly CRC is gated by the self-test (`asm vs C reference over lengths
0..4096: match`) -- it is only ever used after proving it matches the portable
reference bit-for-bit.
