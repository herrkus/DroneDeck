# DroneDeck

A QGroundControl-style **MAVLink ground control station**: live map, attitude
and heading instruments, and a telemetry sidebar, fed by a hand-optimized native
protocol core.

Built deliberately across three languages, each where it earns its place:

| Layer | Language | Job |
|-------|----------|-----|
| Hot path | **x86-64 assembly** (`core/crc_x25.S`) | The CRC-16/MCRF4XX checksum MAVLink runs over every byte of every frame. Slicing-by-8 breaks the serial dependency the compiler can't, beating `-O3` C by ~4x. |
| Engine | **C++** (`core/dronecore.cpp`) | Streaming MAVLink v1/v2 parser with CRC-gated resync, message decoders and encoders, exposed through a flat `extern "C"` ABI. |
| App | **Python / PySide6** (`app/`, `sim/`) | GUI, UDP link, vehicle model and the test telemetry source. Binds the native core via `ctypes`. |

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
  graticule), with the vehicle marker, flight trail and home point. Wheel to
  zoom, drag to pan, double-click to re-center, "Follow" to track.
- **Attitude indicator** -- artificial horizon with pitch ladder and bank scale.
- **Compass** -- heading card with digital readout.
- **Telemetry** -- link health + message rate, arm state, battery, GPS fix/sats,
  position, altitude, speeds, throttle, heading.
- **Commands** -- Arm / Disarm via `COMMAND_LONG` (and a 1 Hz GCS heartbeat).

Supported messages: `HEARTBEAT`, `SYS_STATUS`, `GPS_RAW_INT`, `ATTITUDE`,
`GLOBAL_POSITION_INT`, `VFR_HUD`, `COMMAND_LONG`.

---

## Layout

```
core/    crc_x25.S        x86-64 assembly CRC (slicing-by-8)
         crc_portable.c   portable-C CRC (used on non-x86-64 builds)
         dronecore.cpp    C++ parser/encoder + extern "C" ABI
app/     main.py          GCS entry point
         core.py          ctypes binding (+ pure-Python fallback selector)
         mavlink.py       message catalogue, encoders, pure-Python parser
         link.py          UDP link (QUdpSocket)
         vehicle.py       live state model
         instruments.py   attitude + compass widgets
         mapview.py       tile map widget
         panels.py        telemetry readouts
sim/     simulator.py     TEST telemetry source (flies a circle; not a drone model)
tests/   selftest.cpp     native CRC/parser checks + benchmark
         test_parity.py   Python-encode -> C++-decode parity
         smoke_gui.py     headless end-to-end + screenshot
build.sh   run.sh
```

---

## Connecting a real drone

Point the autopilot's telemetry at this machine's UDP **14550** (the MAVLink GCS
convention) -- e.g. a SiK radio bridged with `mavlink-routerd`, an ESP/UDP
telemetry link, or ArduPilot/PX4 SITL's UDP out. The link learns the drone's
address from its first packet and starts sending the GCS heartbeat back.
Serial links can be bridged to UDP with `mavlink-router` or `mavproxy`.

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
python3 tests/test_parity.py    # cross-language: Python-encode -> C++/asm-decode parity
python3 tests/smoke_gui.py      # headless end-to-end with the simulator + screenshot
DRONEDECK_FORCE_PYTHON=1 python3 tests/smoke_gui.py   # same, exercising the fallback
```

The assembly CRC is gated by the self-test (`asm vs C reference over lengths
0..4096: match`) -- it is only ever used after proving it matches the portable
reference bit-for-bit.
