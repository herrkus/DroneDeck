"""Wind estimate: WIND_COV (231). Crafts a frame, checks the C++ core and the Python
parser decode it identically, and that Vehicle derives speed + 'from' bearing. The
meteorological 'from' convention is the reverse of the NED wind vector. Port-independent."""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "app"))

import mavlink
import core
from vehicle import Vehicle

# time_usec + wind_x/y/z + var_horiz/var_vert + wind_alt + horiz/vert accuracy (8 floats)
payload = struct.pack("<Q8f", 999, -3.0, 4.0, 0.5, 0.1, 0.2, 50.0, 0.3, 0.4)
frame = mavlink.frame(mavlink.WIND_COV, payload, seq=7, sysid=3, compid=1, crc_fn=mavlink.crc16_mcrf4xx)
cf = core.Parser().feed(frame)
pf = mavlink.PyParser().feed(frame)
assert len(cf) == 1 and len(pf) == 1, (len(cf), len(pf))
assert cf[0].fields == pf[0].fields, (cf[0].fields, pf[0].fields)   # C++/Python parity
d = cf[0].fields
assert abs(d["wind_x"] + 3.0) < 1e-4 and abs(d["wind_y"] - 4.0) < 1e-4 and abs(d["wind_alt"] - 50.0) < 1e-4

v = Vehicle()
v._on_wind_cov(d)
assert v.have_wind and abs(v.wind_speed() - 5.0) < 1e-4          # hypot(3, 4)

# 'from' bearings: air toward S -> from N (0); toward E -> from W (270); toward N -> from S (180)
for (wx, wy), expect in (((-3.0, 0.0), 0.0), ((0.0, 4.0), 270.0), ((5.0, 0.0), 180.0)):
    v.wind_x, v.wind_y = wx, wy
    assert abs(v.wind_dir() - expect) < 1e-3, (wx, wy, v.wind_dir())

print("WIND PASSED")
