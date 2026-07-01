"""RC transmitter signal: RC_CHANNELS (65). Confirms the C++ core and Python parser decode
it identically, and that Vehicle maps rssi (0..254, 255=unknown) to a 0-100% figure.
Port-independent (no link)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "app"))

import mavlink
import core
from vehicle import Vehicle

payload = mavlink.enc_rc_channels([1500] * 18, rssi=200)
frame = mavlink.frame(mavlink.RC_CHANNELS, payload, seq=7, sysid=3, compid=1, crc_fn=mavlink.crc16_mcrf4xx)
cf = core.Parser().feed(frame)
pf = mavlink.PyParser().feed(frame)
assert len(cf) == 1 and len(pf) == 1, (len(cf), len(pf))
assert cf[0].fields == pf[0].fields, (cf[0].fields, pf[0].fields)   # C++/Python parity
assert cf[0].fields["rssi"] == 200 and cf[0].fields["chancount"] == 18

v = Vehicle()
v._on_rc_channels(cf[0].fields)
assert v.rc_rssi == 79                       # round(200 * 100 / 254)
v._on_rc_channels({"rssi": 255})
assert v.rc_rssi is None                     # 255 = not reporting
v._on_rc_channels({"rssi": 0})
assert v.rc_rssi == 0                         # 0 = no signal (distinct from unknown)
v._on_rc_channels({"rssi": 254})
assert v.rc_rssi == 100                       # full scale

print("RC PASSED")
