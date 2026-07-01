#!/usr/bin/env python3
"""test_msgunread.py -- unread-message badge. STATUSTEXT that arrives while the Messages dock
isn't the visible tab bumps an unread counter shown as 'Messages (N)' on the dock/tab title and
as a colour-coded '(+N)' on the status-strip MSG chip (red for error sev<=3, amber for warning
sev 4). Viewing the dock clears it. Pure UI, no link."""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import main as m

app = QApplication([])
win = m.DroneDeck(14599)
win.msg_dock.hide()                       # guarantee the "not viewing" state
assert not win.msg_dock.isVisible()

cap = {}
win.status_strip.set_chips = lambda chips: cap.update(chips=chips)
win.vehicle.last_heartbeat = time.monotonic()   # link_alive -> chips get built


def msgchip():
    return next((c for c in cap["chips"] if c[0].startswith("MSG ")), None)


# three messages arrive unseen; worst severity is the error (3)
win._on_new_message(6, "info")
win._on_new_message(3, "ERROR: something")
win._on_new_message(4, "warning")
assert win._msg_unread == 3, win._msg_unread
assert win._msg_worst == 3, win._msg_worst
assert win.msg_dock.windowTitle() == "Messages (3)", win.msg_dock.windowTitle()

# status chip shows (+3) in red (worst <= 3)
win._update_status_strip(win.vehicle, True, 0)
c = msgchip()
assert c and "(+3)" in c[0] and c[1] == "#e05050", c

# viewing the dock clears the badge
win._on_msg_visibility(True)
assert win._msg_unread == 0 and win._msg_worst == 99
assert win.msg_dock.windowTitle() == "Messages"
win._update_status_strip(win.vehicle, True, 0)
c = msgchip()
assert c and "(+" not in c[0] and c[1] == "#9aa0ac", c    # back to grey, no count

# a warning-only backlog colours the chip amber
win.msg_dock.hide()
win._on_new_message(4, "just a warning")
assert win._msg_worst == 4
win._update_status_strip(win.vehicle, True, 0)
c = msgchip()
assert c and c[1] == "#e0a030", c                          # amber for warning

# visibilityChanged(False) must NOT clear (only becoming visible does)
win._on_msg_visibility(False)
assert win._msg_unread == 1

print("MSGUNREAD PASSED")
