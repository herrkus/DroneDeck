#!/usr/bin/env python3
"""test_msgclear.py -- 'Clear' button on the Messages panel. Empties the MessageConsole and
resets the unread badge (count + worst severity + dock title). Pure UI, no link."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "app"))

from PySide6.QtWidgets import QApplication
import main as m

app = QApplication([])
win = m.DroneDeck(14599)
win.msg_dock.hide()                      # so _on_new_message counts them as unread

for sev, txt in [(6, "info"), (3, "ERROR: bad"), (4, "warning")]:
    win.console.add_message(sev, txt)
    win._on_new_message(sev, txt)
win.console.add_note("a local note")     # notes are cleared too
assert win.console.count() == 4
assert win._msg_unread == 3 and win._msg_worst == 3
assert win.msg_dock.windowTitle() == "Messages (3)"

# Clear empties the log and resets the unread badge
win._clear_messages()
assert win.console.count() == 0
assert win._msg_unread == 0 and win._msg_worst == 99
assert win.msg_dock.windowTitle() == "Messages"
assert win.btn_msg_clear is not None

# further messages after a clear still count normally (state wasn't left broken)
win.console.add_message(4, "post-clear warning")
win._on_new_message(4, "post-clear warning")
assert win.console.count() == 1 and win._msg_unread == 1

print("MSGCLEAR PASSED")
