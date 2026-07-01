"""Advisory pre-upload mission checks (mission.validate_mission). Pure function,
port-independent, so it is always safe to run."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "app"))

from mission import MissionItem, validate_mission


def wp(seq, cmd, alt=50.0, p1=0.0):
    return MissionItem(seq, 47.0, 8.0, alt, command=cmd, param1=p1)


# a clean mission: takeoff, waypoints, land -> no warnings
clean = [wp(0, 22, 25.0), wp(1, 16, 50.0), wp(2, 16, 50.0), wp(3, 21, 0.0)]
assert validate_mission(clean) == [], validate_mission(clean)

# missing takeoff
assert any("Takeoff" in x for x in validate_mission([wp(0, 16, 50.0), wp(1, 21, 0.0)]))

# missing land/RTL at the end
assert any("Land or Return" in x for x in validate_mission([wp(0, 22, 25.0), wp(1, 16, 50.0)]))

# DO_JUMP target out of range
bad_jump = [wp(0, 22, 25.0), wp(1, 16, 50.0), MissionItem(2, 0, 0, 0, command=177, param1=9)]
assert any("out of range" in x for x in validate_mission(bad_jump))

# DO_JUMP in range -> no jump warning
ok_jump = [wp(0, 22, 25.0), wp(1, 16, 50.0),
           MissionItem(2, 0, 0, 0, command=177, param1=1), wp(3, 21, 0.0)]
assert not any("out of range" in x for x in validate_mission(ok_jump))

# zero-altitude nav waypoint (but a Land at 0 is fine)
assert any("altitude is 0" in x for x in validate_mission([wp(0, 22, 25.0), wp(1, 16, 0.0), wp(2, 21, 0.0)]))

# large altitude jump between airborne waypoints
big = [wp(0, 22, 20.0), wp(1, 16, 20.0), wp(2, 16, 200.0), wp(3, 21, 0.0)]
assert any("large altitude" in x for x in validate_mission(big))

# descending to land is NOT flagged as a big jump (the 150 m -> 0 land drop is excluded)
descend = [wp(0, 22, 150.0), wp(1, 16, 150.0), wp(2, 21, 0.0)]
assert not any("large altitude" in x for x in validate_mission(descend))

# a DO item before takeoff still recognises takeoff as the first nav waypoint
lead_do = [MissionItem(0, 0, 0, 0, command=178, param1=1, param2=8), wp(1, 22, 25.0), wp(2, 21, 0.0)]
assert not any("Takeoff" in x for x in validate_mission(lead_do))

# empty mission -> nothing to warn about
assert validate_mission([]) == []

print("MISSION VALIDATE PASSED")
