#!/usr/bin/env python3
"""The tow boat is often SYSID_THISMAV=2 so it can share a MAVLink net
with the towfish. Companion computers still occupy vehicle 1, so a
hardcoded vehicles/1/components/1 HEARTBEAT reads as "None" and the UI
reports no boat link even though the IP pings.

Run with: python3 test_boat_sysid.py
"""
from __future__ import annotations

import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "app"))
sys.modules.setdefault("requests", types.ModuleType("requests"))

from mavlink_params import BOAT_MAVTYPES, pick_autopilot

FAILURES: list[str] = []


def check(label, condition, detail=""):
    if condition:
        print(f"  ok   {label}")
    else:
        print(f"  FAIL {label} {detail}")
        FAILURES.append(label)


def heartbeat(autopilot, mavtype):
    return {
        "message": {
            "autopilot": {"type": autopilot},
            "mavtype": {"type": mavtype},
        }
    }


def component(*messages):
    return {"messages": {name: payload for name, payload in messages}}


# Shape taken from the live BlueBoat at 192.168.2.12: vehicle 1 is
# BlueOS, vehicle 2 is ArduRover, 255 is GCS traffic.
LIVE_BOAT = {
    "2": {
        "id": 2,
        "components": {
            "1": component(
                ("HEARTBEAT", heartbeat("MAV_AUTOPILOT_ARDUPILOTMEGA",
                                        "MAV_TYPE_SURFACE_BOAT")),
                ("GLOBAL_POSITION_INT", {"message": {}}),
            ),
            "194": {"id": 194, "messages": {}},
        },
    },
    "1": {
        "id": 1,
        "components": {
            "191": component(
                ("HEARTBEAT", heartbeat("MAV_AUTOPILOT_INVALID",
                                        "MAV_TYPE_ONBOARD_CONTROLLER")),
            ),
            "194": {"id": 194, "messages": {}},
        },
    },
    "255": {
        "id": 255,
        "components": {
            "190": {"id": 190, "messages": {}},
        },
    },
}


print("live BlueBoat tree")
picked = pick_autopilot(LIVE_BOAT, prefer_mavtypes=BOAT_MAVTYPES)
check("picks sysid 2 / component 1", picked == (2, 1), f"got {picked}")
check("ignores the onboard controller on vehicle 1",
      pick_autopilot({"1": LIVE_BOAT["1"]}) is None)


print("\nshared MAVLink network (boat + towfish both visible)")
shared = {
    "1": {
        "components": {
            "1": component(
                ("HEARTBEAT", heartbeat("MAV_AUTOPILOT_ARDUPILOTMEGA",
                                        "MAV_TYPE_SUBMARINE")),
            ),
        },
    },
    "2": LIVE_BOAT["2"],
}
check("prefers the surface boat over the sub",
      pick_autopilot(shared, prefer_mavtypes=BOAT_MAVTYPES) == (2, 1))
check("without a preference, first real autopilot is accepted",
      pick_autopilot(shared) == (1, 1))


print("\ndefaults and junk")
check("empty dict is None", pick_autopilot({}) is None)
check("None is None", pick_autopilot(None) is None)
classic = {
    "1": {
        "components": {
            "1": component(
                ("HEARTBEAT", heartbeat("MAV_AUTOPILOT_ARDUPILOTMEGA",
                                        "MAV_TYPE_SUBMARINE")),
            ),
        },
    },
}
check("classic sysid-1 autopilot still wins",
      pick_autopilot(classic) == (1, 1))


print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    raise SystemExit(1)
print("all checks passed")
