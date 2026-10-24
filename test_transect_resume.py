#!/usr/bin/env python3
"""Exercise the boot-time transect restore without a vehicle.

A fresh start must default per-leg capture to timelapse even when the
config still says video, and a monitor that was armed when the Pi lost
power must come back armed with the capture type it was running.

Run with: python3 test_transect_resume.py
"""
from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "app"))

FAILURES: list[str] = []


def check(label, condition, detail=""):
    print(f"  {'ok  ' if condition else 'FAIL'} {label}"
          + (f" {detail}" if detail and not condition else ""))
    if not condition:
        FAILURES.append(label)


def stub(name, **attrs):
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    sys.modules[name] = mod
    return mod


stub("flask", Flask=mock.MagicMock(), jsonify=mock.MagicMock(),
     request=mock.MagicMock(), send_file=mock.MagicMock())
stub("requests", get=mock.MagicMock(), post=mock.MagicMock(),
     exceptions=mock.MagicMock(),
     Session=mock.MagicMock(), adapters=mock.MagicMock())
gi = stub("gi", require_version=lambda *a, **k: None)
stub("gi.repository", Gst=mock.MagicMock())
gi.repository = sys.modules["gi.repository"]
stub("websockets", serve=mock.MagicMock())
stub("websockets.exceptions", ConnectionClosed=Exception)
sys.modules["websockets"].exceptions = sys.modules["websockets.exceptions"]
stub("usb_storage", get_base_dir=lambda: None, start_probe=lambda: None)
stub("photogrammetry_meta")

import main  # noqa: E402


def boot(armed, armed_capture, configured):
    main.transect_armed = armed
    main.transect_armed_capture_type = armed_capture
    main.transect_capture_type = configured
    main._transect_monitor = None
    main.mode = main.MODE_IDLE
    monitor = mock.MagicMock()
    with mock.patch.object(main, "TransectMonitor", return_value=monitor), \
         mock.patch.object(main, "_persist_config") as persist, \
         mock.patch.object(main, "_apply_awb_loop_state_change"), \
         mock.patch.object(main, "log_event"):
        main._restore_transect_after_restart()
    return monitor, persist


print("fresh start")
monitor, persist = boot(False, None, "video")
check("defaults capture to timelapse", main.transect_capture_type == "timelapse")
check("persists the reset", persist.called)
check("does not arm the monitor", not monitor.enable.called)

monitor, persist = boot(False, None, "timelapse")
check("already timelapse is left alone", not persist.called)

print("resume after a brownout")
monitor, persist = boot(True, "video", "timelapse")
check("re-arms the monitor", monitor.enable.called)
check("keeps the capture type it was running", main.transect_capture_type == "video")
check("stays armed", main.transect_armed is True)

monitor, persist = boot(True, None, "video")
check("armed without a recorded type falls back to timelapse",
      main.transect_capture_type == "timelapse" and monitor.enable.called)

print("busy recorder")
main.transect_armed = True
main.transect_armed_capture_type = "timelapse"
main.mode = main.MODE_VIDEO
main._transect_monitor = None
with mock.patch.object(main, "TransectMonitor") as tm, \
     mock.patch.object(main, "_persist_config"), \
     mock.patch.object(main, "log_event"):
    main._restore_transect_after_restart()
check("does not arm over a running manual recording", not tm.called)
main.mode = main.MODE_IDLE

print("operator arm/disarm bookkeeping")
main.transect_armed = False
main.transect_capture_type = "timelapse"
main._transect_monitor = None
with mock.patch.object(main, "TransectMonitor", return_value=mock.MagicMock()), \
     mock.patch.object(main, "_persist_config") as persist, \
     mock.patch.object(main, "_apply_awb_loop_state_change"):
    main._enable_transect_monitor()
check("enable marks the monitor armed", main.transect_armed is True)
check("enable records the capture type", main.transect_armed_capture_type == "timelapse")
check("enable persists", persist.called)

if FAILURES:
    print(f"\n{len(FAILURES)} failure(s)")
    sys.exit(1)
print("\nall checks passed")
