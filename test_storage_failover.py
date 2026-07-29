#!/usr/bin/env python3
"""Checks for the USB free-space gating that drives mid-recording failover.

The interesting property is that the two thresholds are independent: the
20 GB gate decides whether a drive is worth *starting* a survey on, while
the much smaller recording reserve decides when an *in-flight* session has
to move off. Getting those backwards either refuses usable drives or fails
over too late to finalise the file.

Only ``usb_storage`` is exercised here -- it is pure Python. The rest of
the failover path lives in ``app/main.py``, which needs GStreamer and so is
verified on the vehicle instead.

Run with: python3 test_storage_failover.py
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "app"))

import usb_storage

FAILURES: list[str] = []


def check(label, condition, detail=""):
    if condition:
        print(f"  ok   {label}")
    else:
        print(f"  FAIL {label} {detail}")
        FAILURES.append(label)


class FakeFree:
    """Pin usb_storage's reported free space and mount state."""

    def __init__(self, free_mb, mounted=True):
        self.free_mb = free_mb
        self.mounted = mounted

    def __enter__(self):
        self._free = usb_storage.get_free_mb
        self._mounted_flag = usb_storage._mounted
        usb_storage.get_free_mb = lambda: self.free_mb
        usb_storage._mounted = self.mounted
        return self

    def __exit__(self, *exc):
        usb_storage.get_free_mb = self._free
        usb_storage._mounted = self._mounted_flag


print("thresholds are ordered")
check("recording reserve is well below the start gate",
      usb_storage.USB_MIN_FREE_MB_RECORDING < usb_storage.USB_MIN_FREE_GB * 1024,
      f"reserve={usb_storage.USB_MIN_FREE_MB_RECORDING} MB "
      f"gate={usb_storage.USB_MIN_FREE_GB * 1024} MB")
check("reserve leaves room to finalise a file",
      usb_storage.USB_MIN_FREE_MB_RECORDING >= 512,
      f"got {usb_storage.USB_MIN_FREE_MB_RECORDING} MB")


print("\nstart gate (is_usable)")
gate_mb = usb_storage.USB_MIN_FREE_GB * 1024
for free, want in [(gate_mb + 1, True), (gate_mb, True), (gate_mb - 1, False),
                   (0, False)]:
    with FakeFree(free):
        check(f"is_usable({free} MB) is {want}", usb_storage.is_usable() is want)

with FakeFree(None):
    check("is_usable(unreadable) is False", usb_storage.is_usable() is False)


print("\nrecording reserve (has_recording_headroom)")
res = usb_storage.USB_MIN_FREE_MB_RECORDING
for free, want in [(gate_mb, True), (res + 1, True), (res, True),
                   (res - 1, False), (0, False)]:
    with FakeFree(free):
        check(f"has_recording_headroom({free} MB) is {want}",
              usb_storage.has_recording_headroom() is want)

with FakeFree(None):
    check("has_recording_headroom(unreadable) is False",
          usb_storage.has_recording_headroom() is False)


print("\nthe gap between the two thresholds is the working band")
# A drive in this band is too full to start a new survey on but still has
# plenty of room for a session already running -- it must NOT trigger a
# failover, or a long recording would be bounced to the SD card the moment
# it crossed the start gate.
mid = (res + gate_mb) // 2
with FakeFree(mid):
    check(f"{mid} MB: cannot start a new session",
          usb_storage.is_usable() is False)
    check(f"{mid} MB: an in-flight session keeps running",
          usb_storage.has_recording_headroom() is True)


print("\na full drive still looks healthy, which is why space is checked")
# is_healthy() only stats the mount, and statvfs succeeds on a 100%-full
# filesystem. This is the exact blind spot has_recording_headroom() covers.
with FakeFree(0):
    check("free=0 reports no headroom",
          usb_storage.has_recording_headroom() is False)
    check("free=0 is not 'usable' either", usb_storage.is_usable() is False)


print("\nstatus payload carries the new fields")
with FakeFree(res * 2):
    st = usb_storage.get_status()
    check("status has has_recording_headroom", "has_recording_headroom" in st)
    check("status has min_free_mb_recording", "min_free_mb_recording" in st)
    check("status reports the reserve value",
          st.get("min_free_mb_recording") == res)


print()
if FAILURES:
    print(f"{len(FAILURES)} FAILED: {FAILURES}")
    raise SystemExit(1)
print("all checks passed")
