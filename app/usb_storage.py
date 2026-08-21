"""
USB external storage detection, mounting, and health monitoring for Towfish.

Scans for removable block devices, mounts the first usable partition, and
exposes state for the rest of the application.  A background probe thread
periodically checks for newly-inserted USB drives when the system is idle.

Ported from the dropcam branch's ``usb_storage.py`` -- only the on-disk
subfolder name changes (``DropCam`` -> ``Towfish``); the mount logic,
health checks, and free-space gating are identical so the failure modes
match what we already validated on dropcam hardware.
"""

import glob
import logging
import os
import subprocess
import threading
import time

logger = logging.getLogger(__name__)

USB_MOUNT_POINT = "/mnt/usb"
USB_MIN_FREE_GB = 20
TOWFISH_DIR = "Towfish"
PROBE_INTERVAL_S = 30

#: Free space below which an *already running* recording must move off the
#: drive. Deliberately far smaller than the ``USB_MIN_FREE_GB`` gate used to
#: pick a drive at session start: that one is a "is this stick worth starting
#: a survey on" question, this one is "how much runway do we need to land the
#: current session cleanly". The reserve has to cover the health-watcher poll
#: interval, whatever the encoder still has buffered, and -- the expensive
#: part -- mp4 finalisation, since a moov atom that cannot be written leaves
#: an unplayable file. 1 GiB is a few minutes of 4K H.264 and under 5% of the
#: smallest stick that can pass the start gate, so it is cheap insurance.
USB_MIN_FREE_MB_RECORDING = 1024

_lock = threading.Lock()
_mounted = False
_device = None          # e.g. "/dev/sda1"
_fstype = None          # e.g. "vfat"
_probe_thread = None
_stop_probe = threading.Event()


# ── Detection ────────────────────────────────────────────────────────────

def _scan_usb_devices():
    """Return a list of partition device paths on removable block devices.

    Only returns actual partitions (e.g. /dev/sda1).  Whole-disk devices
    without a partition table are skipped on purpose: mounting them blocks
    the kernel filesystem probe and can hang `mount` indefinitely.
    """
    partitions = []
    for block in sorted(glob.glob("/sys/block/sd*")):
        try:
            with open(os.path.join(block, "removable"), "r") as f:
                if f.read().strip() != "1":
                    continue
        except Exception:
            continue
        dev_name = os.path.basename(block)
        found_any = False
        for part in sorted(glob.glob(os.path.join(block, dev_name + "[0-9]*"))):
            part_name = os.path.basename(part)
            dev_path = f"/dev/{part_name}"
            if os.path.exists(dev_path):
                partitions.append(dev_path)
                found_any = True
        if not found_any:
            logger.debug(
                f"USB block {dev_name} has no partitions; skipping whole-disk mount "
                "(raw device without a partition table cannot be mounted safely)"
            )
    return partitions


# ── Mount / unmount ──────────────────────────────────────────────────────

def _detect_fstype(dev):
    """Return the filesystem type of a block device, or '' if unknown."""
    try:
        r = subprocess.run(
            ["blkid", "-o", "value", "-s", "TYPE", dev],
            capture_output=True, timeout=5, text=True,
        )
        if r.returncode == 0:
            return r.stdout.strip().lower()
    except Exception as e:
        logger.debug(f"blkid {dev} failed: {e}")
    return ""


def _mount_commands(dev, fstype):
    """Build an ordered list of mount argv attempts for a device.

    Prefer the in-kernel drivers and bypass userspace ``/sbin/mount.<type>``
    helpers (``-i``).  The bundled FUSE exfat helper (exfat-fuse) hangs
    uninterruptibly on some drives/controllers, wedging the mount in D state;
    the kernel exfat driver (auto-loaded on demand) is fast and reliable.
    """
    attempts = []
    if fstype in ("exfat", "vfat", "msdos", "fat"):
        attempts.append(["mount", "-i", "-t", fstype, "-o", "rw", dev, USB_MOUNT_POINT])
    elif fstype == "ntfs":
        # Kernel ntfs3 (read/write since 5.15) avoids the ntfs-3g FUSE helper.
        attempts.append(["mount", "-i", "-t", "ntfs3", "-o", "rw", dev, USB_MOUNT_POINT])
        attempts.append(["mount", "-i", "-t", "ntfs", "-o", "rw", dev, USB_MOUNT_POINT])
    elif fstype:
        attempts.append(["mount", "-i", "-t", fstype, "-o", "rw", dev, USB_MOUNT_POINT])
    # Final fallback: let mount auto-detect (covers ext4 and anything missed).
    attempts.append(["mount", "-o", "rw", dev, USB_MOUNT_POINT])
    return attempts


def is_mounted():
    """Check whether USB_MOUNT_POINT is an active mount."""
    try:
        with open("/proc/mounts", "r") as f:
            for line in f:
                if USB_MOUNT_POINT in line.split():
                    return True
    except Exception:
        pass
    return False


def _read_mount_fstype():
    """Return the filesystem type currently mounted at USB_MOUNT_POINT, or ''."""
    try:
        with open("/proc/mounts", "r") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 3 and parts[1] == USB_MOUNT_POINT:
                    return parts[2].lower()
    except Exception:
        pass
    return ""


def try_mount():
    """Detect and mount the first usable USB partition.  Returns True on success."""
    global _mounted, _device, _fstype

    with _lock:
        if _mounted and is_mounted():
            return True

        partitions = _scan_usb_devices()
        if not partitions:
            _mounted = False
            _device = None
            _fstype = None
            return False

        os.makedirs(USB_MOUNT_POINT, exist_ok=True)

        if is_mounted():
            _mounted = True
            _device = _device or partitions[0]
            _fstype = _read_mount_fstype() or _fstype
            return True

        for dev in partitions:
            fstype = _detect_fstype(dev)
            mounted_ok = False
            for cmd in _mount_commands(dev, fstype):
                try:
                    result = subprocess.run(cmd, capture_output=True, timeout=10)
                except subprocess.TimeoutExpired:
                    logger.warning(
                        f"mount {dev} ({fstype or 'auto'}) via {' '.join(cmd)} timed out "
                        "after 10s; device may be unresponsive or the filesystem "
                        "driver hung. Skipping this attempt."
                    )
                    continue
                except Exception as e:
                    logger.warning(f"mount {dev} raised: {e}; trying next method")
                    continue
                if result.returncode == 0:
                    _mounted = True
                    _device = dev
                    # Trust /proc/mounts for the actual fstype the kernel
                    # ended up using (auto-detect can pick something other
                    # than the blkid label when both ntfs and ntfs3 exist).
                    _fstype = _read_mount_fstype() or fstype or ""
                    logger.info(f"USB mounted: {dev} ({_fstype or 'auto'}) -> {USB_MOUNT_POINT}")
                    mounted_ok = True
                    break
                logger.debug(
                    f"mount {dev} via {' '.join(cmd)} failed: "
                    f"{result.stderr.decode(errors='replace').strip()}"
                )
            if mounted_ok:
                return True

        _mounted = False
        _device = None
        _fstype = None
        return False


def unmount():
    """Unmount USB storage if mounted."""
    with _lock:
        _unmount_unlocked()


def _unmount_unlocked():
    """Unmount USB_MOUNT_POINT. Caller must hold ``_lock``.

    Returns ``(ok, message)``. A failed umount leaves ``_mounted`` alone
    so the rest of the app still knows the drive is attached; a success
    (or "already unmounted") clears the in-memory mount state.
    """
    global _mounted, _device, _fstype
    if not is_mounted():
        _mounted = False
        _device = None
        _fstype = None
        return True, ""
    try:
        result = subprocess.run(
            ["umount", USB_MOUNT_POINT], capture_output=True, timeout=15,
        )
    except subprocess.TimeoutExpired:
        logger.warning("umount timed out after 15s; leaving state stale")
        return False, "umount timed out"
    except Exception as e:
        logger.warning(f"umount raised: {e}")
        return False, str(e)
    if result.returncode != 0:
        err = result.stderr.decode(errors="replace").strip() or "umount failed"
        logger.warning(f"umount failed: {err}")
        return False, err
    logger.info("USB unmounted")
    _mounted = False
    _device = None
    _fstype = None
    return True, ""


def _read_label(dev):
    """Return the volume label of a block device, or '' if unknown."""
    try:
        r = subprocess.run(
            ["blkid", "-o", "value", "-s", "LABEL", dev],
            capture_output=True, timeout=5, text=True,
        )
        if r.returncode == 0:
            return r.stdout.strip()
    except Exception as e:
        logger.debug(f"blkid LABEL {dev} failed: {e}")
    return ""


def mkfs_command(dev, fstype, label):
    """Build the mkfs argv that wipes ``dev`` as ``fstype``.

    Volume-label length is clipped to what each formatter accepts.
    An empty label becomes ``TOWFISH`` so a wiped survey stick still
    identifies itself in the host's file manager.
    """
    fstype = (fstype or "").lower()
    name = (label or "").strip() or "TOWFISH"
    # Keep labels to the portable FAT/exFAT subset so a name that was
    # legal on NTFS cannot make mkfs.exfat / mkfs.vfat reject the call.
    safe = "".join(ch if ch.isalnum() or ch in " -_" else "_" for ch in name)
    safe = safe.strip(" _") or "TOWFISH"
    if fstype in ("vfat", "msdos", "fat"):
        return ["mkfs.vfat", "-F", "32", "-n", safe[:11].upper(), dev]
    if fstype == "ntfs":
        return ["mkfs.ntfs", "-Q", "-F", "-L", safe[:32], dev]
    # exfat, unknown, or anything else we can remount with the kernel
    # exfat driver after a format. Survey sticks in this project are
    # exFAT; falling back to it is the least-wrong wipe of a mystery FS.
    return ["mkfs.exfat", "-n", safe[:15], dev]


def _remount_unlocked(dev, fstype):
    """Mount ``dev`` at USB_MOUNT_POINT. Caller must hold ``_lock``.

    Returns True on success and updates ``_mounted`` / ``_device`` /
    ``_fstype``. Mirrors the attempt loop in :func:`try_mount`.
    """
    global _mounted, _device, _fstype
    os.makedirs(USB_MOUNT_POINT, exist_ok=True)
    for cmd in _mount_commands(dev, fstype):
        try:
            result = subprocess.run(cmd, capture_output=True, timeout=10)
        except Exception as e:
            logger.warning(f"remount {dev} via {' '.join(cmd)} raised: {e}")
            continue
        if result.returncode == 0:
            _mounted = True
            _device = dev
            _fstype = _read_mount_fstype() or fstype or ""
            logger.info(f"USB remounted: {dev} ({_fstype or 'auto'}) -> {USB_MOUNT_POINT}")
            return True
        logger.debug(
            f"remount {dev} via {' '.join(cmd)} failed: "
            f"{result.stderr.decode(errors='replace').strip()}"
        )
    _mounted = False
    _device = None
    _fstype = None
    return False


def wipe():
    """Erase the currently attached USB partition and remount it empty.

    Unmounts, recreates the filesystem (same type, preserving the volume
    label when blkid can read one), remounts, and recreates the Towfish
    folder. The probe lock is held for the whole operation so a hot-plug
    scan cannot remount the old filesystem between umount and mkfs.

    Returns ``{"ok": bool, "message": str, "status": dict}``.
    """
    with _lock:
        partitions = _scan_usb_devices()
        if not partitions:
            return {
                "ok": False,
                "message": "No USB drive detected",
                "status": get_status(),
            }

        # Prefer the partition we already had mounted; fall back to the
        # first removable partition the kernel can see.
        dev = _device if _device in partitions else partitions[0]
        fstype = (_fstype or _detect_fstype(dev) or "exfat").lower()
        label = _read_label(dev) or "TOWFISH"
        cmd = mkfs_command(dev, fstype, label)

        ok, err = _unmount_unlocked()
        if not ok:
            return {
                "ok": False,
                "message": f"Could not unmount USB: {err}",
                "status": get_status(),
            }

        try:
            result = subprocess.run(cmd, capture_output=True, timeout=120)
        except subprocess.TimeoutExpired:
            _remount_unlocked(dev, fstype)
            return {
                "ok": False,
                "message": f"Format of {dev} timed out after 120s",
                "status": get_status(),
            }
        except Exception as e:
            _remount_unlocked(dev, fstype)
            return {
                "ok": False,
                "message": f"Format of {dev} raised: {e}",
                "status": get_status(),
            }
        if result.returncode != 0:
            err = result.stderr.decode(errors="replace").strip() or "mkfs failed"
            logger.warning(f"wipe mkfs failed ({' '.join(cmd)}): {err}")
            _remount_unlocked(dev, fstype)
            return {
                "ok": False,
                "message": f"Format failed: {err}",
                "status": get_status(),
            }

        if not _remount_unlocked(dev, fstype):
            return {
                "ok": False,
                "message": f"Formatted {dev} but remount failed",
                "status": get_status(),
            }

        try:
            os.makedirs(os.path.join(USB_MOUNT_POINT, TOWFISH_DIR), exist_ok=True)
        except Exception as e:
            logger.warning(f"Could not recreate {TOWFISH_DIR} after wipe: {e}")

        logger.info(f"USB wiped: {dev} ({fstype}) label={label}")
        return {
            "ok": True,
            "message": f"Wiped {dev} ({_fstype or fstype})",
            "status": get_status(),
        }


# ── Health / space ───────────────────────────────────────────────────────

def is_healthy():
    """Fast health check: can we stat the mount point?"""
    if not _mounted:
        return False
    try:
        os.statvfs(USB_MOUNT_POINT)
        return True
    except Exception:
        return False


def get_free_mb():
    """Return free space in MB on the USB mount, or None if unavailable."""
    if not _mounted:
        return None
    try:
        st = os.statvfs(USB_MOUNT_POINT)
        return round((st.f_bavail * st.f_frsize) / (1024 * 1024), 1)
    except Exception:
        return None


def is_usable():
    """USB is mounted and has at least USB_MIN_FREE_GB free."""
    free = get_free_mb()
    if free is None:
        return False
    return free >= USB_MIN_FREE_GB * 1024


def has_recording_headroom():
    """Is there still room to keep an in-flight recording on this drive?

    Separate from :func:`is_healthy`, which only answers "does the mount
    still respond" -- a filesystem that is 100% full stats perfectly well,
    so health alone never notices a drive filling up mid-mission. Returns
    False once free space drops under ``USB_MIN_FREE_MB_RECORDING``, which
    is the signal to move the session to local storage while there is still
    room to finalise the current file.
    """
    free = get_free_mb()
    if free is None:
        return False
    return free >= USB_MIN_FREE_MB_RECORDING


def get_fstype():
    """Return the filesystem type of the active USB mount, or '' if unknown."""
    return (_fstype or "").lower()


def is_fat_like():
    """True if the mounted filesystem has the FAT 4 GiB per-file cap."""
    return get_fstype() in ("vfat", "msdos", "fat")


def get_recording_dir(subfolder_name):
    """Return the full path for a recording subfolder on USB, creating it."""
    base = os.path.join(USB_MOUNT_POINT, TOWFISH_DIR, subfolder_name)
    os.makedirs(base, exist_ok=True)
    return base


def get_base_dir():
    """Return the Towfish root on the USB mount, or None if not mounted."""
    if not (_mounted and is_mounted()):
        return None
    return os.path.join(USB_MOUNT_POINT, TOWFISH_DIR)


def get_status():
    """Return a status dict for the API."""
    mounted = _mounted and is_mounted()
    free = get_free_mb() if mounted else None
    return {
        "mounted": mounted,
        "device": _device,
        "fstype": _fstype if mounted else None,
        "free_mb": free,
        "usable": is_usable() if mounted else False,
        "has_recording_headroom": has_recording_headroom() if mounted else False,
        "mount_point": USB_MOUNT_POINT,
        "min_free_gb": USB_MIN_FREE_GB,
        "min_free_mb_recording": USB_MIN_FREE_MB_RECORDING,
    }


# ── Background probe ────────────────────────────────────────────────────

def _probe_loop():
    """Periodically scan and mount USB when idle."""
    while not _stop_probe.is_set():
        if not (_mounted and is_mounted()):
            try:
                try_mount()
            except Exception as e:
                logger.debug(f"USB probe error: {e}")
        _stop_probe.wait(PROBE_INTERVAL_S)


def start_probe():
    """Start the background USB probe thread."""
    global _probe_thread
    if _probe_thread and _probe_thread.is_alive():
        return
    _stop_probe.clear()
    _probe_thread = threading.Thread(target=_probe_loop, daemon=True, name="usb-probe")
    _probe_thread.start()
    logger.info("USB probe thread started")


def stop_probe():
    """Stop the background probe thread."""
    _stop_probe.set()
    if _probe_thread and _probe_thread.is_alive():
        _probe_thread.join(timeout=5)
