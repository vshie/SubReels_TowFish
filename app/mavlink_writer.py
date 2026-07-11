"""
Thin mavlink2rest POST helpers for arming, flight mode, mount, focus,
zoom and RC channel overrides on the local ArduSub towfish.

All commands are proven live on the vehicle (see the towfish-usability
plan). We never fall back to pymavlink -- BlueOS' local mavlink2rest
already carries every write we need.

Endpoint conventions
--------------------
* Base URL: ``http://host.docker.internal/mavlink2rest`` (from inside the
  extension container). Overridable so unit tests / local runs on the LAN
  can hit ``http://192.168.1.8/mavlink2rest`` directly.
* Sending: ``POST /mavlink`` with the JSON payload shape mavlink2rest
  expects (``header`` + ``message``).
* GCS identity: ``system_id=255`` (typical Blue Robotics GCS ID) and
  ``component_id=240`` (autopilot-compatible companion). These match the
  values that produced ACCEPTED acks during live verification.

Threading
---------
Callers may hit these helpers from arbitrary Flask threads. Each helper
uses a fresh ``requests`` call (thread-safe) with a bounded timeout so a
slow autopilot can never wedge the caller. On failure we log at DEBUG
and return False -- writes are fire-and-forget from the UI's point of
view, and every button that fires them is idempotent.
"""

from __future__ import annotations

import logging
from typing import Iterable, Optional

import requests

logger = logging.getLogger(__name__)

# Default endpoint used from inside the BlueOS extension container.
DEFAULT_MAVLINK2REST_URL = "http://host.docker.internal/mavlink2rest"

# ---------------------------------------------------------------------------
# Constants -- proven with COMMAND_ACK MAV_RESULT_ACCEPTED on vehicle
# ---------------------------------------------------------------------------
# ArduSub custom_mode numbers (see HAUV.lua for the same values).
MODE_STABILIZE = 0
MODE_ALT_HOLD = 2
MODE_MANUAL = 19

# MAV_MOUNT_MODE_MAVLINK_TARGETING = 2 -- required so DO_MOUNT_CONTROL
# treats pitch as the earth-frame command rather than being ignored.
MOUNT_MODE_MAVLINK_TARGETING = 2

# MAV_CMD_SET_CAMERA_FOCUS / _ZOOM: type 2 = RANGE (0..100 percent).
CAMERA_RANGE_TYPE = 2

# Straight-down camera pitch on this mount (matches MNT1_PITCH_MIN and
# the survey attitude that produced ~2140 us on SERVO16).
TILT_PITCH_DOWN_DEG = -70.0

# Focus PWM ↔ RANGE conversion for SERVO12 (Camera Focus). SERVO12_MIN
# = 870, SERVO12_MAX = 2130 on this vehicle. Trim 1639 ≈ 61.03%.
FOCUS_PWM_MIN = 870
FOCUS_PWM_MAX = 2130

# Neutral (release) PWM used to clear an RC override without commanding
# motion. 0 tells ArduSub "no override on this channel".
RC_OVERRIDE_RELEASE = 0

# Defaults for the depth jog. In stock ArduSub RC3 (Throttle) is the
# vertical thrust axis with the joystick convention "1900 = full up,
# 1100 = full down" (see BlueROV2 setup docs). We default to that on
# the towfish and let the operator flip these constants after the
# first armed AltHold check on 192.168.1.8 if the frame is wired
# opposite -- HAUV.lua's *direct* SERVO5/6 conventions (descent = 1750)
# do NOT determine RC3 override direction because RC3 goes through the
# frame mixer, not the raw output.
Z_CHANNEL = 3
Z_PWM_ASCEND = 1600
Z_PWM_DESCEND = 1400
Z_PWM_NEUTRAL = 1500


class MavlinkWriter:
    """Small wrapper that batches all POSTs against one mavlink2rest URL.

    One instance per process is plenty; the helpers below expose module
    functions that dispatch to a shared default instance.
    """

    def __init__(self, base_url: str = DEFAULT_MAVLINK2REST_URL,
                 target_system: int = 1, target_component: int = 1,
                 gcs_system_id: int = 255, gcs_component_id: int = 240,
                 timeout_s: float = 2.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.target_system = target_system
        self.target_component = target_component
        self.gcs_system_id = gcs_system_id
        self.gcs_component_id = gcs_component_id
        self.timeout_s = timeout_s

    # -- low-level ------------------------------------------------------
    def _post(self, message: dict) -> bool:
        payload = {
            "header": {
                "system_id": self.gcs_system_id,
                "component_id": self.gcs_component_id,
                "sequence": 0,
            },
            "message": message,
        }
        try:
            r = requests.post(
                f"{self.base_url}/mavlink", json=payload,
                timeout=self.timeout_s,
            )
            if r.status_code != 200:
                logger.debug(
                    "mavlink2rest POST %s -> HTTP %s: %s",
                    message.get("type"), r.status_code, r.text[:200],
                )
                return False
            body = (r.text or "").strip()
            # mavlink2rest returns the string "Failed to parse message,
            # not a valid MAVLinkMessage." on schema mismatches. Treat
            # anything obviously error-shaped as failure.
            if body.lower().startswith("failed"):
                logger.warning(
                    "mavlink2rest rejected %s: %s",
                    message.get("type"), body[:200],
                )
                return False
            return True
        except Exception as e:
            logger.debug("mavlink2rest POST %s failed: %s",
                         message.get("type"), e)
            return False

    def command_long(self, command_type: str,
                     p1: float = 0.0, p2: float = 0.0, p3: float = 0.0,
                     p4: float = 0.0, p5: float = 0.0, p6: float = 0.0,
                     p7: float = 0.0, confirmation: int = 0) -> bool:
        """POST a COMMAND_LONG with the given MAV_CMD_* string."""
        return self._post({
            "type": "COMMAND_LONG",
            "param1": float(p1), "param2": float(p2), "param3": float(p3),
            "param4": float(p4), "param5": float(p5), "param6": float(p6),
            "param7": float(p7),
            "command": {"type": command_type},
            "target_system": self.target_system,
            "target_component": self.target_component,
            "confirmation": int(confirmation),
        })

    # -- vehicle --------------------------------------------------------
    def set_mode(self, custom_mode: int) -> bool:
        """DO_SET_MODE with param1=1 (custom mode enabled), param2=mode.

        Raw ``SET_MODE`` messages fail to parse through mavlink2rest, so
        this is the only reliable path -- verified on 192.168.1.8.
        """
        return self.command_long(
            "MAV_CMD_DO_SET_MODE", p1=1.0, p2=float(int(custom_mode)),
        )

    def arm(self, armed: bool) -> bool:
        """COMPONENT_ARM_DISARM: param1=1 arms, 0 disarms."""
        return self.command_long(
            "MAV_CMD_COMPONENT_ARM_DISARM",
            p1=1.0 if armed else 0.0,
        )

    def mount_pitch(self, pitch_deg: float) -> bool:
        """DO_MOUNT_CONTROL with mount mode MAVLINK_TARGETING."""
        return self.command_long(
            "MAV_CMD_DO_MOUNT_CONTROL",
            p1=float(pitch_deg), p7=float(MOUNT_MODE_MAVLINK_TARGETING),
        )

    def tilt_down(self) -> bool:
        return self.mount_pitch(TILT_PITCH_DOWN_DEG)

    # -- camera ---------------------------------------------------------
    def set_camera_focus_range(self, pct: float) -> bool:
        """SET_CAMERA_FOCUS type=RANGE, value 0..100."""
        return self.command_long(
            "MAV_CMD_SET_CAMERA_FOCUS",
            p1=float(CAMERA_RANGE_TYPE), p2=float(max(0.0, min(100.0, pct))),
        )

    def set_camera_zoom_range(self, pct: float) -> bool:
        """SET_CAMERA_ZOOM type=RANGE, value 0..100."""
        return self.command_long(
            "MAV_CMD_SET_CAMERA_ZOOM",
            p1=float(CAMERA_RANGE_TYPE), p2=float(max(0.0, min(100.0, pct))),
        )

    # -- RC / thrust ----------------------------------------------------
    def rc_channels_override(self, channels: dict[int, int]) -> bool:
        """RC_CHANNELS_OVERRIDE with only the given channels forced.

        ``channels`` maps ``channel_number (1..18) -> pwm_us`` (or
        ``RC_OVERRIDE_RELEASE`` / 0 to release the channel). Any channel
        not in the dict is sent as 0 (released).
        """
        msg: dict = {
            "type": "RC_CHANNELS_OVERRIDE",
            "target_system": self.target_system,
            "target_component": self.target_component,
        }
        for ch in range(1, 19):
            msg[f"chan{ch}_raw"] = int(channels.get(ch, RC_OVERRIDE_RELEASE))
        return self._post(msg)

    def rc_release_all(self) -> bool:
        """Clear every RC override (all channels back to autopilot)."""
        return self.rc_channels_override({})


# ---------------------------------------------------------------------------
# Module-level default instance + thin wrappers so callers don't have to
# hold onto a MavlinkWriter object.
# ---------------------------------------------------------------------------
_default = MavlinkWriter()


def get_default_writer() -> MavlinkWriter:
    return _default


def set_default_writer(w: MavlinkWriter) -> None:
    global _default
    _default = w


def focus_pwm_to_pct(pwm: int) -> float:
    """Convert a raw servo12 PWM to RANGE percent."""
    span = FOCUS_PWM_MAX - FOCUS_PWM_MIN
    if span <= 0:
        return 0.0
    pct = 100.0 * (int(pwm) - FOCUS_PWM_MIN) / span
    return max(0.0, min(100.0, pct))


def focus_pct_to_pwm(pct: float) -> int:
    """Convert a RANGE percent back to expected servo12 PWM."""
    pct = max(0.0, min(100.0, float(pct)))
    return int(round(FOCUS_PWM_MIN + pct * (FOCUS_PWM_MAX - FOCUS_PWM_MIN) / 100.0))
