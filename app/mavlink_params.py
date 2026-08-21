"""
MAVLink parameter read/write over mavlink2rest.

The rest of this extension only ever *commands* vehicles (COMMAND_LONG,
RC overrides). Survey setup also needs to inspect and correct a handful
of autopilot parameters on two different vehicles:

* the local towfish (ArduSub) via ``host.docker.internal``
* the tow boat (ArduRover) via the operator-configured tow vehicle IP

Both expose a mavlink2rest instance, so the same client works for both
-- only the base URL differs.

Protocol notes
--------------
mavlink2rest has no dedicated parameter API. Reads and writes go through
plain MAVLink messages:

* ``PARAM_REQUEST_READ`` (POST) asks the autopilot to emit one param.
* ``PARAM_SET`` (POST) writes one param; ArduPilot answers with a
  ``PARAM_VALUE`` carrying the value it actually stored (which may be
  clamped or rounded to the parameter's real storage type).
* ``PARAM_VALUE`` (GET) is where both answers land.

The catch is that mavlink2rest caches exactly *one* PARAM_VALUE per
``(system, component)`` -- the most recent one, whoever asked for it. So
a read is "poke, then poll the single mailbox until the name matches".
Two things make that safe:

* We match on ``param_id`` so a QGC/Cockpit parameter download running
  in parallel can't hand us the wrong value.
* For write-verify we also require the cached message's ``last_update``
  timestamp to *change*, otherwise a pre-existing PARAM_VALUE for the
  same name would look like a successful write. Comparing the timestamp
  string against the one we saw before the POST keeps this correct even
  though mavlink2rest stamps messages with the remote host's clock,
  which is not comparable to ours.
"""

from __future__ import annotations

import copy
import json
import logging
import threading
import time
from typing import Optional

import requests

logger = logging.getLogger(__name__)

# Base URL used for the towfish the extension runs alongside.
LOCAL_MAVLINK2REST_URL = "http://host.docker.internal/mavlink2rest"

# MAVLink pads char arrays with NULs; PARAM_SET / PARAM_REQUEST_READ both
# carry a 16-byte param_id.
PARAM_ID_LEN = 16

# ArduPilot ignores the declared type on PARAM_SET and stores the value
# using the parameter's real type, so REAL32 is safe for everything.
DEFAULT_PARAM_TYPE = "MAV_PARAM_TYPE_REAL32"


def chars_to_str(param_id_chars) -> str:
    """``['W','P','_','S','P','E','E','D','\\x00',...]`` -> ``'WP_SPEED'``."""
    if not isinstance(param_id_chars, (list, tuple)):
        return str(param_id_chars or "").rstrip("\x00")
    return "".join(c for c in param_id_chars if c and c != "\x00")


def str_to_chars(param_id: str, pad_len: int = PARAM_ID_LEN) -> list[str]:
    """``'WP_SPEED'`` -> a NUL-padded 16-element char array."""
    chars = list(param_id)[:pad_len]
    chars.extend("\x00" for _ in range(pad_len - len(chars)))
    return chars


# Heartbeats we treat as "this is an autopilot", as opposed to BlueOS
# companion computers (MAV_AUTOPILOT_INVALID / ONBOARD_CONTROLLER) or
# GCS nodes on system 255.
_REAL_AUTOPILOTS = (
    "MAV_AUTOPILOT_ARDUPILOTMEGA",
    "MAV_AUTOPILOT_PX4",
)

# When a boat's mavlink2rest can also see the towfish (shared MAVLink
# network), prefer the surface vehicle so parameters and GPS don't land
# on the wrong autopilot.
BOAT_MAVTYPES = (
    "MAV_TYPE_SURFACE_BOAT",
    "MAV_TYPE_GROUND_ROVER",
    "MAV_TYPE_GROUND",
)


def _heartbeat_enum(message: dict, field: str) -> str:
    """Pull a mavlink2rest enum ``type`` string out of a HEARTBEAT field."""
    value = message.get(field)
    if isinstance(value, dict):
        return str(value.get("type") or "")
    return str(value or "")


def pick_autopilot(vehicles, prefer_mavtypes=None):
    """Return ``(system_id, component_id)`` of a real autopilot, or None.

    ``vehicles`` is the JSON object from ``GET /mavlink/vehicles``.
    Companion computers and GCS nodes are ignored. When more than one
    autopilot is visible, ``prefer_mavtypes`` (e.g. :data:`BOAT_MAVTYPES`)
    picks the one whose HEARTBEAT.mavtype matches, in listed order.
    """
    if not isinstance(vehicles, dict):
        return None
    found = []
    for vid, vehicle in vehicles.items():
        if not isinstance(vehicle, dict):
            continue
        try:
            sysid = int(vid)
        except (TypeError, ValueError):
            continue
        components = vehicle.get("components") or {}
        if not isinstance(components, dict):
            continue
        for cid, component in components.items():
            if not isinstance(component, dict):
                continue
            try:
                comp = int(cid)
            except (TypeError, ValueError):
                continue
            heartbeat = (((component.get("messages") or {}).get("HEARTBEAT")
                          or {}).get("message") or {})
            if not isinstance(heartbeat, dict):
                continue
            if _heartbeat_enum(heartbeat, "autopilot") not in _REAL_AUTOPILOTS:
                continue
            found.append((sysid, comp, _heartbeat_enum(heartbeat, "mavtype")))
    if not found:
        return None
    for preferred in tuple(prefer_mavtypes or ()):
        for sysid, comp, mavtype in found:
            if mavtype == preferred:
                return sysid, comp
    return found[0][0], found[0][1]


def discover_autopilot(base_url: str, prefer_mavtypes=None,
                       timeout_s: float = 2.0):
    """Ask mavlink2rest which vehicle/component is the autopilot.

    Returns ``(system_id, component_id)`` or ``None`` when the host is
    down or isn't publishing an autopilot HEARTBEAT.
    """
    url = f"{base_url.rstrip('/')}/mavlink/vehicles"
    try:
        response = requests.get(url, timeout=timeout_s)
        if response.status_code != 200 or not response.content:
            return None
        body = (response.text or "").strip()
        if not body or body == "None":
            return None
        return pick_autopilot(response.json(), prefer_mavtypes=prefer_mavtypes)
    except Exception as e:
        logger.debug("autopilot discovery at %s failed: %s", url, e)
        return None


class ParamReadError(Exception):
    """Raised when a parameter could not be read or written in time."""


class ParamClient:
    """Read and write autopilot parameters through one mavlink2rest host.

    Instances are cheap and hold only an HTTP session plus a message
    template cache, so callers can build one per operation. All methods
    are safe to call from Flask worker threads; a per-instance lock
    serialises access to the single PARAM_VALUE mailbox so two concurrent
    reads can't consume each other's answers.
    """

    def __init__(self, base_url: str,
                 target_system: int = 1, target_component: int = 1,
                 gcs_system_id: int = 255, gcs_component_id: int = 240,
                 http_timeout_s: float = 2.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.target_system = target_system
        self.target_component = target_component
        self.gcs_system_id = gcs_system_id
        self.gcs_component_id = gcs_component_id
        self.http_timeout_s = http_timeout_s
        self._template_cache: dict[str, dict] = {}
        self._lock = threading.Lock()

    # -- low-level HTTP --------------------------------------------------
    def _get_json(self, path: str) -> Optional[dict]:
        url = f"{self.base_url}{path}"
        try:
            r = requests.get(url, timeout=self.http_timeout_s)
            if r.status_code != 200:
                return None
            body = (r.text or "").strip()
            # mavlink2rest answers "None" for a message it has never seen.
            if not body or body == "None":
                return None
            return json.loads(body)
        except Exception as e:
            logger.debug("mavlink2rest GET %s failed: %s", url, e)
            return None

    def _post(self, msg: dict, info: str) -> bool:
        try:
            r = requests.post(f"{self.base_url}/mavlink", json=msg,
                              timeout=self.http_timeout_s)
            if r.status_code != 200:
                logger.debug("mavlink2rest POST %s -> HTTP %s: %s",
                             info, r.status_code, r.text[:200])
                return False
            if (r.text or "").strip().lower().startswith("failed"):
                logger.warning("mavlink2rest rejected %s: %s",
                               info, r.text[:200])
                return False
            return True
        except Exception as e:
            logger.debug("mavlink2rest POST %s failed: %s", info, e)
            return False

    def _envelope(self, message: dict) -> dict:
        """Wrap a message body in the header mavlink2rest expects.

        Prefers the server's own template for the message type so field
        names track whatever dialect mavlink2rest was built against; falls
        back to the hand-built body when ``/helper/mavlink`` is missing.
        """
        msg_type = message["type"]
        template = self._template_cache.get(msg_type)
        if template is None:
            fetched = self._get_json(f"/helper/mavlink?name={msg_type}")
            if isinstance(fetched, dict) and "message" in fetched:
                self._template_cache[msg_type] = fetched
                template = fetched
        body = message
        if template is not None:
            body = copy.deepcopy(template["message"])
            body.update(message)
        return {
            "header": {
                "system_id": self.gcs_system_id,
                "component_id": self.gcs_component_id,
                "sequence": 0,
            },
            "message": body,
        }

    # -- PARAM_VALUE mailbox ---------------------------------------------
    def _param_value_mailbox(self) -> tuple[Optional[dict], Optional[str]]:
        """Return ``(message, last_update)`` for the cached PARAM_VALUE.

        ``last_update`` is mavlink2rest's own timestamp string. We only
        ever compare it for equality against a previously observed value,
        never against our own clock -- the tow boat stamps with its clock,
        not ours.
        """
        wrapper = self._get_json(
            f"/mavlink/vehicles/{self.target_system}"
            f"/components/{self.target_component}/messages/PARAM_VALUE"
        )
        if not isinstance(wrapper, dict):
            return None, None
        message = wrapper.get("message")
        stamp = (((wrapper.get("status") or {}).get("time") or {})
                 .get("last_update"))
        return (message if isinstance(message, dict) else None), stamp

    @staticmethod
    def _decode_param_value(message: dict) -> dict:
        param_type = message.get("param_type")
        if isinstance(param_type, dict):
            param_type = param_type.get("type")
        return {
            "name": chars_to_str(message.get("param_id")),
            "value": message.get("param_value"),
            "type": param_type,
            "index": message.get("param_index"),
            "count": message.get("param_count"),
        }

    def _request_read(self, param_id: str) -> bool:
        return self._post(self._envelope({
            "type": "PARAM_REQUEST_READ",
            "target_system": self.target_system,
            "target_component": self.target_component,
            # -1 means "look the parameter up by name, not by index".
            "param_index": -1,
            "param_id": str_to_chars(param_id),
        }), f"PARAM_REQUEST_READ:{param_id}")

    def _await_param_value(self, param_id: str, deadline: float,
                           reject_stamp: Optional[str],
                           repoke) -> Optional[dict]:
        """Poll the PARAM_VALUE mailbox until ``param_id`` shows up.

        ``reject_stamp`` makes the wait ignore a cached message that was
        already there before we asked, which is what turns a write into a
        genuine read-back rather than an echo of the old value. ``repoke``
        is called roughly once a second to re-send the request, since a
        single UDP-ish MAVLink request can be dropped in transit.
        """
        last_poke = time.monotonic()
        while time.monotonic() < deadline:
            message, stamp = self._param_value_mailbox()
            if message is not None:
                decoded = self._decode_param_value(message)
                fresh = reject_stamp is None or stamp != reject_stamp
                if decoded["name"] == param_id and fresh:
                    return decoded
            now = time.monotonic()
            if now - last_poke >= 1.0:
                repoke()
                last_poke = now
            time.sleep(0.12)
        return None

    # -- public API -------------------------------------------------------
    def read(self, param_id: str, timeout_s: float = 3.0) -> dict:
        """Read one parameter, raising ``ParamReadError`` on timeout.

        Returns ``{"name", "value", "type", "index", "count"}``.
        """
        with self._lock:
            deadline = time.monotonic() + timeout_s
            if not self._request_read(param_id):
                raise ParamReadError("mavlink2rest unreachable")
            result = self._await_param_value(
                param_id, deadline, reject_stamp=None,
                repoke=lambda: self._request_read(param_id),
            )
        if result is None:
            raise ParamReadError(f"no PARAM_VALUE for {param_id}")
        return result

    def write(self, param_id: str, value: float,
              param_type: str = DEFAULT_PARAM_TYPE,
              timeout_s: float = 5.0) -> dict:
        """Write one parameter and return the autopilot's read-back.

        The returned value is what the autopilot says it stored, which is
        the only thing worth showing the operator -- ArduPilot silently
        clamps out-of-range values and rounds integer-typed parameters.
        """
        message = {
            "type": "PARAM_SET",
            "target_system": self.target_system,
            "target_component": self.target_component,
            "param_id": str_to_chars(param_id),
            "param_value": float(value),
            "param_type": {"type": param_type},
        }

        def send() -> bool:
            return self._post(self._envelope(message),
                              f"PARAM_SET:{param_id}={value}")

        with self._lock:
            # Snapshot the mailbox first so the read-back below can tell a
            # genuine ack apart from a stale PARAM_VALUE for the same name.
            _, before_stamp = self._param_value_mailbox()
            deadline = time.monotonic() + timeout_s
            if not send():
                raise ParamReadError("mavlink2rest unreachable")
            result = self._await_param_value(
                param_id, deadline, reject_stamp=before_stamp, repoke=send,
            )
        if result is None:
            raise ParamReadError(f"no PARAM_VALUE ack for {param_id}")
        return result

    def is_reachable(self, timeout_s: float = 1.5) -> bool:
        """True when the host answers with a recent HEARTBEAT.

        Used to tell "the boat is off / wrong IP" apart from "the boat is
        up but this parameter doesn't exist on its firmware".
        """
        try:
            r = requests.get(
                f"{self.base_url}/mavlink/vehicles/{self.target_system}"
                f"/components/{self.target_component}/messages/HEARTBEAT",
                timeout=timeout_s,
            )
        except Exception:
            return False
        if r.status_code != 200:
            return False
        body = (r.text or "").strip()
        return bool(body) and body != "None"
