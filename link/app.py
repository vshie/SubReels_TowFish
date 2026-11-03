#!/usr/bin/env python3
"""SubReels TowFish Link: puts the towfish widget in the tow boat's Cockpit.

Cockpit only lists widgets from the BlueOS of the vehicle it is connected
to, and that is the boat, while the TowFish extension runs on the fish.
This service runs on the boat's BlueOS and does nothing but advertise the
fish's widget: its cockpit_extras.json names an iframe URL on the fish
(``http://<fish>/widget``), so the widget loads straight from the fish over
the shared network.

Standard library only, so the image stays small.
"""
from __future__ import annotations

import base64
import json
import os
import re
import threading
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PORT = 5424
DATA_DIR = Path(os.environ.get("LINK_DATA_DIR", "/app/data"))
CONFIG_PATH = DATA_DIR / "link_config.json"
HERE = Path(__file__).resolve().parent

# Where the TowFish extension listens: the fish's IP and the extension's own
# port. The widget fetches root paths (/status, /config), so it has to load
# from that port, not through BlueOS's /extensionv2/ proxy.
DEFAULT_FISH_ADDRESS = "192.168.2.2:5423"
_ADDRESS_RE = re.compile(r"^[A-Za-z0-9.\-]+(:\d{1,5})?$")

# Inlined so the widget menu shows an icon even while the fish is powered off.
ICON_URL = "data:image/png;base64," + base64.b64encode(
    (HERE / "icon.png").read_bytes()).decode()

_lock = threading.Lock()


def _sanitize_address(raw) -> str | None:
    """Accept ``host`` or ``host:port``, dropping any scheme or trailing path."""
    if not isinstance(raw, str):
        return None
    addr = raw.strip()
    addr = re.sub(r"^[a-z]+://", "", addr, flags=re.I).split("/")[0]
    if not _ADDRESS_RE.match(addr):
        return None
    if ":" not in addr:
        addr += ":5423"
    return addr


def load_address() -> str:
    try:
        saved = json.loads(CONFIG_PATH.read_text()).get("fish_address")
    except (OSError, ValueError, AttributeError):
        saved = None
    return _sanitize_address(saved) or DEFAULT_FISH_ADDRESS


def save_address(addr: str) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    tmp = CONFIG_PATH.with_suffix(".tmp")
    tmp.write_text(json.dumps({"fish_address": addr}))
    tmp.replace(CONFIG_PATH)


def fish_status(addr: str) -> dict:
    """Ask the fish's extension who it is, so the page can say if it is reachable."""
    try:
        with urllib.request.urlopen(f"http://{addr}/register_service", timeout=2) as r:
            info = json.loads(r.read().decode())
        return {"reachable": True, "name": info.get("name"), "version": info.get("version")}
    except Exception as exc:  # noqa: BLE001 - any failure just means "not reachable"
        return {"reachable": False, "error": str(exc)}


REGISTER_SERVICE = {
    "name": "SubReels: TowFish Link",
    "description": "Runs on the tow boat and adds the SubReels TowFish widget to Cockpit, "
                   "loading it from the towfish over the network.",
    "icon": "mdi-link-variant",
    "company": "Blue Robotics",
    "version": "0.99",
    "webpage": "https://github.com/vshie/SubReels_TowFish",
    "api": "https://github.com/bluerobotics/BlueOS-docker",
    "works_in_relative_paths": True,
    "extras": {"cockpit": "/cockpit_extras.json"},
}


def cockpit_extras(addr: str) -> dict:
    return {
        "targetCockpitApiVersion": "1.0.0",
        "targetSystem": "Cockpit",
        "widgets": [{
            "name": "SubReels TowFish",
            "iframeUrl": f"http://{addr}/widget",
            "iconUrl": ICON_URL,
        }],
    }


class Handler(BaseHTTPRequestHandler):
    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code: int = 200) -> None:
        self._send(code, json.dumps(obj).encode(), "application/json")

    def do_GET(self):  # noqa: N802 - http.server naming
        path = self.path.split("?")[0]
        if path == "/register_service":
            self._json(REGISTER_SERVICE)
        elif path == "/cockpit_extras.json":
            self._json(cockpit_extras(load_address()))
        elif path == "/status":
            addr = load_address()
            self._json({"fish_address": addr, "widget_url": f"http://{addr}/widget",
                        **fish_status(addr)})
        elif path in ("/", "/index.html"):
            self._send(200, (HERE / "index.html").read_bytes(), "text/html; charset=utf-8")
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):  # noqa: N802
        if self.path.split("?")[0] != "/config":
            self._json({"error": "not found"}, 404)
            return
        try:
            length = int(self.headers.get("Content-Length") or 0)
            data = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            self._json({"error": "body must be JSON"}, 400)
            return
        addr = _sanitize_address(data.get("fish_address"))
        if addr is None:
            self._json({"error": "fish_address must be an IP or hostname, optionally with :port"}, 400)
            return
        with _lock:
            save_address(addr)
        self._json({"fish_address": addr})

    def log_message(self, fmt, *args):
        pass


if __name__ == "__main__":
    print(f"TowFish Link on :{PORT}, fish at {load_address()}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
