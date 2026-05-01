#!/usr/bin/env python3
"""
BlueOS extension: Marine Magnetics Explorer magnetometer reader.
Reads serial sentences, joins GPS from Mavlink2Rest, logs CSV, sends NAMED_VALUE_FLOAT.
"""
from __future__ import annotations

import csv
import json
import math
import os
import queue
import re
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import requests
import serial
from flask import Flask, Response, jsonify, request, send_file, send_from_directory, stream_with_context
from flask_cors import CORS
from serial.tools import list_ports
from waitress import serve

# -----------------------------------------------------------------------------
# Paths & constants
# -----------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent
LOG_DIR = Path(os.environ.get("LOG_DIR", "/app/logs"))
STATE_FILENAME = "state.json"
LOG_PREFIX = "explorer_"
LOG_SUFFIX = ".csv"
SIM_LOG_INFIX = "_sim"

MAVLINK_BASE = os.environ.get(
    "MAVLINK2REST_BASE", "http://host.docker.internal/mavlink2rest/mavlink"
)
MAVLINK_VEHICLE_ID = int(os.environ.get("MAVLINK_VEHICLE_ID", "1"))

MAVLINK_POST_ENDPOINTS = [
    "http://host.docker.internal/mavlink2rest/mavlink",
    "http://host.docker.internal:6040/v1/mavlink",
    "http://192.168.2.2/mavlink2rest/mavlink",
    "http://localhost/mavlink2rest/mavlink",
    "http://blueos.local/mavlink2rest/mavlink",
]

BOOT_MONO = time.monotonic()

# Explorer default per manufacturer: 9600 8N1
BAUD_CHOICES = [1200, 2400, 4800, 9600, 19200]
DEFAULT_LAYBACK_X_M = 0.0  # positive = starboard, negative = port
DEFAULT_LAYBACK_Y_M = -5.0  # positive = forward, negative = behind / layback

# Max ~4 Hz bursts to autopilot (4 NVF per burst)
NVF_MIN_INTERVAL_S = 0.25
EXPLORER_4HZ_COMMAND = b"1"  # Manual: "1 Set 250ms cycle time" (4 Hz)

SENTENCE_RE = re.compile(
    r"^\*(?P<year>\d{2})\.(?P<jday>\d{3})/"
    r"(?P<time>\d{2}:\d{2}:\d{2}\.\d)\s+"
    r"F:(?P<field>[-+]?\d+\.\d+)\s+"
    r"S:(?P<signal>\d+)"
    r"(?:\s+D:(?P<depth>[-+]?\d+\.\d+)m)?"
    r"(?:\s+A:(?P<alt>[-+]?\d+\.\d+)m)?"
    r"\s+L(?P<leak>\d)\s+"
    r"(?P<larmor>\d+)ms"
    r"\s+Q:(?P<quality>\d+)"
    r"(?:\s+\S+)*\s*$"
)

CSV_COLUMNS = [
    "utc_time",
    "unix_ms",
    "year",
    "julian_day",
    "sensor_time",
    "field_nt",
    "signal",
    "depth_m",
    "altitude_m",
    "leak",
    "larmor_ms",
    "quality",
    "lat",
    "lon",
    "gps_alt_m",
    "gps_age_ms",
    "layback_lat",
    "layback_lon",
    "layback_bearing_deg",
    "layback_x_m",
    "layback_y_m",
    "raw_sentence",
]

# BlueOS-style path-to-position map (aligned with Airmar-WX). Longest keys first.
_USB_PATH_POSITION_MAP = sorted(
    [
        ("platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3", "top-left"),
        ("platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4", "bottom-left"),
        ("platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1", "top-right"),
        ("platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2", "bottom-right"),
        ("-usb-0:1.1.3", "top-left"),
        ("-usb-0:1.1.2", "bottom-left"),
        ("-usb-0:1.1.4", "bottom-left"),
        ("-usb-0:1.3:", "top-right"),
        ("-usb-0:1.3-", "top-right"),
        ("-usb-0:1.2:", "bottom-right"),
        ("-usb-0:1.2-", "bottom-right"),
        ("-usb-0:1.4", "bottom-left"),
        ("-usb-0:1.1", "top-right"),
        ("platform-3f980000.usb-usb-0:1.5:1", "bottom-right"),
        ("platform-3f980000.usb-usb-0:1.4:1", "top-right"),
        ("platform-3f980000.usb-usb-0:1.3:1", "bottom-left"),
        ("platform-3f980000.usb-usb-0:1.2:1", "top-left"),
        ("platform-xhci-hcd.1-usb-0:2", "bottom-right"),
        ("platform-xhci-hcd.0-usb-0:2", "top-right"),
        ("platform-xhci-hcd.1-usb-0:1", "top-left"),
        ("platform-xhci-hcd.0-usb-0:1", "bottom-left"),
    ],
    key=lambda p: -len(p[0]),
)


def parse_usb_port(path_name: str) -> Dict[str, Any]:
    """Map /dev/serial/by-path symlink name to physical USB slot (BlueOS / Pi layouts)."""
    try:
        usb_root = path_name.split("-port0")[0] if path_name else ""
        bus_match = re.search(r"usb-(\d+:\d+(?:\.\d+)*)", path_name or "")
        bus_path = bus_match.group(1) if bus_match else (usb_root or "")

        hub_info: Optional[str] = None
        hub_match = re.search(r"usb-0:(?:[0-9]+\.)+([0-9]+):1\.0", path_name or "")
        if hub_match:
            hub_info = f"Via hub, port {hub_match.group(1)}"

        for key, position in _USB_PATH_POSITION_MAP:
            if key in usb_root:
                label = position.replace("-", " ").title()
                result: Dict[str, Any] = {
                    "position": position,
                    "label": label,
                    "type": "usb3" if "right" in position else "usb2",
                    "bus": bus_path,
                }
                if hub_info:
                    result["hub_info"] = hub_info
                return result

        result = {"position": "unknown", "label": "Unknown", "type": "unknown", "bus": bus_path}
        if hub_info:
            result["hub_info"] = hub_info
        return result
    except Exception:
        return {"position": "unknown", "label": "Unknown", "type": "unknown", "bus": ""}


def get_device_ids() -> List[Dict[str, Any]]:
    """Serial devices under /dev/serial/by-id with USB port hints from /dev/serial/by-path."""
    devices: List[Dict[str, Any]] = []
    by_path_map: Dict[str, Dict[str, Any]] = {}
    try:
        serial_by_path = Path("/dev/serial/by-path")
        if serial_by_path.is_dir():
            for link in serial_by_path.iterdir():
                try:
                    if link.is_symlink():
                        real_device = str(link.resolve())
                        path_name = link.name
                        usb_port = parse_usb_port(path_name)
                        by_path_map[real_device] = {"path_name": path_name, "usb_port": usb_port}
                except OSError:
                    continue
    except OSError:
        pass

    try:
        serial_by_id = Path("/dev/serial/by-id")
        if serial_by_id.is_dir():
            for link in serial_by_id.iterdir():
                try:
                    if link.is_symlink():
                        real_device = str(link.resolve())
                        by_id_name = link.name
                        display_name = (
                            by_id_name.replace("usb-", "")
                            .replace("-if00-port0", "")
                            .replace("_", " ")
                        )
                        path_info = by_path_map.get(real_device, {})
                        usb_port = path_info.get(
                            "usb_port",
                            {"position": "unknown", "label": "Unknown", "type": "unknown"},
                        )
                        devices.append(
                            {
                                "device": real_device,
                                "by_id_name": by_id_name,
                                "display_name": display_name,
                                "usb_port": usb_port,
                                "path_name": path_info.get("path_name", ""),
                            }
                        )
                except OSError:
                    continue
    except OSError:
        pass

    devices.sort(key=lambda x: x["device"])
    return devices


app = Flask(__name__, static_folder=str(ROOT / "static"), static_url_path="/static")
CORS(app)

# -----------------------------------------------------------------------------
# Global runtime state (protected by state_lock)
# -----------------------------------------------------------------------------
state_lock = threading.Lock()
sse_clients: List[queue.Queue] = []

connected = False
simulating = False
stop_event = threading.Event()
serial_thread: Optional[threading.Thread] = None
simulation_thread: Optional[threading.Thread] = None
gps_thread: Optional[threading.Thread] = None

ser: Optional[serial.Serial] = None
csv_file = None
csv_writer: Optional[csv.writer] = None
current_log_name: Optional[str] = None

raw_lines: deque = deque(maxlen=10)
vehicle_id = MAVLINK_VEHICLE_ID

gps_fix: Dict[str, Any] = {
    "lat": None,
    "lon": None,
    "alt_m": None,
    "last_ok_mono": None,
    "last_error": None,
}
gps_prev_position: Optional[Tuple[float, float]] = None
gps_bearing_rad: Optional[float] = None
layback_x_m = DEFAULT_LAYBACK_X_M
layback_y_m = DEFAULT_LAYBACK_Y_M

mavlink_seq = 0
nvf_posts_ok = 0
nvf_last_status = "idle"
nvf_last_endpoint: Optional[str] = None
last_parsed: Optional[Dict[str, Any]] = None
last_nvf_burst_mono = 0.0

reader_error: Optional[str] = None


def ensure_log_dir() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)


def state_path() -> Path:
    return LOG_DIR / STATE_FILENAME


def load_settings() -> Dict[str, Any]:
    ensure_log_dir()
    p = state_path()
    if not p.is_file():
        return {
            "port": "",
            "baud_rate": 9600,
            "layback_x_m": DEFAULT_LAYBACK_X_M,
            "layback_y_m": DEFAULT_LAYBACK_Y_M,
        }
    try:
        with p.open("r", encoding="utf-8") as f:
            data = json.load(f)
        br = int(data.get("baud_rate", 9600))
        if br not in BAUD_CHOICES:
            br = 9600
        return {
            "port": str(data.get("port", "")),
            "baud_rate": br,
            "layback_x_m": float(data.get("layback_x_m", DEFAULT_LAYBACK_X_M)),
            "layback_y_m": float(data.get("layback_y_m", DEFAULT_LAYBACK_Y_M)),
        }
    except Exception:
        return {
            "port": "",
            "baud_rate": 9600,
            "layback_x_m": DEFAULT_LAYBACK_X_M,
            "layback_y_m": DEFAULT_LAYBACK_Y_M,
        }


def save_settings(
    port: str,
    baud_rate: int,
    layback_x_m: Optional[float] = None,
    layback_y_m: Optional[float] = None,
) -> None:
    ensure_log_dir()
    if baud_rate not in BAUD_CHOICES:
        baud_rate = 9600
    current = load_settings()
    if layback_x_m is None:
        layback_x_m = float(current.get("layback_x_m", DEFAULT_LAYBACK_X_M))
    if layback_y_m is None:
        layback_y_m = float(current.get("layback_y_m", DEFAULT_LAYBACK_Y_M))
    with state_path().open("w", encoding="utf-8") as f:
        json.dump(
            {
                "port": port,
                "baud_rate": baud_rate,
                "layback_x_m": float(layback_x_m),
                "layback_y_m": float(layback_y_m),
            },
            f,
            indent=2,
        )


def sse_broadcast(obj: Dict[str, Any]) -> None:
    line = f"data: {json.dumps(obj)}\n\n"
    with state_lock:
        dead: List[queue.Queue] = []
        for q in sse_clients:
            try:
                q.put_nowait(line)
            except queue.Full:
                dead.append(q)
        for q in dead:
            if q in sse_clients:
                sse_clients.remove(q)


def apply_layback_settings(settings: Dict[str, Any]) -> None:
    global layback_x_m, layback_y_m
    with state_lock:
        layback_x_m = float(settings.get("layback_x_m", DEFAULT_LAYBACK_X_M))
        layback_y_m = float(settings.get("layback_y_m", DEFAULT_LAYBACK_Y_M))


def parse_sentence(raw: str) -> Optional[Dict[str, Any]]:
    m = SENTENCE_RE.match(raw.strip())
    if not m:
        return None
    g = m.groupdict()
    depth = g.get("depth")
    alt = g.get("alt")
    qual = g.get("quality")
    return {
        "year": g["year"],
        "julian_day": g["jday"],
        "sensor_time": g["time"],
        "field_nt": float(g["field"]),
        "signal": int(g["signal"]),
        "depth_m": float(depth) if depth is not None else None,
        "altitude_m": float(alt) if alt is not None else None,
        "leak": int(g["leak"]),
        "larmor_ms": int(g["larmor"]),
        "quality": int(qual) if qual is not None else None,
    }


def calculate_direction(
    prev_lat: float,
    prev_lon: float,
    curr_lat: float,
    curr_lon: float,
) -> Optional[float]:
    """Bearing in radians from previous to current GPS point (0=north, pi/2=east)."""
    if abs(prev_lat - curr_lat) < 1e-9 and abs(prev_lon - curr_lon) < 1e-9:
        return None
    dlon = curr_lon - prev_lon
    dlat = curr_lat - prev_lat
    return math.atan2(dlon, dlat)


def calculate_layback_position(
    gnss_lat: Optional[float],
    gnss_lon: Optional[float],
    offset_x_m: float,
    offset_y_m: float,
    bearing: Optional[float],
) -> Tuple[Optional[float], Optional[float]]:
    """Estimate towfish position using vessel GPS, lateral/longitudinal offset, and motion bearing.

    Uses the BlueBoatExplorerMag.py convention:
    x is starboard/port, y is forward/back. Instead of pyproj UTM transforms, use a local
    meters-per-degree approximation, which is accurate enough for short layback offsets.
    """
    if gnss_lat is None or gnss_lon is None or bearing is None:
        return gnss_lat, gnss_lon

    cos_b = math.cos(bearing)
    sin_b = math.sin(bearing)
    east_offset_m = offset_x_m * cos_b + offset_y_m * sin_b
    north_offset_m = -offset_x_m * sin_b + offset_y_m * cos_b

    lat_rad = math.radians(gnss_lat)
    meters_per_deg_lat = 111_320.0
    meters_per_deg_lon = max(1e-6, 111_320.0 * math.cos(lat_rad))

    return (
        gnss_lat + (north_offset_m / meters_per_deg_lat),
        gnss_lon + (east_offset_m / meters_per_deg_lon),
    )


def mavlink_sequence_next() -> int:
    global mavlink_seq
    with state_lock:
        mavlink_seq = (mavlink_seq + 1) % 256
        return mavlink_seq


def build_nvf_payload(name: str, value: float, seq: int) -> Dict[str, Any]:
    name_array: List[str] = []
    for i in range(10):
        if i < len(name):
            name_array.append(name[i])
        else:
            name_array.append("\u0000")
    time_boot_ms = int((time.monotonic() - BOOT_MONO) * 1000) & 0xFFFFFFFF
    return {
        "header": {"system_id": 255, "component_id": 0, "sequence": seq},
        "message": {
            "type": "NAMED_VALUE_FLOAT",
            "time_boot_ms": time_boot_ms,
            "value": float(value),
            "name": name_array,
        },
    }


def send_named_value_float(name: str, value: float) -> Tuple[bool, Optional[str]]:
    """POST NAMED_VALUE_FLOAT to Mavlink2Rest; try endpoints in order."""
    seq = mavlink_sequence_next()
    payload = build_nvf_payload(name, value, seq)
    for post_url in MAVLINK_POST_ENDPOINTS:
        try:
            r = requests.post(post_url, json=payload, timeout=2.0)
            if r.status_code == 200:
                return True, post_url
        except Exception:
            continue
    return False, None


def send_nvf_burst(field_nt: float, signal: int, depth_m: Optional[float], quality: Optional[int]) -> None:
    """Send four NVFs; update counters and SSE."""
    global nvf_posts_ok, nvf_last_status, nvf_last_endpoint

    depth_val = float(depth_m) if depth_m is not None else 0.0
    qual_val = float(quality) if quality is not None else 0.0

    specs = [
        ("MAG_NT", field_nt),
        ("MAG_SIG", float(signal)),
        ("MAG_DEPTH", depth_val),
        ("MAG_QUAL", qual_val),
    ]
    ok_any = False
    last_url: Optional[str] = None
    for n, v in specs:
        ok, url = send_named_value_float(n, v)
        if ok:
            ok_any = True
            last_url = url
            with state_lock:
                nvf_posts_ok += 1
        else:
            with state_lock:
                nvf_last_status = f"failed:{n}"
                nvf_last_endpoint = None
            sse_broadcast({"type": "mavlink_ack", "ok": False, "name": n})
            return

    with state_lock:
        nvf_last_status = "ok"
        nvf_last_endpoint = last_url
    sse_broadcast({"type": "mavlink_ack", "ok": ok_any, "endpoint": last_url, "names": [s[0] for s in specs]})


def detect_vehicle_id() -> int:
    try:
        r = requests.get(f"{MAVLINK_BASE.rstrip('/')}/vehicles", timeout=1.5)
        if r.status_code != 200:
            return MAVLINK_VEHICLE_ID
        data = r.json()
        if isinstance(data, list) and data:
            first = data[0]
            if isinstance(first, dict):
                return int(first.get("id", first.get("system_id", MAVLINK_VEHICLE_ID)))
            return int(first)
        if isinstance(data, dict):
            vehicles = data.get("vehicles") or data.get("data") or data.get("items")
            if isinstance(vehicles, list) and vehicles:
                v0 = vehicles[0]
                if isinstance(v0, dict):
                    return int(v0.get("id", v0.get("system_id", MAVLINK_VEHICLE_ID)))
    except Exception:
        pass
    return MAVLINK_VEHICLE_ID


def gps_poll_loop() -> None:
    global vehicle_id, gps_fix, gps_prev_position, gps_bearing_rad
    vid = detect_vehicle_id()
    with state_lock:
        vehicle_id = vid
    url = f"{MAVLINK_BASE.rstrip('/')}/vehicles/{vid}/components/1/messages/GLOBAL_POSITION_INT"
    while not stop_event.is_set():
        try:
            r = requests.get(url, timeout=1.0)
            if r.status_code == 200:
                body = r.json()
                msg = body.get("message") or body
                lat_e7 = msg.get("lat")
                lon_e7 = msg.get("lon")
                alt_e7 = msg.get("alt")
                if lat_e7 is not None and lon_e7 is not None:
                    lat_deg = float(lat_e7) / 1e7
                    lon_deg = float(lon_e7) / 1e7
                    alt_m = float(alt_e7) / 1e7 if alt_e7 is not None else None
                    with state_lock:
                        if gps_prev_position is not None:
                            new_bearing = calculate_direction(
                                gps_prev_position[0],
                                gps_prev_position[1],
                                lat_deg,
                                lon_deg,
                            )
                            if new_bearing is not None:
                                gps_bearing_rad = new_bearing
                        gps_prev_position = (lat_deg, lon_deg)
                        gps_fix["lat"] = lat_deg
                        gps_fix["lon"] = lon_deg
                        gps_fix["alt_m"] = alt_m
                        gps_fix["last_ok_mono"] = time.monotonic()
                        gps_fix["last_error"] = None
                    sse_broadcast(
                        {
                            "type": "gps",
                            "lat": lat_deg,
                            "lon": lon_deg,
                            "alt_m": alt_m,
                        }
                    )
                else:
                    with state_lock:
                        gps_fix["last_error"] = "no lat/lon in message"
            else:
                with state_lock:
                    gps_fix["last_error"] = f"HTTP {r.status_code}"
        except Exception as e:
            with state_lock:
                gps_fix["last_error"] = str(e)
        stop_event.wait(0.5)


def open_csv_log() -> Tuple[str, Path]:
    ensure_log_dir()
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    name = f"{LOG_PREFIX}{ts}{LOG_SUFFIX}"
    path = LOG_DIR / name
    return name, path


def configure_explorer_4hz(serial_conn: serial.Serial) -> None:
    """Put Explorer into 250 ms / 4 Hz cycling mode on each serial connection."""
    serial_conn.write(EXPLORER_4HZ_COMMAND)
    serial_conn.flush()


def write_csv_row(parsed: Dict[str, Any], raw: str, unix_ms: int, utc_time: str) -> None:
    global csv_writer
    with state_lock:
        lat = gps_fix["lat"]
        lon = gps_fix["lon"]
        alt_m = gps_fix["alt_m"]
        last_ok = gps_fix["last_ok_mono"]
        bearing = gps_bearing_rad
        lb_x = layback_x_m
        lb_y = layback_y_m
    age_ms = ""
    if last_ok is not None:
        age_ms = int((time.monotonic() - last_ok) * 1000)
    layback_lat, layback_lon = calculate_layback_position(lat, lon, lb_x, lb_y, bearing)
    bearing_deg = ""
    if bearing is not None:
        bearing_deg = (math.degrees(bearing) + 360.0) % 360.0
    row = [
        utc_time,
        unix_ms,
        parsed["year"],
        parsed["julian_day"],
        parsed["sensor_time"],
        parsed["field_nt"],
        parsed["signal"],
        parsed["depth_m"] if parsed["depth_m"] is not None else "",
        parsed["altitude_m"] if parsed["altitude_m"] is not None else "",
        parsed["leak"],
        parsed["larmor_ms"],
        parsed["quality"] if parsed["quality"] is not None else "",
        lat if lat is not None else "",
        lon if lon is not None else "",
        alt_m if alt_m is not None else "",
        age_ms,
        layback_lat if layback_lat is not None else "",
        layback_lon if layback_lon is not None else "",
        bearing_deg,
        lb_x,
        lb_y,
        raw,
    ]
    with state_lock:
        if csv_writer is None:
            return
        csv_writer.writerow(row)
        if csv_file:
            csv_file.flush()


def build_simulated_sentence() -> str:
    """One Explorer-style line that matches SENTENCE_RE (for bench testing without hardware)."""
    now = datetime.now(timezone.utc)
    yy = now.strftime("%y")
    jjj = f"{now.timetuple().tm_yday:03d}"
    frac = (now.microsecond // 100_000) % 10
    tstr = now.strftime("%H:%M:%S") + f".{frac}"
    tmono = time.monotonic()
    field = 50_000.0 + 150.0 * math.sin(tmono * 0.8)
    signal = max(1, min(999, 85 + int(14 * (0.5 + 0.5 * math.sin(tmono * 0.31)))))
    depth = 8.0 + 2.0 * math.sin(tmono * 0.47)
    larmor = 100 + int(25 * abs(math.sin(tmono * 0.62)))
    return (
        f"*{yy}.{jjj}/{tstr} F:{field:.3f} S:{signal:03d} D:{depth:+.1f}m L0 {larmor:04d}ms Q:99"
    )


def process_incoming_sentence(raw: str) -> None:
    """Handle one raw line: UI feed, parse, CSV row, optional NAMED_VALUE_FLOAT burst."""
    global last_parsed, last_nvf_burst_mono

    raw = raw.strip()
    if not raw:
        return
    unix_ms = int(time.time() * 1000)
    utc_time = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

    with state_lock:
        raw_lines.append(raw)
    sse_broadcast({"type": "raw_sentence", "line": raw})

    parsed = parse_sentence(raw)
    if not parsed:
        return

    with state_lock:
        lat = gps_fix["lat"]
        lon = gps_fix["lon"]
        bearing = gps_bearing_rad
        lb_x = layback_x_m
        lb_y = layback_y_m
    layback_lat, layback_lon = calculate_layback_position(lat, lon, lb_x, lb_y, bearing)
    bearing_deg = None
    if bearing is not None:
        bearing_deg = (math.degrees(bearing) + 360.0) % 360.0

    merged: Dict[str, Any] = {
        **parsed,
        "raw": raw,
        "unix_ms": unix_ms,
        "utc_time": utc_time,
        "layback_lat": layback_lat,
        "layback_lon": layback_lon,
        "layback_bearing_deg": bearing_deg,
        "layback_x_m": lb_x,
        "layback_y_m": lb_y,
    }
    with state_lock:
        last_parsed = merged

    write_csv_row(parsed, raw, unix_ms, utc_time)
    sse_broadcast({"type": "parsed", "data": merged})

    now_m = time.monotonic()
    if now_m - last_nvf_burst_mono >= NVF_MIN_INTERVAL_S:
        last_nvf_burst_mono = now_m
        send_nvf_burst(
            parsed["field_nt"],
            parsed["signal"],
            parsed["depth_m"],
            parsed["quality"],
        )


def open_csv_log_sim() -> Tuple[str, Path]:
    ensure_log_dir()
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    name = f"{LOG_PREFIX}{ts}{SIM_LOG_INFIX}{LOG_SUFFIX}"
    return name, LOG_DIR / name


def simulation_loop() -> None:
    """Emit synthetic sentences at ~4 Hz until stop_event (same pipeline as serial)."""
    global connected, simulating, reader_error, csv_file, csv_writer, current_log_name, last_nvf_burst_mono

    reader_error = None
    log_name, log_path = open_csv_log_sim()
    try:
        log_fp = open(log_path, "a", newline="", encoding="utf-8")
        log_writer = csv.writer(log_fp)
        log_writer.writerow(CSV_COLUMNS)
        log_fp.flush()
    except Exception as e:
        reader_error = f"Cannot create simulation log file: {e}"
        stop_event.set()
        with state_lock:
            connected = False
            simulating = False
        sse_broadcast({"type": "status", "connected": False, "simulate": False, "error": reader_error})
        return

    with state_lock:
        current_log_name = log_name
        csv_file = log_fp
        csv_writer = log_writer
        connected = True
        simulating = True
        last_nvf_burst_mono = 0.0

    sse_broadcast(
        {
            "type": "status",
            "connected": True,
            "simulate": True,
            "log": log_name,
            "port": "SIMULATION",
            "message": "Synthetic Explorer sentences (no USB hardware).",
        }
    )

    while not stop_event.is_set():
        try:
            process_incoming_sentence(build_simulated_sentence())
        except Exception as e:
            reader_error = str(e)
            sse_broadcast(
                {
                    "type": "status",
                    "connected": True,
                    "simulate": True,
                    "serial_error": reader_error,
                }
            )
        stop_event.wait(0.25)

    try:
        if csv_file:
            csv_file.close()
    except Exception:
        pass
    with state_lock:
        csv_file = None
        csv_writer = None
        current_log_name = None
        connected = False
        simulating = False
    sse_broadcast({"type": "status", "connected": False, "simulate": False})


def serial_read_loop(port: str, baud: int) -> None:
    global ser, connected, reader_error, last_parsed, last_nvf_burst_mono, raw_lines, csv_file, csv_writer, current_log_name

    reader_error = None
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=1.0, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        configure_explorer_4hz(ser)
    except Exception as e:
        reader_error = str(e)
        stop_event.set()
        with state_lock:
            connected = False
        sse_broadcast({"type": "status", "connected": False, "error": reader_error})
        return

    log_name, log_path = open_csv_log()
    try:
        log_fp = open(log_path, "a", newline="", encoding="utf-8")
        log_writer = csv.writer(log_fp)
        log_writer.writerow(CSV_COLUMNS)
        log_fp.flush()
    except Exception as e:
        reader_error = f"Cannot create log file: {e}"
        stop_event.set()
        try:
            ser.close()
        except Exception:
            pass
        ser = None
        with state_lock:
            connected = False
        sse_broadcast({"type": "status", "connected": False, "error": reader_error})
        return

    with state_lock:
        current_log_name = log_name
        csv_file = log_fp
        csv_writer = log_writer
        connected = True

    sse_broadcast(
        {
            "type": "status",
            "connected": True,
            "log": log_name,
            "port": port,
            "baud": baud,
            "message": "Explorer 4 Hz cycling command sent.",
        }
    )

    while not stop_event.is_set() and ser and ser.is_open:
        try:
            line_b = ser.readline()
            if not line_b:
                continue
            raw = line_b.decode("utf-8", errors="replace").strip()
            if not raw:
                continue
            process_incoming_sentence(raw)
        except Exception as e:
            reader_error = str(e)
            sse_broadcast({"type": "status", "connected": True, "serial_error": reader_error})
            time.sleep(0.1)

    # cleanup serial + csv
    try:
        if ser and ser.is_open:
            ser.close()
    except Exception:
        pass
    ser = None
    try:
        if csv_file:
            csv_file.close()
    except Exception:
        pass
    with state_lock:
        csv_file = None
        csv_writer = None
        current_log_name = None
        connected = False
    sse_broadcast({"type": "status", "connected": False})


def list_serial_ports() -> List[Dict[str, Any]]:
    out: Dict[str, Dict[str, str]] = {}
    for p in list_ports.comports():
        dev = p.device
        out[dev] = {"device": dev, "description": p.description or "", "hwid": p.hwid or ""}
    by_id = Path("/dev/serial/by-id")
    if by_id.is_dir():
        for link in sorted(by_id.iterdir()):
            if link.is_symlink() or link.is_file():
                try:
                    target = os.path.realpath(str(link))
                    if target not in out:
                        out[target] = {"device": target, "description": link.name, "hwid": "by-id"}
                    else:
                        out[target]["by_id"] = link.name
                except Exception:
                    pass
    return sorted(out.values(), key=lambda x: x["device"])


def safe_log_name(name: str) -> Optional[Path]:
    if not name or "/" in name or "\\" in name or name.startswith("."):
        return None
    if not (name.startswith(LOG_PREFIX) and name.endswith(LOG_SUFFIX)):
        return None
    path = (LOG_DIR / name).resolve()
    try:
        path.relative_to(LOG_DIR.resolve())
    except ValueError:
        return None
    return path


# -----------------------------------------------------------------------------
# Flask routes
# -----------------------------------------------------------------------------
@app.route("/register_service")
def register_service():
    return jsonify(
        {
            "name": "Marine Magnetics Explorer",
            "description": "USB serial Explorer data with GPS; CSV logs and NAMED_VALUE_FLOAT to autopilot.",
            "icon": "mdi-magnet",
            "company": "Blue Robotics",
            "version": "1.0.0",
            "webpage": "https://github.com/vshie/BlueOS-Marine-Magnetics-Explorer",
            "api": "https://github.com/vshie/BlueOS-Marine-Magnetics-Explorer",
        }
    )


@app.route("/")
def index_page():
    return send_from_directory(app.static_folder, "index.html")


@app.route("/api/serial/ports")
def api_serial_ports():
    return jsonify({"ports": list_serial_ports()})


@app.route("/api/serial/device-ids", methods=["GET"])
def api_serial_device_ids():
    return jsonify({"devices": get_device_ids()})


@app.route("/api/settings", methods=["GET"])
def api_settings_get():
    s = load_settings()
    apply_layback_settings(s)
    return jsonify(s)


@app.route("/api/settings", methods=["POST"])
def api_settings_post():
    data = request.get_json(force=True, silent=True) or {}
    current_settings = load_settings()
    port = str(data.get("port", ""))
    baud = int(data.get("baud_rate", current_settings.get("baud_rate", 9600)))
    layback_x = float(data.get("layback_x_m", current_settings.get("layback_x_m", DEFAULT_LAYBACK_X_M)))
    layback_y = float(data.get("layback_y_m", current_settings.get("layback_y_m", DEFAULT_LAYBACK_Y_M)))
    if baud not in BAUD_CHOICES:
        return jsonify({"ok": False, "error": "invalid baud_rate"}), 400
    save_settings(port, baud, layback_x, layback_y)
    apply_layback_settings(
        {"layback_x_m": layback_x, "layback_y_m": layback_y}
    )
    return jsonify(
        {
            "ok": True,
            "port": port,
            "baud_rate": baud,
            "layback_x_m": layback_x,
            "layback_y_m": layback_y,
        }
    )


@app.route("/api/status")
def api_status():
    with state_lock:
        lines = list(raw_lines)
        lp = dict(last_parsed) if last_parsed else None
        gf = dict(gps_fix)
        cur = current_log_name
        nvf_ok = nvf_posts_ok
        nvf_st = nvf_last_status
        nvf_ep = nvf_last_endpoint
        conn = connected
        sim = simulating
        vid = vehicle_id
        err = reader_error
        bearing = gps_bearing_rad
        lb_x = layback_x_m
        lb_y = layback_y_m
    gps_age_s: Optional[float] = None
    if gf.get("last_ok_mono") is not None:
        gps_age_s = time.monotonic() - float(gf["last_ok_mono"])
    return jsonify(
        {
            "connected": conn,
            "simulation": sim,
            "vehicle_id": vid,
            "current_log": cur,
            "last_lines": lines,
            "last_parsed": lp,
            "gps": gf,
            "gps_age_s": gps_age_s,
            "layback_x_m": lb_x,
            "layback_y_m": lb_y,
            "layback_bearing_deg": (math.degrees(bearing) + 360.0) % 360.0 if bearing is not None else None,
            "nvf_posts_ok": nvf_ok,
            "nvf_last_status": nvf_st,
            "nvf_last_endpoint": nvf_ep,
            "reader_error": err,
        }
    )


@app.route("/api/connect", methods=["POST"])
def api_connect():
    global serial_thread, simulation_thread, gps_thread, stop_event, reader_error, last_nvf_burst_mono
    global gps_prev_position, gps_bearing_rad

    with state_lock:
        if connected:
            return jsonify({"ok": False, "error": "already connected"}), 400

    data = request.get_json(force=True, silent=True) or {}
    if bool(data.get("simulate")):
        stop_event.clear()
        reader_error = None
        last_nvf_burst_mono = 0.0
        gps_prev_position = None
        gps_bearing_rad = None
        serial_thread = None

        gps_thread = threading.Thread(target=gps_poll_loop, name="gps", daemon=True)
        gps_thread.start()
        simulation_thread = threading.Thread(target=simulation_loop, name="simulation", daemon=True)
        simulation_thread.start()

        time.sleep(0.2)
        with state_lock:
            err = reader_error
            ok = connected
        if err and not ok:
            stop_event.set()
            if gps_thread:
                gps_thread.join(timeout=2.0)
            if simulation_thread:
                simulation_thread.join(timeout=5.0)
            simulation_thread = None
            gps_thread = None
            return jsonify({"ok": False, "error": err}), 400

        return jsonify({"ok": True, "simulate": True})

    port = str(data.get("port", "")).strip()
    baud = int(data.get("baud_rate", load_settings()["baud_rate"]))
    if baud not in BAUD_CHOICES:
        return jsonify({"ok": False, "error": "invalid baud_rate"}), 400
    if not port:
        return jsonify({"ok": False, "error": "port required"}), 400

    save_settings(port, baud)

    stop_event.clear()
    reader_error = None
    last_nvf_burst_mono = 0.0
    gps_prev_position = None
    gps_bearing_rad = None
    simulation_thread = None

    gps_thread = threading.Thread(target=gps_poll_loop, name="gps", daemon=True)
    gps_thread.start()

    serial_thread = threading.Thread(target=serial_read_loop, args=(port, baud), name="serial", daemon=True)
    serial_thread.start()

    time.sleep(0.15)
    with state_lock:
        err = reader_error
        ok = connected
    if err and not ok:
        stop_event.set()
        if gps_thread:
            gps_thread.join(timeout=2.0)
        if serial_thread:
            serial_thread.join(timeout=2.0)
        return jsonify({"ok": False, "error": err}), 400

    return jsonify({"ok": True, "port": port, "baud_rate": baud})


@app.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    global stop_event, serial_thread, simulation_thread, gps_thread

    stop_event.set()
    t_ser = serial_thread
    t_sim = simulation_thread
    t_gps = gps_thread
    if t_ser:
        t_ser.join(timeout=5.0)
    if t_sim:
        t_sim.join(timeout=5.0)
    if t_gps:
        t_gps.join(timeout=2.0)
    serial_thread = None
    simulation_thread = None
    gps_thread = None
    stop_event = threading.Event()
    return jsonify({"ok": True})


@app.route("/api/events")
def api_events():
    def gen():
        q: queue.Queue = queue.Queue(maxsize=100)
        with state_lock:
            sse_clients.append(q)
        try:
            init = {
                "type": "init",
                "baud_choices": BAUD_CHOICES,
                "settings": load_settings(),
            }
            yield f"data: {json.dumps(init)}\n\n"
            while True:
                try:
                    item = q.get(timeout=20.0)
                    yield item
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            with state_lock:
                if q in sse_clients:
                    sse_clients.remove(q)

    return Response(
        stream_with_context(gen()),
        mimetype="text/event-stream",
        # Do not set Connection (hop-by-hop); Waitress/WSGI rejects it (PEP 3333).
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


def _csv_line_count(path: Path) -> Optional[int]:
    try:
        size = path.stat().st_size
        if size > 50 * 1024 * 1024:
            return None
        with path.open("r", encoding="utf-8", errors="replace") as f:
            n = sum(1 for _ in f)
        return max(0, n - 1)
    except Exception:
        return None


@app.route("/api/logs")
def api_logs():
    ensure_log_dir()
    items = []
    with state_lock:
        cur = current_log_name
    for p in sorted(LOG_DIR.glob(f"{LOG_PREFIX}*{LOG_SUFFIX}"), key=lambda x: x.stat().st_mtime, reverse=True):
        st = p.stat()
        items.append(
            {
                "name": p.name,
                "size_bytes": st.st_size,
                "mtime": st.st_mtime,
                "row_count": _csv_line_count(p),
                "current": p.name == cur,
            }
        )
    return jsonify({"logs": items})


@app.route("/api/logs/<name>")
def api_log_download(name: str):
    path = safe_log_name(name)
    if not path or not path.is_file():
        return jsonify({"ok": False, "error": "not found"}), 404
    return send_file(
        path,
        mimetype="text/csv",
        as_attachment=True,
        download_name=name,
        cache_timeout=0,
    )


@app.route("/api/logs/<name>", methods=["DELETE"])
def api_log_delete(name: str):
    path = safe_log_name(name)
    if not path or not path.is_file():
        return jsonify({"ok": False, "error": "not found"}), 404
    with state_lock:
        if current_log_name == name:
            return jsonify({"ok": False, "error": "cannot delete active log"}), 400
    try:
        path.unlink()
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


def main() -> None:
    ensure_log_dir()
    apply_layback_settings(load_settings())
    port = int(os.environ.get("PORT", "9091"))
    print(f"Marine Magnetics Explorer extension listening on 0.0.0.0:{port}", flush=True)
    serve(app, host="0.0.0.0", port=port, threads=8)


if __name__ == "__main__":
    main()
