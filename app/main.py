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

# Explorer default per manufacturer: 9600 8N1
BAUD_CHOICES = [1200, 2400, 4800, 9600, 19200]
DEFAULT_LAYBACK_X_M = 0.0  # positive = starboard, negative = port
DEFAULT_LAYBACK_Y_M = -5.0  # positive = forward, negative = behind / layback

# Max ~4 Hz bursts to autopilot (6 NVF per burst with deviation + alarm)
NVF_MIN_INTERVAL_S = 0.25
EXPLORER_CYCLE_OFF_COMMAND = b"0"  # Manual: "0 Stop cycling"
EXPLORER_4HZ_COMMAND = b"1"  # Manual: "1 Set 250ms cycle time" (4 Hz)
EXPLORER_CYCLE_2HZ_COMMAND = b"2"  # Manual: "2 Set 500ms cycle time" (2 Hz)
EXPLORER_CYCLE_1HZ_COMMAND = b"3"  # Manual: "3 Set 1000ms cycle time" (1 Hz)
EXPLORER_AUTOTUNE_ON_COMMAND = b"X"  # Manual: "X Auto-tune on"
EXPLORER_AUTOTUNE_OFF_COMMAND = b"Y"  # Manual: "Y Auto-tune off"
EXPLORER_INFO_COMMAND = b"I"  # Manual: "I" returns towfish info incl. serial
EXPLORER_TIME_SYNC_PREFIX = b"T"  # Followed by 11 digits JJJYYHHMMSS, no CR

SAMPLE_RATE_CHOICES = ("0", "1", "2", "3")
SAMPLE_RATE_LABELS = {"0": "Off", "1": "4 Hz", "2": "2 Hz", "3": "1 Hz"}
SAMPLE_RATE_BYTES = {
    "0": EXPLORER_CYCLE_OFF_COMMAND,
    "1": EXPLORER_4HZ_COMMAND,
    "2": EXPLORER_CYCLE_2HZ_COMMAND,
    "3": EXPLORER_CYCLE_1HZ_COMMAND,
}
DEFAULT_SAMPLE_RATE = "1"

MOVING_AVG_CHOICES = (4, 10, 15, 30, 60, 120)
MAX_MOVING_AVG_S = MOVING_AVG_CHOICES[-1]
DEFAULT_AVG_WINDOW_S = 15
DEFAULT_ALARM_THRESHOLD_NT = 4.0

INFO_READ_TIMEOUT_S = 1.5
RAW_MAG_PREFIX = "RawMag"
RAW_GPS_PREFIX = "RawGPS"
RAW_FILE_SUFFIX = ".txt"

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
    "vessel_lat",
    "vessel_lon",
    "vessel_alt_m",
    "vessel_gps_age_ms",
    "vessel_bearing_deg",
    "towfish_lat",
    "towfish_lon",
    "layback_x_m",
    "layback_y_m",
    "moving_avg_nt",
    "deviation_nt",
    "avg_window_s",
    "alarm",
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
stop_event = threading.Event()
serial_thread: Optional[threading.Thread] = None
gps_thread: Optional[threading.Thread] = None

ser: Optional[serial.Serial] = None
csv_file = None
csv_writer: Optional[csv.writer] = None
current_log_name: Optional[str] = None
raw_mag_file = None
raw_gps_file = None
current_raw_mag_name: Optional[str] = None
current_raw_gps_name: Optional[str] = None

raw_lines: deque = deque(maxlen=10)
vehicle_id = MAVLINK_VEHICLE_ID

gps_fix: Dict[str, Any] = {
    "lat": None,
    "lon": None,
    "alt_m": None,
    "last_ok_mono": None,
    "last_error": None,
}
# AHRS-derived heading from VFR_HUD (degrees) is the single source of truth
# for the towfish layback bearing. Cached as radians for math compatibility.
gps_bearing_rad: Optional[float] = None
vfr_hud: Dict[str, Any] = {
    "groundspeed_m_s": None,
    "heading_deg": None,
    "last_ok_mono": None,
}
gps_raw: Dict[str, Any] = {
    "fix_type": None,
    "satellites_visible": None,
    "hdop": None,
    "vdop": None,
    "last_ok_mono": None,
}
layback_x_m = DEFAULT_LAYBACK_X_M
layback_y_m = DEFAULT_LAYBACK_Y_M

sample_rate: str = DEFAULT_SAMPLE_RATE
avg_window_s: int = DEFAULT_AVG_WINDOW_S
alarm_threshold_nt: float = DEFAULT_ALARM_THRESHOLD_NT
towfish_serial: Optional[str] = None
last_time_sync_str: Optional[str] = None
last_time_sync_unix: Optional[int] = None

# (monotonic_ts, field_nt) pairs, pruned to <= MAX_MOVING_AVG_S worth of data
_field_window: deque = deque()
last_moving_avg_nt: Optional[float] = None
last_deviation_nt: Optional[float] = None
last_alarm: bool = False

nvf_posts_ok = 0
nvf_last_status = "idle"
nvf_last_endpoint: Optional[str] = None
nvf_last_sent: Dict[str, float] = {}
last_parsed: Optional[Dict[str, Any]] = None
last_nvf_burst_mono = 0.0

reader_error: Optional[str] = None


def ensure_log_dir() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)


def state_path() -> Path:
    return LOG_DIR / STATE_FILENAME


def _default_settings() -> Dict[str, Any]:
    return {
        "port": "",
        "baud_rate": 9600,
        "layback_x_m": DEFAULT_LAYBACK_X_M,
        "layback_y_m": DEFAULT_LAYBACK_Y_M,
        "sample_rate": DEFAULT_SAMPLE_RATE,
        "avg_window_s": DEFAULT_AVG_WINDOW_S,
        "alarm_threshold_nt": DEFAULT_ALARM_THRESHOLD_NT,
    }


def _coerce_sample_rate(v: Any) -> str:
    s = str(v).strip()
    return s if s in SAMPLE_RATE_CHOICES else DEFAULT_SAMPLE_RATE


def _coerce_avg_window(v: Any) -> int:
    try:
        n = int(v)
    except Exception:
        return DEFAULT_AVG_WINDOW_S
    return n if n in MOVING_AVG_CHOICES else DEFAULT_AVG_WINDOW_S


def _coerce_threshold(v: Any) -> float:
    try:
        n = float(v)
    except Exception:
        return DEFAULT_ALARM_THRESHOLD_NT
    if not math.isfinite(n) or n < 0:
        return DEFAULT_ALARM_THRESHOLD_NT
    return n


def load_settings() -> Dict[str, Any]:
    ensure_log_dir()
    p = state_path()
    if not p.is_file():
        return _default_settings()
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
            "sample_rate": _coerce_sample_rate(data.get("sample_rate", DEFAULT_SAMPLE_RATE)),
            "avg_window_s": _coerce_avg_window(data.get("avg_window_s", DEFAULT_AVG_WINDOW_S)),
            "alarm_threshold_nt": _coerce_threshold(
                data.get("alarm_threshold_nt", DEFAULT_ALARM_THRESHOLD_NT)
            ),
        }
    except Exception:
        return _default_settings()


def save_settings(
    port: str,
    baud_rate: int,
    layback_x_m: Optional[float] = None,
    layback_y_m: Optional[float] = None,
    sample_rate_v: Optional[str] = None,
    avg_window_v: Optional[int] = None,
    alarm_threshold_v: Optional[float] = None,
) -> None:
    ensure_log_dir()
    if baud_rate not in BAUD_CHOICES:
        baud_rate = 9600
    current = load_settings()
    if layback_x_m is None:
        layback_x_m = float(current.get("layback_x_m", DEFAULT_LAYBACK_X_M))
    if layback_y_m is None:
        layback_y_m = float(current.get("layback_y_m", DEFAULT_LAYBACK_Y_M))
    if sample_rate_v is None:
        sample_rate_v = _coerce_sample_rate(current.get("sample_rate", DEFAULT_SAMPLE_RATE))
    else:
        sample_rate_v = _coerce_sample_rate(sample_rate_v)
    if avg_window_v is None:
        avg_window_v = _coerce_avg_window(current.get("avg_window_s", DEFAULT_AVG_WINDOW_S))
    else:
        avg_window_v = _coerce_avg_window(avg_window_v)
    if alarm_threshold_v is None:
        alarm_threshold_v = _coerce_threshold(
            current.get("alarm_threshold_nt", DEFAULT_ALARM_THRESHOLD_NT)
        )
    else:
        alarm_threshold_v = _coerce_threshold(alarm_threshold_v)
    with state_path().open("w", encoding="utf-8") as f:
        json.dump(
            {
                "port": port,
                "baud_rate": baud_rate,
                "layback_x_m": float(layback_x_m),
                "layback_y_m": float(layback_y_m),
                "sample_rate": sample_rate_v,
                "avg_window_s": int(avg_window_v),
                "alarm_threshold_nt": float(alarm_threshold_v),
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


def apply_runtime_settings(settings: Dict[str, Any]) -> None:
    """Apply persisted runtime settings (sample rate, avg window, alarm threshold)."""
    global sample_rate, avg_window_s, alarm_threshold_nt
    with state_lock:
        sample_rate = _coerce_sample_rate(settings.get("sample_rate", DEFAULT_SAMPLE_RATE))
        avg_window_s = _coerce_avg_window(settings.get("avg_window_s", DEFAULT_AVG_WINDOW_S))
        alarm_threshold_nt = _coerce_threshold(
            settings.get("alarm_threshold_nt", DEFAULT_ALARM_THRESHOLD_NT)
        )


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


def calculate_layback_position(
    gnss_lat: Optional[float],
    gnss_lon: Optional[float],
    offset_x_m: float,
    offset_y_m: float,
    bearing: Optional[float],
) -> Tuple[Optional[float], Optional[float]]:
    """Estimate towfish position using vessel GPS, lateral/longitudinal offset, and AHRS heading.

    Uses the BlueBoatExplorerMag.py convention:
    x is starboard/port, y is forward/back. Instead of pyproj UTM transforms, use a local
    meters-per-degree approximation, which is accurate enough for short layback offsets.
    Bearing is sourced from VFR_HUD (AHRS heading), not from successive position deltas.
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


def format_explorer_time(dt_utc: datetime) -> bytes:
    """Format a datetime as the Explorer T-command sequence: T + JJJ + YY + HHMMSS.

    The magnetometer prompts for 11 digits after the leading 'T'; per the manual the
    order is 3-digit Julian day, 2-digit year, then HHMMSS (UTC). No CR is needed —
    the sensor latches as soon as the eleventh digit lands. We emit all 12 bytes in
    a single write so the device sees them as a contiguous burst.
    """
    julian_day = dt_utc.timetuple().tm_yday
    s = "T{:03d}{:02d}{:02d}{:02d}{:02d}".format(
        julian_day,
        dt_utc.year % 100,
        dt_utc.hour,
        dt_utc.minute,
        dt_utc.second,
    )
    return s.encode("ascii")


def _nmea_checksum(body: str) -> str:
    """XOR every byte of `body` (the part between '$' and '*') as 2 upper-hex digits."""
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"{cs:02X}"


def _nmea_lat(lat: Optional[float]) -> Tuple[str, str]:
    if lat is None or not math.isfinite(lat):
        return "", ""
    hemi = "N" if lat >= 0 else "S"
    a = abs(lat)
    deg = int(a)
    minutes = (a - deg) * 60.0
    return f"{deg:02d}{minutes:07.4f}", hemi


def _nmea_lon(lon: Optional[float]) -> Tuple[str, str]:
    if lon is None or not math.isfinite(lon):
        return "", ""
    hemi = "E" if lon >= 0 else "W"
    a = abs(lon)
    deg = int(a)
    minutes = (a - deg) * 60.0
    return f"{deg:03d}{minutes:07.4f}", hemi


def _nmea_sentence(body: str) -> str:
    return f"${body}*{_nmea_checksum(body)}"


def build_gprmc(
    utc: datetime,
    lat: Optional[float],
    lon: Optional[float],
    speed_kn: Optional[float],
    course_deg: Optional[float],
) -> str:
    lat_s, lat_h = _nmea_lat(lat)
    lon_s, lon_h = _nmea_lon(lon)
    status = "A" if lat_s and lon_s else "V"
    spd = "" if speed_kn is None or not math.isfinite(speed_kn) else f"{speed_kn:.2f}"
    crs = "" if course_deg is None or not math.isfinite(course_deg) else f"{course_deg:.2f}"
    body = "GPRMC,{t},{st},{lat},{lat_h},{lon},{lon_h},{spd},{crs},{date},,,A".format(
        t=utc.strftime("%H%M%S.000"),
        st=status,
        lat=lat_s,
        lat_h=lat_h,
        lon=lon_s,
        lon_h=lon_h,
        spd=spd,
        crs=crs,
        date=utc.strftime("%d%m%y"),
    )
    return _nmea_sentence(body)


def build_gpgga(
    utc: datetime,
    lat: Optional[float],
    lon: Optional[float],
    fix_quality: int,
    sats: int,
    hdop: Optional[float],
    alt_m: Optional[float],
) -> str:
    lat_s, lat_h = _nmea_lat(lat)
    lon_s, lon_h = _nmea_lon(lon)
    hd = "99.9" if hdop is None or not math.isfinite(hdop) else f"{hdop:.1f}"
    alt = "" if alt_m is None or not math.isfinite(alt_m) else f"{alt_m:.1f}"
    body = "GPGGA,{t},{lat},{lat_h},{lon},{lon_h},{q},{n:02d},{h},{a},M,0.0,M,,0000".format(
        t=utc.strftime("%H%M%S.000"),
        lat=lat_s,
        lat_h=lat_h,
        lon=lon_s,
        lon_h=lon_h,
        q=int(fix_quality),
        n=int(sats),
        h=hd,
        a=alt,
    )
    return _nmea_sentence(body)


def build_gpgsa(fix_type: int, hdop: Optional[float], vdop: Optional[float]) -> str:
    pdop = None
    if hdop is not None and vdop is not None and math.isfinite(hdop) and math.isfinite(vdop):
        pdop = math.sqrt(hdop * hdop + vdop * vdop)
    pd = "99.9" if pdop is None else f"{pdop:.1f}"
    hd = "99.9" if hdop is None or not math.isfinite(hdop) else f"{hdop:.1f}"
    vd = "99.9" if vdop is None or not math.isfinite(vdop) else f"{vdop:.1f}"
    ft = max(1, min(3, int(fix_type) if fix_type else 1))
    body = f"GPGSA,A,{ft},,,,,,,,,,,,,{pd},{hd},{vd}"
    return _nmea_sentence(body)


def build_gpgsv(sats_visible: int) -> str:
    n = max(0, int(sats_visible))
    body = f"GPGSV,1,1,{n:02d},,,,"
    return _nmea_sentence(body)


def synthesise_nmea_block(
    utc: datetime,
    lat: Optional[float],
    lon: Optional[float],
    alt_m: Optional[float],
    groundspeed_m_s: Optional[float],
    course_deg: Optional[float],
    fix_type: Optional[int],
    sats_visible: Optional[int],
    hdop: Optional[float],
    vdop: Optional[float],
) -> str:
    """Build a 4-sentence NMEA block (RMC/GGA/GSA/GSV) matching the surveyor example."""
    speed_kn = None
    if groundspeed_m_s is not None and math.isfinite(groundspeed_m_s):
        speed_kn = groundspeed_m_s * 1.943844
    fix_q = 0
    ft = fix_type if fix_type is not None else 0
    if ft >= 2:
        fix_q = 1
    sats_n = sats_visible if sats_visible is not None else 0
    return "\n".join(
        [
            build_gprmc(utc, lat, lon, speed_kn, course_deg),
            build_gpgga(utc, lat, lon, fix_q, sats_n, hdop, alt_m),
            build_gpgsa(ft, hdop, vdop),
            build_gpgsv(sats_n),
        ]
    )


def compute_moving_average(window_s: int) -> Tuple[Optional[float], Optional[float], int]:
    """Mean and deviation over the last `window_s` seconds of mag samples.

    Caller must hold `state_lock` since `_field_window` is mutated by the serial thread.
    Returns (avg, deviation, n_samples). deviation is field - avg of the most recent sample.
    """
    if not _field_window:
        return None, None, 0
    cutoff = _field_window[-1][0] - max(1, int(window_s))
    total = 0.0
    n = 0
    for ts, val in _field_window:
        if ts >= cutoff:
            total += val
            n += 1
    if n == 0:
        return None, None, 0
    avg = total / n
    latest = _field_window[-1][1]
    return avg, latest - avg, n


def _prune_field_window() -> None:
    """Drop samples older than MAX_MOVING_AVG_S relative to the newest sample.

    Caller must hold `state_lock`. We anchor pruning on the newest sample's timestamp
    (rather than `time.monotonic()` directly) so a brief data dropout does not erase
    history before the next sample arrives.
    """
    if not _field_window:
        return
    cutoff = _field_window[-1][0] - MAX_MOVING_AVG_S
    while _field_window and _field_window[0][0] < cutoff:
        _field_window.popleft()


def build_nvf_payload(name: str, value: float) -> Dict[str, Any]:
    """Match the Odometer extension's working NAMED_VALUE_FLOAT payload exactly."""
    name_array: List[str] = []
    for i in range(10):
        if i < len(name):
            name_array.append(name[i])
        else:
            name_array.append("\u0000")
    return {
        "header": {"system_id": 255, "component_id": 0, "sequence": 0},
        "message": {
            "type": "NAMED_VALUE_FLOAT",
            "time_boot_ms": 0,
            "value": float(value),
            "name": name_array,
        },
    }


def send_named_value_float(name: str, value: float) -> Tuple[bool, Optional[str]]:
    """POST NAMED_VALUE_FLOAT to Mavlink2Rest; try endpoints in order."""
    payload = build_nvf_payload(name, value)
    for post_url in MAVLINK_POST_ENDPOINTS:
        try:
            r = requests.post(post_url, json=payload, timeout=2.0)
            if r.status_code == 200:
                return True, post_url
        except Exception:
            continue
    return False, None


def send_nvf_burst(
    field_nt: float,
    signal: int,
    depth_m: Optional[float],
    quality: Optional[int],
    deviation_nt: Optional[float],
    alarm: bool,
) -> None:
    """Send six NVFs; update counters and SSE."""
    global nvf_posts_ok, nvf_last_status, nvf_last_endpoint, nvf_last_sent

    depth_val = float(depth_m) if depth_m is not None else 0.0
    qual_val = float(quality) if quality is not None else 0.0
    dev_val = float(deviation_nt) if deviation_nt is not None and math.isfinite(deviation_nt) else 0.0

    specs = [
        ("MAG_NT", float(field_nt)),
        ("MAG_SIG", float(signal)),
        ("MAG_DEPTH", depth_val),
        ("MAG_QUAL", qual_val),
        ("MAG_DEV", dev_val),
        ("MAG_ALRM", 1.0 if alarm else 0.0),
    ]
    last_url: Optional[str] = None
    sent: Dict[str, float] = {}
    for n, v in specs:
        ok, url = send_named_value_float(n, v)
        if ok:
            last_url = url
            sent[n] = v
            with state_lock:
                nvf_posts_ok += 1
        else:
            with state_lock:
                nvf_last_status = f"failed:{n}"
                nvf_last_endpoint = None
                nvf_last_sent = sent
            sse_broadcast(
                {
                    "type": "mavlink_ack",
                    "ok": False,
                    "name": n,
                    "sent": sent,
                }
            )
            return

    with state_lock:
        nvf_last_status = "ok"
        nvf_last_endpoint = last_url
        nvf_last_sent = sent
    sse_broadcast(
        {
            "type": "mavlink_ack",
            "ok": True,
            "endpoint": last_url,
            "sent": sent,
        }
    )


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


def _mavlink_msg_url(vid: int, name: str) -> str:
    return f"{MAVLINK_BASE.rstrip('/')}/vehicles/{vid}/components/1/messages/{name}"


def _poll_global_position_int(url: str) -> None:
    try:
        r = requests.get(url, timeout=1.0)
        if r.status_code != 200:
            with state_lock:
                gps_fix["last_error"] = f"HTTP {r.status_code}"
            return
        body = r.json()
        msg = body.get("message") or body
        lat_e7 = msg.get("lat")
        lon_e7 = msg.get("lon")
        alt_e7 = msg.get("alt")
        if lat_e7 is None or lon_e7 is None:
            with state_lock:
                gps_fix["last_error"] = "no lat/lon in message"
            return
        lat_deg = float(lat_e7) / 1e7
        lon_deg = float(lon_e7) / 1e7
        alt_m = float(alt_e7) / 1e7 if alt_e7 is not None else None
        with state_lock:
            gps_fix["lat"] = lat_deg
            gps_fix["lon"] = lon_deg
            gps_fix["alt_m"] = alt_m
            gps_fix["last_ok_mono"] = time.monotonic()
            gps_fix["last_error"] = None
        sse_broadcast({"type": "gps", "lat": lat_deg, "lon": lon_deg, "alt_m": alt_m})
    except Exception as e:
        with state_lock:
            gps_fix["last_error"] = str(e)


def _poll_vfr_hud(url: str) -> None:
    """Authoritative source for groundspeed and AHRS heading (used for layback bearing)."""
    global gps_bearing_rad
    try:
        r = requests.get(url, timeout=1.0)
        if r.status_code != 200:
            return
        body = r.json()
        msg = body.get("message") or body
        gs = msg.get("groundspeed")
        hdg = msg.get("heading")
        with state_lock:
            if gs is not None:
                vfr_hud["groundspeed_m_s"] = float(gs)
            if hdg is not None:
                hdg_deg = float(hdg)
                vfr_hud["heading_deg"] = hdg_deg
                gps_bearing_rad = math.radians(hdg_deg)
            vfr_hud["last_ok_mono"] = time.monotonic()
    except Exception:
        pass


def _poll_gps_raw_int(url: str) -> None:
    """Source for fix_type / sat count / HDOP / VDOP used in synthesised NMEA."""
    try:
        r = requests.get(url, timeout=1.0)
        if r.status_code != 200:
            return
        body = r.json()
        msg = body.get("message") or body
        eph = msg.get("eph")
        epv = msg.get("epv")
        with state_lock:
            ft = msg.get("fix_type")
            if ft is not None:
                gps_raw["fix_type"] = int(ft)
            sv = msg.get("satellites_visible")
            if sv is not None:
                gps_raw["satellites_visible"] = int(sv)
            if eph is not None:
                gps_raw["hdop"] = float(eph) / 100.0
            if epv is not None:
                gps_raw["vdop"] = float(epv) / 100.0
            gps_raw["last_ok_mono"] = time.monotonic()
    except Exception:
        pass


def gps_poll_loop() -> None:
    global vehicle_id
    vid = detect_vehicle_id()
    with state_lock:
        vehicle_id = vid
    pos_url = _mavlink_msg_url(vid, "GLOBAL_POSITION_INT")
    vfr_url = _mavlink_msg_url(vid, "VFR_HUD")
    raw_url = _mavlink_msg_url(vid, "GPS_RAW_INT")
    while not stop_event.is_set():
        _poll_global_position_int(pos_url)
        _poll_vfr_hud(vfr_url)
        _poll_gps_raw_int(raw_url)
        stop_event.wait(0.5)


def open_csv_log() -> Tuple[str, Path]:
    ensure_log_dir()
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    name = f"{LOG_PREFIX}{ts}{LOG_SUFFIX}"
    path = LOG_DIR / name
    return name, path


def raw_log_paths(start_dt_utc: datetime) -> Tuple[Tuple[str, Path], Tuple[str, Path]]:
    """Per-session raw NMEA + raw mag filenames (date-stamped from session start)."""
    ensure_log_dir()
    date_s = start_dt_utc.strftime("%Y-%m-%d")
    mag_name = f"{RAW_MAG_PREFIX}{date_s}{RAW_FILE_SUFFIX}"
    gps_name = f"{RAW_GPS_PREFIX}{date_s}{RAW_FILE_SUFFIX}"
    return (mag_name, LOG_DIR / mag_name), (gps_name, LOG_DIR / gps_name)


def explorer_send_info_and_capture(serial_conn: serial.Serial) -> Optional[str]:
    """Send 'I' and capture any response text within a short idle window.

    Returns the captured text (stripped, multi-line OK) or None if no response.
    The Explorer is idle until cycling is started, so we use this window before the
    sample-rate command is sent. Lines beginning with '*' (a cycling sentence) are
    ignored to avoid contaminating the serial number capture.
    """
    try:
        existing_timeout = serial_conn.timeout
        serial_conn.reset_input_buffer()
        serial_conn.write(EXPLORER_INFO_COMMAND)
        serial_conn.flush()
        deadline = time.monotonic() + INFO_READ_TIMEOUT_S
        chunks: List[str] = []
        serial_conn.timeout = 0.2
        while time.monotonic() < deadline:
            buf = serial_conn.read(256)
            if buf:
                try:
                    chunks.append(buf.decode("utf-8", errors="replace"))
                except Exception:
                    pass
            elif chunks:
                break
        serial_conn.timeout = existing_timeout
    except Exception:
        return None
    text = "".join(chunks).strip()
    if not text:
        return None
    # Drop any cycling sentences that may have slipped in (begin with '*')
    cleaned_lines = []
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("*"):
            continue
        cleaned_lines.append(s)
    return "\n".join(cleaned_lines) if cleaned_lines else None


def configure_explorer_sample_rate(serial_conn: serial.Serial, rate_key: str) -> None:
    """Send the persisted sample-rate byte (0/1/2/3) on each serial connection."""
    cmd = SAMPLE_RATE_BYTES.get(rate_key, EXPLORER_4HZ_COMMAND)
    serial_conn.write(cmd)
    serial_conn.flush()


def send_explorer_command(cmd: bytes) -> Tuple[bool, str]:
    """Write Explorer ASCII command bytes to the live serial port."""
    with state_lock:
        local_ser = ser
    if not local_ser or not local_ser.is_open:
        return False, "Serial port is not open"
    try:
        local_ser.write(cmd)
        local_ser.flush()
        return True, "ok"
    except Exception as e:
        return False, f"Serial write failed: {e}"


def write_csv_row(
    parsed: Dict[str, Any],
    raw: str,
    unix_ms: int,
    utc_time: str,
    moving_avg: Optional[float],
    deviation: Optional[float],
    avg_window: int,
    alarm: bool,
) -> None:
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
    towfish_lat, towfish_lon = calculate_layback_position(lat, lon, lb_x, lb_y, bearing)
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
        bearing_deg,
        towfish_lat if towfish_lat is not None else "",
        towfish_lon if towfish_lon is not None else "",
        lb_x,
        lb_y,
        f"{moving_avg:.4f}" if moving_avg is not None else "",
        f"{deviation:.4f}" if deviation is not None else "",
        avg_window,
        1 if alarm else 0,
        raw,
    ]
    with state_lock:
        if csv_writer is None:
            return
        csv_writer.writerow(row)
        if csv_file:
            csv_file.flush()


def write_raw_logs(raw_mag: str, utc_now: datetime) -> None:
    """Append the raw mag sentence and a synthesised NMEA block to per-session text files.

    Each magnetometer reception triggers a 1:1 line in RawMag<date>.txt and a 4-sentence
    NMEA block in RawGPS<date>.txt, so the two files share an identical row count.
    """
    with state_lock:
        mag_fp = raw_mag_file
        gps_fp = raw_gps_file
        lat = gps_fix["lat"]
        lon = gps_fix["lon"]
        alt_m = gps_fix["alt_m"]
        gs = vfr_hud.get("groundspeed_m_s")
        hd = vfr_hud.get("heading_deg")
        ft = gps_raw.get("fix_type")
        sv = gps_raw.get("satellites_visible")
        hdop = gps_raw.get("hdop")
        vdop = gps_raw.get("vdop")
    try:
        if mag_fp:
            mag_fp.write(raw_mag + "\n")
            mag_fp.flush()
    except Exception:
        pass
    try:
        if gps_fp:
            block = synthesise_nmea_block(
                utc_now, lat, lon, alt_m, gs, hd, ft, sv, hdop, vdop
            )
            gps_fp.write(block + "\n\n")
            gps_fp.flush()
    except Exception:
        pass


def process_incoming_sentence(raw: str) -> None:
    """Handle one raw line: UI feed, parse, CSV row, optional NAMED_VALUE_FLOAT burst."""
    global last_parsed, last_nvf_burst_mono
    global last_moving_avg_nt, last_deviation_nt, last_alarm

    raw = raw.strip()
    if not raw:
        return
    now_dt = datetime.now(timezone.utc)
    unix_ms = int(now_dt.timestamp() * 1000)
    utc_time = now_dt.strftime("%Y-%m-%dT%H:%M:%SZ")

    with state_lock:
        raw_lines.append(raw)
    sse_broadcast({"type": "raw_sentence", "line": raw})

    parsed = parse_sentence(raw)
    if not parsed:
        return

    field_v = float(parsed["field_nt"])
    now_m = time.monotonic()

    with state_lock:
        _field_window.append((now_m, field_v))
        _prune_field_window()
        win_s = avg_window_s
        avg_v, dev_v, n_samples = compute_moving_average(win_s)
        threshold = alarm_threshold_nt
        lat = gps_fix["lat"]
        lon = gps_fix["lon"]
        alt_m = gps_fix["alt_m"]
        bearing = gps_bearing_rad
        lb_x = layback_x_m
        lb_y = layback_y_m

    alarm_now = bool(
        dev_v is not None
        and n_samples >= 2
        and math.isfinite(dev_v)
        and abs(dev_v) > float(threshold)
    )

    with state_lock:
        last_moving_avg_nt = avg_v
        last_deviation_nt = dev_v
        last_alarm = alarm_now

    towfish_lat, towfish_lon = calculate_layback_position(lat, lon, lb_x, lb_y, bearing)
    bearing_deg = None
    if bearing is not None:
        bearing_deg = (math.degrees(bearing) + 360.0) % 360.0

    merged: Dict[str, Any] = {
        **parsed,
        "raw": raw,
        "unix_ms": unix_ms,
        "utc_time": utc_time,
        "vessel_lat": lat,
        "vessel_lon": lon,
        "vessel_alt_m": alt_m,
        "vessel_bearing_deg": bearing_deg,
        "towfish_lat": towfish_lat,
        "towfish_lon": towfish_lon,
        "layback_x_m": lb_x,
        "layback_y_m": lb_y,
        "moving_avg_nt": avg_v,
        "deviation_nt": dev_v,
        "avg_window_s": win_s,
        "alarm": alarm_now,
        "alarm_threshold_nt": threshold,
    }
    with state_lock:
        last_parsed = merged

    write_csv_row(parsed, raw, unix_ms, utc_time, avg_v, dev_v, win_s, alarm_now)
    write_raw_logs(raw, now_dt)
    sse_broadcast({"type": "parsed", "data": merged})

    if now_m - last_nvf_burst_mono >= NVF_MIN_INTERVAL_S:
        last_nvf_burst_mono = now_m
        send_nvf_burst(
            parsed["field_nt"],
            parsed["signal"],
            parsed["depth_m"],
            parsed["quality"],
            dev_v,
            alarm_now,
        )


def serial_read_loop(port: str, baud: int) -> None:
    global ser, connected, reader_error, last_parsed, last_nvf_burst_mono, raw_lines
    global csv_file, csv_writer, current_log_name
    global raw_mag_file, raw_gps_file, current_raw_mag_name, current_raw_gps_name
    global towfish_serial

    reader_error = None
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=1.0,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
    except Exception as e:
        reader_error = str(e)
        stop_event.set()
        with state_lock:
            connected = False
        sse_broadcast({"type": "status", "connected": False, "error": reader_error})
        return

    # Capture towfish info while the device is still idle, then start cycling.
    serial_info = explorer_send_info_and_capture(ser)
    with state_lock:
        towfish_serial = serial_info
        rate_key = sample_rate
    try:
        configure_explorer_sample_rate(ser, rate_key)
    except Exception as e:
        reader_error = f"Cannot set sample rate: {e}"

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

    session_start = datetime.now(timezone.utc)
    (raw_mag_name, raw_mag_path), (raw_gps_name, raw_gps_path) = raw_log_paths(session_start)
    raw_mag_fp = None
    raw_gps_fp = None
    try:
        raw_mag_fp = open(raw_mag_path, "a", encoding="utf-8")
        raw_gps_fp = open(raw_gps_path, "a", encoding="utf-8")
    except Exception as e:
        reader_error = f"Cannot create raw text logs: {e}"
        try:
            if raw_mag_fp:
                raw_mag_fp.close()
        except Exception:
            pass
        try:
            if raw_gps_fp:
                raw_gps_fp.close()
        except Exception:
            pass
        try:
            log_fp.close()
        except Exception:
            pass
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
        raw_mag_file = raw_mag_fp
        raw_gps_file = raw_gps_fp
        current_raw_mag_name = raw_mag_name
        current_raw_gps_name = raw_gps_name
        connected = True
        # Reset deviation/alarm state when (re)connecting.
        _field_window.clear()

    sse_broadcast(
        {
            "type": "status",
            "connected": True,
            "log": log_name,
            "raw_mag_log": raw_mag_name,
            "raw_gps_log": raw_gps_name,
            "port": port,
            "baud": baud,
            "sample_rate": rate_key,
            "towfish_serial": serial_info,
            "message": f"Explorer init: I + cycle '{rate_key}' ({SAMPLE_RATE_LABELS.get(rate_key, '?')}).",
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

    # cleanup serial + csv + raw text logs
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
    try:
        if raw_mag_file:
            raw_mag_file.close()
    except Exception:
        pass
    try:
        if raw_gps_file:
            raw_gps_file.close()
    except Exception:
        pass
    with state_lock:
        csv_file = None
        csv_writer = None
        current_log_name = None
        raw_mag_file = None
        raw_gps_file = None
        current_raw_mag_name = None
        current_raw_gps_name = None
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
    """Validate that `name` refers to a CSV log file inside LOG_DIR."""
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


def safe_raw_log_name(name: str) -> Optional[Path]:
    """Validate that `name` refers to a per-session RawMag/RawGPS text file inside LOG_DIR."""
    if not name or "/" in name or "\\" in name or name.startswith("."):
        return None
    if not name.endswith(RAW_FILE_SUFFIX):
        return None
    if not (name.startswith(RAW_MAG_PREFIX) or name.startswith(RAW_GPS_PREFIX)):
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


@app.route("/widget")
@app.route("/widget/")
@app.route("/widget/field")
def widget_field():
    """Standalone, framework-free line-chart widget for embedding in Cockpit iframes."""
    return send_from_directory(app.static_folder, "widget.html")


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
    apply_runtime_settings(s)
    s["sample_rate_choices"] = list(SAMPLE_RATE_CHOICES)
    s["sample_rate_labels"] = dict(SAMPLE_RATE_LABELS)
    s["avg_window_choices"] = list(MOVING_AVG_CHOICES)
    return jsonify(s)


@app.route("/api/settings", methods=["POST"])
def api_settings_post():
    data = request.get_json(force=True, silent=True) or {}
    current_settings = load_settings()
    port = str(data.get("port", current_settings.get("port", "")))
    baud = int(data.get("baud_rate", current_settings.get("baud_rate", 9600)))
    layback_x = float(data.get("layback_x_m", current_settings.get("layback_x_m", DEFAULT_LAYBACK_X_M)))
    layback_y = float(data.get("layback_y_m", current_settings.get("layback_y_m", DEFAULT_LAYBACK_Y_M)))
    sample_rate_v = _coerce_sample_rate(
        data.get("sample_rate", current_settings.get("sample_rate", DEFAULT_SAMPLE_RATE))
    )
    avg_window_v = _coerce_avg_window(
        data.get("avg_window_s", current_settings.get("avg_window_s", DEFAULT_AVG_WINDOW_S))
    )
    alarm_threshold_v = _coerce_threshold(
        data.get("alarm_threshold_nt", current_settings.get("alarm_threshold_nt", DEFAULT_ALARM_THRESHOLD_NT))
    )
    if baud not in BAUD_CHOICES:
        return jsonify({"ok": False, "error": "invalid baud_rate"}), 400
    save_settings(
        port,
        baud,
        layback_x,
        layback_y,
        sample_rate_v=sample_rate_v,
        avg_window_v=avg_window_v,
        alarm_threshold_v=alarm_threshold_v,
    )
    apply_layback_settings({"layback_x_m": layback_x, "layback_y_m": layback_y})
    apply_runtime_settings(
        {
            "sample_rate": sample_rate_v,
            "avg_window_s": avg_window_v,
            "alarm_threshold_nt": alarm_threshold_v,
        }
    )
    return jsonify(
        {
            "ok": True,
            "port": port,
            "baud_rate": baud,
            "layback_x_m": layback_x,
            "layback_y_m": layback_y,
            "sample_rate": sample_rate_v,
            "avg_window_s": avg_window_v,
            "alarm_threshold_nt": alarm_threshold_v,
        }
    )


@app.route("/api/status")
def api_status():
    with state_lock:
        lines = list(raw_lines)
        lp = dict(last_parsed) if last_parsed else None
        gf = dict(gps_fix)
        vfr = dict(vfr_hud)
        graw = dict(gps_raw)
        cur = current_log_name
        cur_raw_mag = current_raw_mag_name
        cur_raw_gps = current_raw_gps_name
        nvf_ok = nvf_posts_ok
        nvf_st = nvf_last_status
        nvf_ep = nvf_last_endpoint
        nvf_sent = dict(nvf_last_sent)
        conn = connected
        vid = vehicle_id
        err = reader_error
        bearing = gps_bearing_rad
        lb_x = layback_x_m
        lb_y = layback_y_m
        sr = sample_rate
        aw = avg_window_s
        thr = alarm_threshold_nt
        ts_serial = towfish_serial
        last_avg = last_moving_avg_nt
        last_dev = last_deviation_nt
        last_alm = last_alarm
        ts_str = last_time_sync_str
        ts_unix = last_time_sync_unix
    gps_age_s: Optional[float] = None
    if gf.get("last_ok_mono") is not None:
        gps_age_s = time.monotonic() - float(gf["last_ok_mono"])
    return jsonify(
        {
            "connected": conn,
            "vehicle_id": vid,
            "current_log": cur,
            "current_raw_mag_log": cur_raw_mag,
            "current_raw_gps_log": cur_raw_gps,
            "last_lines": lines,
            "last_parsed": lp,
            "gps": gf,
            "gps_age_s": gps_age_s,
            "vfr_hud": vfr,
            "gps_raw": graw,
            "layback_x_m": lb_x,
            "layback_y_m": lb_y,
            "layback_bearing_deg": (math.degrees(bearing) + 360.0) % 360.0 if bearing is not None else None,
            "sample_rate": sr,
            "sample_rate_label": SAMPLE_RATE_LABELS.get(sr, "?"),
            "avg_window_s": aw,
            "alarm_threshold_nt": thr,
            "towfish_serial": ts_serial,
            "last_moving_avg_nt": last_avg,
            "last_deviation_nt": last_dev,
            "last_alarm": last_alm,
            "last_time_sync": ts_str,
            "last_time_sync_unix": ts_unix,
            "nvf_posts_ok": nvf_ok,
            "nvf_last_status": nvf_st,
            "nvf_last_endpoint": nvf_ep,
            "nvf_last_sent": nvf_sent,
            "reader_error": err,
        }
    )


@app.route("/api/connect", methods=["POST"])
def api_connect():
    global serial_thread, gps_thread, stop_event, reader_error, last_nvf_burst_mono
    global gps_bearing_rad, towfish_serial
    global last_moving_avg_nt, last_deviation_nt, last_alarm

    with state_lock:
        if connected:
            return jsonify({"ok": False, "error": "already connected"}), 400

    data = request.get_json(force=True, silent=True) or {}
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
    with state_lock:
        gps_bearing_rad = None
        towfish_serial = None
        last_moving_avg_nt = None
        last_deviation_nt = None
        last_alarm = False
        _field_window.clear()
        for k in ("groundspeed_m_s", "heading_deg", "last_ok_mono"):
            vfr_hud[k] = None
        for k in ("fix_type", "satellites_visible", "hdop", "vdop", "last_ok_mono"):
            gps_raw[k] = None

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
    global stop_event, serial_thread, gps_thread

    stop_event.set()
    t_ser = serial_thread
    t_gps = gps_thread
    if t_ser:
        t_ser.join(timeout=5.0)
    if t_gps:
        t_gps.join(timeout=2.0)
    serial_thread = None
    gps_thread = None
    stop_event = threading.Event()
    return jsonify({"ok": True})


EXPLORER_COMMANDS: Dict[str, bytes] = {
    "autotune_on": EXPLORER_AUTOTUNE_ON_COMMAND,
    "autotune_off": EXPLORER_AUTOTUNE_OFF_COMMAND,
    "cycle_off": EXPLORER_CYCLE_OFF_COMMAND,
    "cycle_4hz": EXPLORER_4HZ_COMMAND,
    "cycle_2hz": EXPLORER_CYCLE_2HZ_COMMAND,
    "cycle_1hz": EXPLORER_CYCLE_1HZ_COMMAND,
}

# Map cycle commands back to their persisted sample-rate key for state.json.
CYCLE_COMMAND_TO_RATE: Dict[str, str] = {
    "cycle_off": "0",
    "cycle_4hz": "1",
    "cycle_2hz": "2",
    "cycle_1hz": "3",
}


@app.route("/api/explorer/command", methods=["POST"])
def api_explorer_command():
    global sample_rate
    body = request.get_json(silent=True) or {}
    name = str(body.get("command", "")).strip()
    cmd = EXPLORER_COMMANDS.get(name)
    if not cmd:
        return (
            jsonify({"ok": False, "error": f"unknown command: {name}"}),
            400,
        )
    ok, msg = send_explorer_command(cmd)
    if ok and name in CYCLE_COMMAND_TO_RATE:
        rate_key = CYCLE_COMMAND_TO_RATE[name]
        with state_lock:
            sample_rate = rate_key
        try:
            current = load_settings()
            save_settings(
                current.get("port", ""),
                int(current.get("baud_rate", 9600)),
                float(current.get("layback_x_m", DEFAULT_LAYBACK_X_M)),
                float(current.get("layback_y_m", DEFAULT_LAYBACK_Y_M)),
                sample_rate_v=rate_key,
            )
        except Exception:
            pass
    sse_broadcast(
        {
            "type": "explorer_command",
            "ok": ok,
            "command": name,
            "byte": cmd.decode("ascii"),
            "message": msg,
            "sample_rate": sample_rate if name in CYCLE_COMMAND_TO_RATE else None,
        }
    )
    return jsonify(
        {
            "ok": ok,
            "command": name,
            "byte": cmd.decode("ascii"),
            "message": msg,
            "sample_rate": sample_rate,
        }
    ), (200 if ok else 409)


@app.route("/api/explorer/sync-time", methods=["POST"])
def api_explorer_sync_time():
    """Send T + 11-digit JJJYYHHMMSS to the Explorer (manual GPS time-sync trigger).

    Time source: host system UTC (already disciplined to autopilot/GPS via BlueOS).
    We require a fresh GPS fix (< 5 s old) so the operator only ever syncs when the
    host clock is GPS-disciplined; without that we'd risk pushing stale time.
    """
    global last_time_sync_str, last_time_sync_unix
    with state_lock:
        local_ser = ser
        last_ok = gps_fix["last_ok_mono"]
    if not local_ser or not local_ser.is_open:
        return jsonify({"ok": False, "error": "Serial port is not open"}), 409
    if last_ok is None or (time.monotonic() - float(last_ok)) > 5.0:
        return (
            jsonify({"ok": False, "error": "No fresh GPS fix; refusing to push host time"}),
            400,
        )
    now = datetime.now(timezone.utc)
    payload = format_explorer_time(now)
    try:
        local_ser.write(payload)
        local_ser.flush()
    except Exception as e:
        return jsonify({"ok": False, "error": f"Serial write failed: {e}"}), 500
    with state_lock:
        last_time_sync_str = payload.decode("ascii", errors="replace")
        last_time_sync_unix = int(now.timestamp())
    sse_broadcast(
        {
            "type": "explorer_time_sync",
            "ok": True,
            "sent": payload.decode("ascii", errors="replace"),
            "utc": now.strftime("%Y-%m-%dT%H:%M:%SZ"),
        }
    )
    return jsonify(
        {
            "ok": True,
            "sent": payload.decode("ascii", errors="replace"),
            "utc": now.strftime("%Y-%m-%dT%H:%M:%SZ"),
        }
    )


@app.route("/api/avg-window", methods=["POST"])
def api_avg_window():
    """Update the deviation moving-average window (one of MOVING_AVG_CHOICES seconds)."""
    body = request.get_json(silent=True) or {}
    raw_v = body.get("window_s", body.get("avg_window_s"))
    try:
        requested = int(raw_v)
    except Exception:
        return (
            jsonify({"ok": False, "error": "window_s must be an integer"}),
            400,
        )
    if requested not in MOVING_AVG_CHOICES:
        return (
            jsonify(
                {
                    "ok": False,
                    "error": "window_s must be one of " + ", ".join(str(c) for c in MOVING_AVG_CHOICES),
                }
            ),
            400,
        )
    win = requested
    current = load_settings()
    save_settings(
        current.get("port", ""),
        int(current.get("baud_rate", 9600)),
        float(current.get("layback_x_m", DEFAULT_LAYBACK_X_M)),
        float(current.get("layback_y_m", DEFAULT_LAYBACK_Y_M)),
        avg_window_v=win,
    )
    apply_runtime_settings({"avg_window_s": win})
    sse_broadcast({"type": "avg_window", "avg_window_s": win})
    return jsonify({"ok": True, "avg_window_s": win})


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
                "sample_rate_choices": list(SAMPLE_RATE_CHOICES),
                "sample_rate_labels": dict(SAMPLE_RATE_LABELS),
                "avg_window_choices": list(MOVING_AVG_CHOICES),
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
        cur_raw_mag = current_raw_mag_name
        cur_raw_gps = current_raw_gps_name
    for p in sorted(LOG_DIR.glob(f"{LOG_PREFIX}*{LOG_SUFFIX}"), key=lambda x: x.stat().st_mtime, reverse=True):
        st = p.stat()
        items.append(
            {
                "name": p.name,
                "kind": "csv",
                "size_bytes": st.st_size,
                "mtime": st.st_mtime,
                "row_count": _csv_line_count(p),
                "current": p.name == cur,
            }
        )
    raw_glob = list(LOG_DIR.glob(f"{RAW_MAG_PREFIX}*{RAW_FILE_SUFFIX}"))
    raw_glob += list(LOG_DIR.glob(f"{RAW_GPS_PREFIX}*{RAW_FILE_SUFFIX}"))
    for p in sorted(raw_glob, key=lambda x: x.stat().st_mtime, reverse=True):
        st = p.stat()
        items.append(
            {
                "name": p.name,
                "kind": "raw_mag" if p.name.startswith(RAW_MAG_PREFIX) else "raw_gps",
                "size_bytes": st.st_size,
                "mtime": st.st_mtime,
                "row_count": None,
                "current": p.name in (cur_raw_mag, cur_raw_gps),
            }
        )
    return jsonify({"logs": items})


@app.route("/api/logs/<name>")
def api_log_download(name: str):
    csv_path = safe_log_name(name)
    raw_path = safe_raw_log_name(name) if not csv_path else None
    path = csv_path or raw_path
    if not path or not path.is_file():
        return jsonify({"ok": False, "error": "not found"}), 404
    if csv_path:
        return send_file(
            path,
            mimetype="text/csv",
            as_attachment=True,
            download_name=name,
            cache_timeout=0,
        )
    return send_file(
        path,
        mimetype="text/plain",
        as_attachment=True,
        download_name=name,
        cache_timeout=0,
    )


@app.route("/api/logs/<name>", methods=["DELETE"])
def api_log_delete(name: str):
    csv_path = safe_log_name(name)
    raw_path = safe_raw_log_name(name) if not csv_path else None
    path = csv_path or raw_path
    if not path or not path.is_file():
        return jsonify({"ok": False, "error": "not found"}), 404
    with state_lock:
        if name in (current_log_name, current_raw_mag_name, current_raw_gps_name):
            return jsonify({"ok": False, "error": "cannot delete active log"}), 400
    try:
        path.unlink()
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


def main() -> None:
    ensure_log_dir()
    s = load_settings()
    apply_layback_settings(s)
    apply_runtime_settings(s)
    port = int(os.environ.get("PORT", "9091"))
    print(f"Marine Magnetics Explorer extension listening on 0.0.0.0:{port}", flush=True)
    serve(app, host="0.0.0.0", port=port, threads=8)


if __name__ == "__main__":
    main()
