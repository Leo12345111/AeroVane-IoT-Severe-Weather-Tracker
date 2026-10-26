from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import queue
import re
import socket
import threading
import time
import webbrowser
from dataclasses import asdict, dataclass, replace
from datetime import datetime
from pathlib import Path

from aiohttp import web

try:
    import serial
    from serial.tools import list_ports
except Exception as exc:
    serial = None
    list_ports = None
    serial_import_error = exc
else:
    serial_import_error = None


DEFAULT_BAUD = 115200
DEFAULT_SERIAL_NUMBER = "1333877"
VALID_FLIGHT_STATES = {"IDLE", "LAUNCH", "BOOST", "COAST", "FLIGHT", "APOGEE", "DESCENT", "LANDED", "STOPPED", "RECOVERY"}
LOW_ALT_REPEAT_MAX_M = 10.0
LOW_ALT_KEEP_SECONDS = 3.0
CSV_HEADER = [
    "time_ms",
    "latitude",
    "longitude",
    "altitude_m",
    "satellites",
    "temp_c",
    "pressure_hpa",
    "acc_x_g",
    "acc_y_g",
    "acc_z_g",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "high_accel_x_g",
    "high_accel_y_g",
    "high_accel_z_g",
    "pitch_deg",
    "roll_deg",
    "received_at",
    "rssi",
    "flight_state",
    "payload_recording",
    "records_written",
    "sd_free_kb",
    "rocket_id",
    "rocket_name",
    "raw_line",
]


@dataclass
class Telemetry:
    received_at: float
    time_ms: float
    lat: float
    lon: float
    alt_m: float
    sats: int
    temp_c: float
    pressure_hpa: float
    acc_x_g: float
    acc_y_g: float
    acc_z_g: float
    gyro_x_dps: float = 0.0
    gyro_y_dps: float = 0.0
    gyro_z_dps: float = 0.0
    high_acc_x_g: float = 0.0
    high_acc_y_g: float = 0.0
    high_acc_z_g: float = 0.0
    sd_frames: int = 0
    sd_free_kb: float = 0.0
    rssi: int | None = None
    flight_state: str = ""
    payload_recording: bool | None = None
    raw_total_g: float | None = None
    mpx1: int = 512
    mpx2: int = 512
    mpx3: int = 512
    wind_speed_m_s: float = 0.0
    wind_dir_deg: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    rocket_id: str = "12345678"
    rocket_name: str = "Rocket 1"
    raw_line: str = ""

    @property
    def acc_total_g(self) -> float:
        return math.sqrt(self.acc_x_g * self.acc_x_g + self.acc_y_g * self.acc_y_g + self.acc_z_g * self.acc_z_g)

    @property
    def high_acc_total_g(self) -> float:
        return math.sqrt(self.high_acc_x_g * self.high_acc_x_g + self.high_acc_y_g * self.high_acc_y_g + self.high_acc_z_g * self.high_acc_z_g)

    def to_json(self) -> dict:
        data = asdict(self)
        data["acc_total_g"] = self.acc_total_g
        data["high_acc_total_g"] = self.high_acc_total_g
        return data

    def to_csv_row(self) -> dict:
        return {
            "time_ms": self.time_ms,
            "latitude": self.lat,
            "longitude": self.lon,
            "altitude_m": self.alt_m,
            "satellites": self.sats,
            "temp_c": self.temp_c,
            "pressure_hpa": self.pressure_hpa,
            "acc_x_g": self.acc_x_g,
            "acc_y_g": self.acc_y_g,
            "acc_z_g": self.acc_z_g,
            "gyro_x_dps": self.gyro_x_dps,
            "gyro_y_dps": self.gyro_y_dps,
            "gyro_z_dps": self.gyro_z_dps,
            "high_accel_x_g": self.high_acc_x_g,
            "high_accel_y_g": self.high_acc_y_g,
            "high_accel_z_g": self.high_acc_z_g,
            "pitch_deg": self.pitch,
            "roll_deg": self.roll,
            "received_at": self.received_at,
            "rssi": self.rssi,
            "flight_state": self.flight_state,
            "payload_recording": self.payload_recording,
            "records_written": self.sd_frames,
            "sd_free_kb": self.sd_free_kb,
            "rocket_id": self.rocket_id,
            "rocket_name": self.rocket_name,
            "raw_line": self.raw_line,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fast browser rocket telemetry dashboard.")
    parser.add_argument("--port", default=None, help="Serial port, for example COM5. Omit to auto-detect.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--serial-number", default=DEFAULT_SERIAL_NUMBER)
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--no-open", action="store_true")
    return parser.parse_args()


def clean_ascii(text: str) -> str:
    return re.sub(r"[^\x20-\x7e]+", "", text)


def parse_float_field(value: str, default: float = 0.0) -> float:
    if re.search(r"\d", value):
        value = value.replace("O", "0").replace("o", "0").replace("I", "1").replace("l", "1")
    match = re.search(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", value)
    if not match:
        return default
    try:
        number = float(match.group(0))
    except ValueError:
        return default
    return number if math.isfinite(number) else default


STRICT_FLOAT_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)")
STRICT_INT_RE = re.compile(r"[-+]?\d+")


def parse_strict_float_field(value: str) -> float | None:
    clean = value.strip()
    if not STRICT_FLOAT_RE.fullmatch(clean):
        return None
    try:
        number = float(clean)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def parse_strict_int_field(value: str) -> int | None:
    clean = value.strip()
    if not STRICT_INT_RE.fullmatch(clean):
        return None
    try:
        return int(clean)
    except ValueError:
        return None


def parse_bool_field(value: str) -> bool | None:
    clean = value.strip().lower()
    if clean in {"1", "true", "yes", "recording", "rec"}:
        return True
    if clean in {"0", "false", "no", "idle", "off"}:
        return False
    return None


def parse_state_field(value: str) -> str:
    clean = re.sub(r"[^A-Za-z]", "", value).upper()
    return clean if clean in VALID_FLIGHT_STATES else ""


FLIGHT_STATE_CODES = {
    "0": "IDLE", "1": "LAUNCH", "2": "BOOST", "3": "COAST", "4": "FLIGHT",
    "5": "APOGEE", "6": "DESCENT", "7": "LANDED", "8": "STOPPED", "9": "RECOVERY",
}


def unwrap_lora_line(text: str) -> tuple[str, int | None]:
    """Return the application payload and RSSI from receiver or raw RYLR998 output."""
    rssi = None
    match = re.search(r'Received packet:\s*"(.*?)"\s*with RSSI:\s*(-?\d+)', text)
    if match:
        return match.group(1).strip(), int(match.group(2))
    if text.startswith("Received packet:"):
        quoted = re.search(r'"(.*?)"', text)
        if quoted:
            return quoted.group(1).strip(), None
    if text.startswith("+RCV="):
        header = text[5:].split(",", 2)
        if len(header) != 3:
            return "", None
        declared_length = parse_strict_int_field(header[1])
        if declared_length is None or declared_length < 0 or declared_length > len(header[2]):
            return "", None
        payload = header[2][:declared_length]
        suffix = header[2][declared_length:]
        if suffix.startswith(","):
            metrics = suffix[1:].split(",", 1)
            if metrics:
                rssi = parse_strict_int_field(metrics[0])
        return payload.strip(), rssi
    return text.strip(), None


def parse_telemetry_line(line: str) -> Telemetry | None:
    raw = line.strip()
    text = clean_ascii(raw)
    text, rssi = unwrap_lora_line(text)
    if not text or text.lower().startswith("time"):
        return None

    parts = [p.strip() for p in text.replace(";", ",").split(",")]
    try:
        if len(parts) == 25 and parts[0] == "RT2":
            time_ms = float(parts[1])
            lat, lon, alt_m = float(parts[2]), float(parts[3]), float(parts[4])
            sats, temp_c, pressure_hpa = int(parts[5]), float(parts[6]), float(parts[7])
            acc_x_g, acc_y_g, acc_z_g = float(parts[8]), float(parts[9]), float(parts[10])
            gyro_x_dps, gyro_y_dps, gyro_z_dps = float(parts[11]), float(parts[12]), float(parts[13])
            high_acc_x_g, high_acc_y_g, high_acc_z_g = float(parts[14]), float(parts[15]), float(parts[16])
            pitch, roll = float(parts[17]), float(parts[18])
            state = FLIGHT_STATE_CODES.get(parts[19], parts[19].upper())
            payload_recording = parse_bool_field(parts[20])
            sd_frames, sd_free_kb = int(parts[21]), float(parts[22])
            rocket_id, rocket_name = parts[23], parts[24]
            raw_total_g = math.sqrt(acc_x_g**2 + acc_y_g**2 + acc_z_g**2)
            mpx1 = mpx2 = mpx3 = 512
        elif len(parts) in (18, 21):
            time_ms = float(parts[0])
            lat, lon, alt_m = float(parts[1]), float(parts[2]), float(parts[3])
            sats, temp_c, pressure_hpa = int(parts[4]), float(parts[5]), float(parts[6])
            acc_x_g, acc_y_g, acc_z_g = float(parts[7]), float(parts[8]), float(parts[9])
            state = FLIGHT_STATE_CODES.get(parts[10], parts[10].upper())
            payload_recording = parse_bool_field(parts[11])
            raw_total_g = float(parts[12])
            if len(parts) == 21:
                gyro_x_dps, gyro_y_dps, gyro_z_dps = float(parts[13]), float(parts[14]), float(parts[15])
                mpx1, mpx2, mpx3 = int(parts[16]), int(parts[17]), int(parts[18])
                rocket_id, rocket_name = parts[19], parts[20]
            else:
                gyro_x_dps = gyro_y_dps = gyro_z_dps = 0.0
                mpx1, mpx2, mpx3 = int(parts[13]), int(parts[14]), int(parts[15])
                rocket_id, rocket_name = parts[16], parts[17]
            high_acc_x_g = high_acc_y_g = high_acc_z_g = 0.0
            pitch = roll = 0.0
            sd_frames, sd_free_kb = 0, 0.0
        else:
            return None
    except (ValueError, IndexError):
        return None

    if state not in VALID_FLIGHT_STATES or payload_recording is None:
        return None
    numeric_values = (time_ms, lat, lon, alt_m, temp_c, pressure_hpa, acc_x_g, acc_y_g, acc_z_g,
                      gyro_x_dps, gyro_y_dps, gyro_z_dps, high_acc_x_g, high_acc_y_g, high_acc_z_g,
                      pitch, roll, sd_free_kb, raw_total_g)
    if not all(math.isfinite(value) for value in numeric_values):
        return None

    return Telemetry(
        received_at=time.time(), time_ms=time_ms, lat=lat, lon=lon, alt_m=alt_m,
        sats=sats, temp_c=temp_c, pressure_hpa=pressure_hpa,
        acc_x_g=acc_x_g, acc_y_g=acc_y_g, acc_z_g=acc_z_g,
        gyro_x_dps=gyro_x_dps, gyro_y_dps=gyro_y_dps, gyro_z_dps=gyro_z_dps,
        high_acc_x_g=high_acc_x_g, high_acc_y_g=high_acc_y_g, high_acc_z_g=high_acc_z_g,
        pitch=pitch, roll=roll, sd_frames=sd_frames, sd_free_kb=sd_free_kb, rssi=rssi,
        flight_state=state, payload_recording=payload_recording, raw_total_g=raw_total_g,
        mpx1=mpx1, mpx2=mpx2, mpx3=mpx3, wind_speed_m_s=0.0, wind_dir_deg=0.0,
        rocket_id=rocket_id, rocket_name=rocket_name, raw_line=raw,
    )


def find_serial_port(serial_number_hint: str | None, excluded: set[str] | None = None) -> str | None:
    if list_ports is None:
        return None
    excluded = {port.upper() for port in (excluded or set())}
    ports = [port for port in list_ports.comports() if port.device.upper() not in excluded]
    if serial_number_hint:
        hint = serial_number_hint.lower()
        for port in ports:
            fields = [
                getattr(port, "serial_number", None),
                getattr(port, "hwid", None),
                getattr(port, "description", None),
                getattr(port, "device", None),
            ]
            if any(value and hint in str(value).lower() for value in fields):
                return port.device
    for port in ports:
        desc = f"{getattr(port, 'description', '')} {getattr(port, 'hwid', '')}".lower()
        if any(word in desc for word in ("arduino", "nano esp32", "esp32", "usb serial", "usb jtag", "cdc", "ch340", "cp210", "wchusb")):
            return port.device
    return ports[0].device if ports else None


def normalize_dashboard_command(command: str) -> str:
    clean = command.strip().upper()
    return clean[4:] if clean.startswith("CMD:") else clean


class TelemetryHub:
    def __init__(self) -> None:
        self.websockets: set[web.WebSocketResponse] = set()
        self.command_queue: queue.Queue[str] = queue.Queue()
        self.latest: Telemetry | None = None
        self.total_packets = 0
        self.status_message = "Starting."
        self.launched = False
        self.landed = False
        self.recording = False
        self.stop_requested = False
        self.suppress_auto_recording = False
        self.start_retry_commands: list[str] = []
        self.start_retry_count = 0
        self.start_retry_at = 0.0
        self.requested_storage_mode = "BOTH"
        self.recorded: list[Telemetry | None] = []
        self.csv_exported = False
        self.last_export_path: str | None = None
        self.last_payload_recording: bool | None = None
        self.correction_count = 0
        self.altitude_zero_m: float | None = None
        self.csv_buffer: list[Telemetry | None] = []
        self.last_csv_flush = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.last_gyro_time_ms: float | None = None

    def _active_point(self, point: Telemetry) -> bool:
        return bool(point.payload_recording)

    def _stopped_point(self, point: Telemetry) -> bool:
        return False

    def corrected_point(self, point: Telemetry) -> Telemetry:
        if self.last_gyro_time_ms is not None and point.time_ms > self.last_gyro_time_ms:
            dt = (point.time_ms - self.last_gyro_time_ms) / 1000.0
            if 0 < dt < 1.0:
                # We only integrate Yaw here, because MPU6050 yaw drifts without a magnetometer,
                # so the payload doesn't compute it. 
                self.current_yaw += point.gyro_z_dps * dt

        self.last_gyro_time_ms = point.time_ms
        # We do NOT overwrite point.roll and point.pitch! The payload computes these accurately at 100Hz.
        point.yaw = self.current_yaw
        
        if point.wind_speed_m_s > 0:
            point.wind_dir_deg = (point.wind_dir_deg + point.yaw) % 360.0


        previous = self.latest
        updates: dict[str, object] = {}
        reasons: list[str] = []

        def use_previous(field: str, reason: str, fallback: float = 0.0) -> None:
            if previous is not None:
                updates[field] = getattr(previous, field)
            else:
                updates[field] = fallback
            reasons.append(reason)

        if not (-90.0 <= point.lat <= 90.0 and -180.0 <= point.lon <= 180.0):
            use_previous("lat", "bad_lat")
            use_previous("lon", "bad_lon")
        elif (abs(point.lat) < 1e-6 and abs(point.lon) < 1e-6) and previous and abs(previous.lat) > 1e-6:
            use_previous("lat", "zero_gps")
            use_previous("lon", "zero_gps")

        if previous and abs(point.lat) > 1e-6 and abs(point.lon) > 1e-6 and abs(previous.lat) > 1e-6 and abs(previous.lon) > 1e-6:
            gps_jump_m = math.hypot((point.lat - previous.lat) * 111320.0, (point.lon - previous.lon) * 111320.0 * math.cos(math.radians(previous.lat)))
            elapsed_s = max(0.05, (point.received_at - previous.received_at))
            if gps_jump_m / elapsed_s > 1200.0 and not self._active_point(point):
                use_previous("lat", "gps_jump")
                use_previous("lon", "gps_jump")

        if not math.isfinite(point.alt_m) or not (-10000.0 <= point.alt_m <= 100000.0):
            use_previous("alt_m", "bad_alt")
        if not (0.0 <= point.pressure_hpa <= 1500.0):
            use_previous("pressure_hpa", "bad_pressure", 0.0)
        if not (-100.0 <= point.temp_c <= 150.0):
            use_previous("temp_c", "bad_temp", 0.0)
        if not (0 <= point.sats <= 99):
            use_previous("sats", "bad_sats", 0)

        for field in ("acc_x_g", "acc_y_g", "acc_z_g"):
            value = getattr(point, field)
            if not math.isfinite(value) or abs(value) > 330.0:
                use_previous(field, f"bad_{field}", 0.0)

        for field in ("gyro_x_dps", "gyro_y_dps", "gyro_z_dps"):
            value = getattr(point, field)
            if not math.isfinite(value) or abs(value) > 2100.0:
                use_previous(field, f"bad_{field}", 0.0)
        
        if point.raw_total_g is not None and (not math.isfinite(point.raw_total_g) or point.raw_total_g > 350.0):
            use_previous("raw_total_g", "bad_raw_total", 0.0)
        if point.raw_total_g is not None and (not math.isfinite(point.raw_total_g) or point.raw_total_g > 350.0 or point.raw_total_g < 0.0):
            use_previous("raw_total_g", "bad_total_g")

        if point.flight_state and point.flight_state.upper() not in VALID_FLIGHT_STATES:
            updates["flight_state"] = previous.flight_state if previous else ""
            reasons.append("bad_state")

        if not reasons:
            return point
        self.correction_count += 1
        raw_line = f"{point.raw_line} | corrected:{','.join(sorted(set(reasons)))}"
        return replace(point, raw_line=raw_line, **updates)

    def zeroed_point(self, point: Telemetry) -> Telemetry:
        # The payload already applies its BME280 reference pressure and the
        # explicit ZERO_ALT command. Never rewrite flight data in the dashboard.
        return point

    def start_recording(self, seed: Telemetry | None = None) -> None:
        if self.recording:
            return
        self.recording = True
        self.csv_exported = False
        self.recorded = []
        self.landed = False
        # Create exactly one crash-safe file for this launch. Rows are appended
        # only while recording is active, so idle telemetry before START_RECORD
        # and after STOP_RECORD never enters the launch CSV.
        self.last_export_path = self.create_csv_path("launch")
        self.csv_buffer = []
        self.last_csv_flush = time.time()

    def reset_flight_state(self) -> None:
        self.flush_csv_buffer()
        self.latest = None
        self.total_packets = 0
        self.launched = False
        self.landed = False
        self.recording = False
        self.stop_requested = False
        self.suppress_auto_recording = False
        self.start_retry_commands = []
        self.start_retry_count = 0
        self.start_retry_at = 0.0
        self.recorded = []
        self.csv_exported = False
        self.last_export_path = None
        self.last_payload_recording = None
        self.altitude_zero_m = None
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.last_gyro_time_ms = None
        self.csv_buffer = []
        self.last_csv_flush = 0.0

    def stop_recording(self, reason: str) -> str | None:
        self.flush_csv_buffer()
        self.recording = False
        if not self.recorded:
            return self.last_export_path
        self.csv_exported = True
        return self.last_export_path

    def csv_log_dir(self) -> Path:
        log_dir = Path.home() / "Downloads"
        if not log_dir.exists():
            log_dir = Path(__file__).with_name("flight_logs")
        log_dir.mkdir(parents=True, exist_ok=True)
        return log_dir

    def unique_csv_path(self, reason: str) -> Path:
        log_dir = self.csv_log_dir()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = log_dir / f"rocket_{reason}_{stamp}.csv"
        suffix = 1
        while path.exists():
            path = log_dir / f"rocket_{reason}_{stamp}_{suffix}.csv"
            suffix += 1
        return path

    def create_csv_path(self, reason: str) -> str:
        path = self.unique_csv_path(reason)
        import csv

        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_HEADER)
            writer.writeheader()
            handle.flush()
            os.fsync(handle.fileno())
        return str(path)

    def append_recorded(self, point: Telemetry | None) -> None:
        if point is None:
            return
        self.recorded.append(point)
        if not self.last_export_path:
            self.last_export_path = self.create_csv_path("launch")
        self.csv_buffer.append(point)
        now = time.time()
        if len(self.csv_buffer) >= 25 or now - self.last_csv_flush >= 0.5:
            self.flush_csv_buffer()

    def flush_csv_buffer(self) -> None:
        if not self.csv_buffer or not self.last_export_path:
            return
        import csv

        with Path(self.last_export_path).open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_HEADER)
            for point in self.csv_buffer:
                if point is None:
                    continue
                else:
                    writer.writerow(point.to_csv_row())
            handle.flush()
            os.fsync(handle.fileno())
        self.csv_buffer = []
        self.last_csv_flush = time.time()

    def last_recorded_point(self) -> Telemetry | None:
        for point in reversed(self.recorded):
            if point is not None:
                return point
        return None

    def should_skip_low_alt_run(self, point: Telemetry) -> bool:
        alt = max(0.0, point.alt_m)
        if alt > LOW_ALT_REPEAT_MAX_M:
            return False
        last_point = self.last_recorded_point()
        if last_point is None or max(0.0, last_point.alt_m) > LOW_ALT_REPEAT_MAX_M:
            return False
        run_start = last_point
        for previous in reversed(self.recorded):
            if previous is None:
                break
            if max(0.0, previous.alt_m) > LOW_ALT_REPEAT_MAX_M:
                break
            run_start = previous
        elapsed_s = max(0.0, (point.time_ms - run_start.time_ms) / 1000.0)
        return elapsed_s > LOW_ALT_KEEP_SECONDS

    def append_landed_point(self) -> None:
        source = self.latest or self.last_recorded_point()
        if source is None:
            return
        landed_point = self.landed_view(source)
        last_point = self.last_recorded_point()
        if last_point is None or last_point.flight_state.upper() != "LANDED":
            self.append_recorded(landed_point)
        self.latest = landed_point

    def landed_view(self, source: Telemetry) -> Telemetry:
        return Telemetry(
            received_at=time.time(),
            time_ms=source.time_ms,
            lat=source.lat,
            lon=source.lon,
            alt_m=0.0,
            sats=source.sats,
            temp_c=source.temp_c,
            pressure_hpa=source.pressure_hpa,
            acc_x_g=source.acc_x_g,
            acc_y_g=source.acc_y_g,
            acc_z_g=source.acc_z_g,
            gyro_x_dps=source.gyro_x_dps,
            gyro_y_dps=source.gyro_y_dps,
            gyro_z_dps=source.gyro_z_dps,
            high_acc_x_g=source.high_acc_x_g,
            high_acc_y_g=source.high_acc_y_g,
            high_acc_z_g=source.high_acc_z_g,
            sd_frames=source.sd_frames,
            sd_free_kb=source.sd_free_kb,
            rssi=source.rssi,
            flight_state="LANDED",
            payload_recording=False,
            raw_total_g=source.raw_total_g,
            mpx1=source.mpx1,
            mpx2=source.mpx2,
            mpx3=source.mpx3,
            wind_speed_m_s=source.wind_speed_m_s,
            wind_dir_deg=source.wind_dir_deg,
            rocket_id=source.rocket_id,
            rocket_name=source.rocket_name,
            raw_line=source.raw_line,
        )

    def export_csv(self, reason: str) -> str:
        path = self.unique_csv_path(reason)
        import csv

        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_HEADER)
            writer.writeheader()
            for point in self.recorded:
                if point is None:
                    continue
                else:
                    writer.writerow(point.to_csv_row())
            handle.flush()
            os.fsync(handle.fileno())
        return str(path)

    async def publish_gap(self) -> None:
        return

    async def update_recording(self, point: Telemetry) -> None:
        payload_recording = bool(point.payload_recording)
        if self.start_retry_commands:
            if payload_recording:
                self.start_retry_commands = []
                self.start_retry_count = 0
                await self.status("Payload confirmed internal recording started.")
            elif time.time() >= self.start_retry_at:
                if self.start_retry_count < 3:
                    for retry_command in self.start_retry_commands:
                        self.command_queue.put(retry_command)
                    self.start_retry_count += 1
                    self.start_retry_at = time.time() + 2.5
                    await self.status(
                        f"Payload has not confirmed recording; retrying Start ({self.start_retry_count}/3)."
                    )
                else:
                    self.start_retry_commands = []
                    await self.status("Payload did not confirm recording after three automatic retries.")
        if self.stop_requested:
            # Stop is two-phase: keep every tail packet, including the final
            # stopped acknowledgement, before closing the crash-safe CSV.
            if self.recording:
                self.append_recorded(point)
            self.last_payload_recording = payload_recording
            if not payload_recording:
                self.stop_requested = False
                self.launched = False
                self.landed = False
                path = self.stop_recording("payload_stop")
                if path:
                    await self.status(f"Payload confirmed stopped. Complete live LoRa CSV saved: {path}")
            return
        recording_started = payload_recording and self.last_payload_recording is not True
        recording_stopped = not payload_recording and self.last_payload_recording is True
        self.last_payload_recording = payload_recording

        if recording_started and not self.recording and not self.suppress_auto_recording:
            self.launched = True
            self.landed = False
            self.start_recording(point)
            await self.status(f"Payload recording started. Building live LoRa CSV: {self.last_export_path}")
        if self.recording:
            self.append_recorded(point)
        if recording_stopped and self.recording:
            self.launched = False
            self.landed = False
            path = self.stop_recording("payload_stop")
            if path:
                await self.status(f"Payload recording stopped. Live LoRa CSV saved: {path}")

    async def publish(self, point: Telemetry) -> None:
        self.total_packets += 1
        await self.update_recording(point)
        outgoing = point
        self.latest = outgoing
        payload = json.dumps({"type": "telemetry", "point": outgoing.to_json(), "total": self.total_packets})
        dead = []
        for ws in self.websockets:
            try:
                await ws.send_str(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.websockets.discard(ws)

    async def status(self, message: str) -> None:
        self.status_message = message
        payload = json.dumps({"type": "status", "message": message})
        for ws in list(self.websockets):
            try:
                await ws.send_str(payload)
            except Exception:
                self.websockets.discard(ws)

    async def handle_command(self, command: str) -> None:
        clean = normalize_dashboard_command(command)
        if not clean:
            return
        action = clean.split(",", 1)[0]
        queued_commands: list[str] = []
        if action in {"START_RECORD", "START_REC"} and "," in clean:
            destination = clean.split(",", 1)[1]
            if destination in {"INTERNAL", "SD", "BOTH"}:
                self.requested_storage_mode = destination
                queued_commands = [f"CMD:STORAGE,{destination}", "CMD:START_RECORD"]
            else:
                queued_commands = [f"CMD:{clean}"]
        elif action in {"START_RECORD", "START_REC", "RECORD"}:
            # The browser sends STORAGE and START separately. Pair them so a
            # retry always restores the selected destination before recording.
            queued_commands = [
                f"CMD:STORAGE,{self.requested_storage_mode}",
                "CMD:START_RECORD",
            ]
        else:
            queued_commands = [f"CMD:{clean}"]
        if action == "STORAGE" and "," in clean:
            destination = clean.split(",", 1)[1]
            if destination in {"INTERNAL", "SD", "BOTH"}:
                self.requested_storage_mode = destination
        for queued_command in queued_commands:
            self.command_queue.put(queued_command)
        if action in {"START_RECORD", "START_REC", "RECORD"}:
            self.stop_requested = False
            self.suppress_auto_recording = False
            # A stale True from the previous flight made the first ordinary
            # pre-acknowledgement False packet look like an instant stop.
            self.last_payload_recording = False
            self.start_retry_commands = list(queued_commands)
            self.start_retry_count = 0
            self.start_retry_at = time.time() + 2.5
            self.launched = True
            self.start_recording(self.latest)
            await self.status(f"Sending START_RECORD to receiver. Building live LoRa CSV: {self.last_export_path}")
        elif action in {"STOP_RECORD", "STOP_REC"}:
            self.start_retry_commands = []
            self.stop_requested = True
            self.suppress_auto_recording = True
            await self.status("Sending STOP_RECORD to receiver; keeping tail packets until payload acknowledgement.")
        elif clean == "RESET_FLIGHT":
            self.reset_flight_state()
            await self.status("Sending RESET_FLIGHT to receiver.")
        else:
            await self.status(f"Sending {clean} to receiver...")

def simulated_flight_values(flight_t: float, t: float) -> tuple[float, float, float, str, bool]:
    if flight_t < 0:
        return 0.0, 0.0, 1.0, "IDLE", False
    if flight_t < 3.0:
        return 35.0 * flight_t * flight_t, 0.000025 * flight_t, 7.2 - flight_t * 0.65, "BOOST", True
    if flight_t < 10.0:
        coast_t = flight_t - 3.0
        return (
            315.0 + 90.0 * coast_t - 4.5 * coast_t * coast_t,
            0.000075 + 0.000018 * coast_t,
            1.05 + math.sin(t * 2.0) * 0.08,
            "COAST",
            True,
        )
    if flight_t < 20.0:
        descent_t = flight_t - 10.0
        peak_alt = 315.0 + 90.0 * 7.0 - 4.5 * 49.0
        return (
            max(0.0, peak_alt - 72.0 * descent_t),
            0.000201 + 0.000011 * descent_t,
            1.0 + math.sin(t * 1.8) * 0.06,
            "DESCENT",
            True,
        )
    return 0.0, 0.00031, 1.0, "LANDED", False


async def simulation_task(hub: TelemetryHub) -> None:
    start = time.time()
    base_lat, base_lon = 34.0522, -118.2437
    await hub.status("Simulation mode running.")
    while True:
        while not hub.command_queue.empty():
            try:
                command = hub.command_queue.get_nowait()
            except queue.Empty:
                break
            command = normalize_dashboard_command(command)
            if command == "RESET_FLIGHT":
                start = time.time()
            elif command.split(",", 1)[0] in {"START_RECORD", "START_REC", "RECORD"}:
                start = time.time() - 2.1
        t = time.time() - start
        flight_t = t - 2.0
        climb, drift, raw_g, state, rec = simulated_flight_values(flight_t, t)
        wobble = math.sin(t * 0.9) * 0.000012 if rec else 0.0
        await hub.publish(
            Telemetry(
                received_at=time.time(),
                time_ms=t * 1000.0,
                lat=base_lat + drift + wobble,
                lon=base_lon + drift * 1.4 + math.cos(t * 0.7) * 0.00012,
                alt_m=climb,
                sats=8,
                temp_c=22.0 - climb * 0.006,
                pressure_hpa=1013.25 * (1.0 - min(climb, 11000.0) / 44330.0) ** 5.255,
                acc_x_g=math.sin(t * 2.0) * 0.25,
                acc_y_g=math.cos(t * 1.7) * 0.2,
                acc_z_g=raw_g + math.sin(t * 5.0) * 0.1,
                rssi=int(-48 - min(55, t * 0.8) + math.sin(t) * 4),
                high_acc_x_g=0.0,
                high_acc_y_g=0.0,
                high_acc_z_g=0.0,
                sd_frames=0,
                sd_free_kb=0,
                flight_state=state,
                payload_recording=rec,
                raw_total_g=raw_g,
                raw_line="simulated",
            )
        )
        await asyncio.sleep(0.1)


def queue_loop_task(loop: asyncio.AbstractEventLoop, coro) -> bool:
    try:
        asyncio.run_coroutine_threadsafe(coro, loop)
        return True
    except RuntimeError:
        coro.close()
        return False


def serial_thread(loop: asyncio.AbstractEventLoop, hub: TelemetryHub, args: argparse.Namespace, stop_event: threading.Event) -> None:
    if serial is None:
        detail = f" Import error: {serial_import_error}" if serial_import_error else ""
        queue_loop_task(
            loop,
            hub.status(f"pyserial is missing. Run: py -m pip install -r requirements.txt{detail}"),
        )
        return
    last_error = ""
    rejected_ports: dict[str, float] = {}
    while not stop_event.is_set():
        now = time.time()
        rejected_ports = {port: until for port, until in rejected_ports.items() if until > now}
        active = args.port or find_serial_port(args.serial_number, set(rejected_ports))
        if not active:
            message = "No serial port found. Rechecking..."
            if message != last_error:
                if not queue_loop_task(loop, hub.status(message)):
                    return
                last_error = message
            stop_event.wait(1.0)
            continue
        try:
            with serial.Serial(active, args.baud, timeout=0.05, write_timeout=0.2) as ser:
                last_error = ""
                receiver_verified = False
                role_probe_started = time.time()
                if not queue_loop_task(loop, hub.status(f"Checking {active} for the LoRa receiver.")):
                    return
                last_packet_seen = time.time()
                last_no_data_report = 0.0
                while not stop_event.is_set():
                    while receiver_verified and not hub.command_queue.empty():
                        try:
                            command = hub.command_queue.get_nowait()
                        except queue.Empty:
                            break
                        ser.write((command.strip().upper() + "\n").encode("utf-8"))
                    raw = ser.readline().decode("utf-8", errors="ignore")
                    now = time.time()
                    if not raw:
                        if not receiver_verified and now - role_probe_started > 6.0:
                            rejected_ports[active.upper()] = now + 10.0
                            queue_loop_task(loop, hub.status(
                                f"Ignoring {active}: it did not identify as the LoRa receiver."
                            ))
                            break
                        if now - last_packet_seen > 15.0 and now - last_no_data_report > 15.0:
                            last_no_data_report = now
                            if not queue_loop_task(loop, hub.status(
                                f"Receiver {active} is connected; waiting for payload LoRa telemetry."
                            )):
                                return
                        continue
                    clean_raw = raw.strip()
                    if clean_raw.startswith((
                        "Xeno payload production telemetry starting",
                        "Saved storage mode:",
                        "Recorder idle:",
                        "HEALTH imu=",
                    )):
                        rejected_ports[active.upper()] = now + 10.0
                        queue_loop_task(loop, hub.status(
                            f"Ignoring payload USB on {active}; connect the LoRa receiver USB to control recording."
                        ))
                        break
                    if clean_raw.startswith((
                        "Initializing production RFM95 receiver",
                        "RECEIVER_HEALTH",
                        "Received packet:",
                        "Sent command to payload",
                        "Command TX failed",
                    )) and not receiver_verified:
                        receiver_verified = True
                        if not queue_loop_task(loop, hub.status(f"LoRa receiver verified on {active}.")):
                            return
                    if clean_raw.startswith("Sent command to payload"):
                        sent_command = clean_raw.split("CMD:", 1)[1] if "CMD:" in clean_raw else clean_raw
                        if sent_command == "WIFI_ON":
                            sent_status = "WIFI_ON transmitted. Connect to XenoCam-1333877 using password XenoCam915, then click Flight Files."
                        elif sent_command == "WIFI_OFF":
                            sent_status = "WIFI_OFF transmitted to payload."
                        else:
                            sent_status = f"Receiver transmitted {sent_command} to payload (3 LoRa copies)."
                        if not queue_loop_task(loop, hub.status(sent_status)):
                            return
                        continue
                    if clean_raw.startswith("Command TX failed"):
                        if not queue_loop_task(loop, hub.status(clean_raw)):
                            return
                        continue
                    if clean_raw.startswith("RECEIVER_HEALTH"):
                        if now - last_packet_seen > 3.0:
                            if not queue_loop_task(loop, hub.status(
                                f"Receiver {active} healthy; waiting for payload LoRa telemetry."
                            )):
                                return
                        continue
                    point = parse_telemetry_line(raw)
                    if point:
                        last_packet_seen = time.time()
                        if not queue_loop_task(loop, hub.publish(point)):
                            return
        except Exception as ex:
            message = f"Serial disconnected: {ex}. Reconnecting..."
            if message != last_error:
                if not queue_loop_task(loop, hub.status(message)):
                    return
                last_error = message
            stop_event.wait(1.0)


HTML = r"""<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>Fast Rocket Dashboard</title>
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css"/>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    *{box-sizing:border-box} body{margin:0;padding-top:20px;font-family:Segoe UI,Arial,sans-serif;background:#0b1220;color:#e5e7eb;overflow:hidden}
    header{height:54px;display:flex;align-items:center;gap:10px;padding:8px 12px;background:#111827;border-bottom:1px solid #253044}
    button,select,input.copybox{background:#1f2937;color:white;border:1px solid #334155;border-radius:7px;padding:8px 10px} button{cursor:pointer} button:hover,select:hover,input.copybox:focus{background:#334155}button:disabled{opacity:.36;cursor:not-allowed;background:#111827;color:#64748b;border-color:#1f2937}
    input.copybox{width:194px;font:12px Consolas,monospace;user-select:text}
    #status{margin-left:auto;color:#9ca3af;font-size:13px}
    #substatus{color:#64748b;font-size:12px;max-width:340px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
    nav{height:42px;display:flex;gap:8px;padding:6px 12px;background:#0f172a}
    .tab{display:none;height:calc(100vh - 116px)} .tab.active{display:block}
    #graphs{padding:10px;overflow:auto}.stats{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px}.stat{background:#111827;border:1px solid #263348;border-radius:8px;padding:8px 10px;min-width:128px}.stat b{display:block;font-size:18px;color:white}.grid{display:grid;grid-template-columns:repeat(2,minmax(320px,1fr));gap:10px}.panel{background:white;color:#111827;border-radius:8px;border:1px solid #e5e7eb;padding:10px}.panel h3{margin:0 0 6px;font-size:14px}canvas.chart{width:100%;height:148px;display:block}
    #csvReview{display:none;align-items:center;gap:10px;background:#111827;border:1px solid #263348;border-radius:8px;padding:9px 10px;margin-bottom:10px}#csvReview input{flex:1;min-width:160px}#csvReview b{color:white;min-width:82px}#csvReview span{color:#cbd5e1;font-size:13px;white-space:nowrap}
    #map{height:calc(100vh - 116px);position:relative}#leaflet{height:100%}#mapTimeline{position:absolute;left:14px;right:14px;bottom:16px;z-index:500;display:none;align-items:center;gap:10px;background:#111827e8;border:1px solid #263348;border-radius:8px;padding:10px 12px;box-shadow:0 8px 24px #0007;backdrop-filter:blur(8px)}#mapTimeline b{color:white;min-width:68px}#mapTimeline input{flex:1;min-width:160px}#mapTimeline span{color:#cbd5e1;font-size:13px;white-space:nowrap}
    #mobile{height:calc(100vh - 116px);background:#020617;overflow:hidden}#mobileCameraPane{height:60%;position:relative;background:#020617}#mobileVideo{width:100%;height:100%;object-fit:cover;display:block;background:#111827}#mobileOverlay{position:absolute;inset:0;pointer-events:none;background:linear-gradient(180deg,#0006,#0000 34%,#0008)}#mobileHud{position:absolute;left:12px;right:12px;top:10px;display:flex;align-items:flex-start;justify-content:space-between;gap:10px}#mobileStartBtn{pointer-events:auto;background:#f8fafc;color:#0f172a;border-color:#e5e7eb;font-weight:700}#mobileLocationText{font:12px Consolas,monospace;color:#e5e7eb;background:#020617bf;border:1px solid #334155;border-radius:7px;padding:8px;max-width:58%;user-select:text;pointer-events:auto}#rocketDirection{position:absolute;left:50%;top:50%;width:92px;height:92px;margin:-46px 0 0 -46px;display:grid;place-items:center;color:#f8fafc;text-shadow:0 2px 8px #000;filter:drop-shadow(0 0 8px #000);transition:left .18s ease,top .18s ease,opacity .18s ease}#rocketDirection.offscreen{color:#facc15;animation:pulseEdge 1s ease-in-out infinite}#rocketDirection i{font-size:54px;transform-origin:50% 50%;will-change:transform}#cameraTurnCue{position:absolute;left:50%;top:calc(50% + 52px);transform:translateX(-50%);min-width:128px;text-align:center;color:#f8fafc;background:#020617bf;border:1px solid #475569;border-radius:7px;padding:6px 8px;font-size:13px;font-weight:800;text-shadow:0 2px 8px #000;opacity:0;transition:opacity .18s ease,left .18s ease,top .18s ease}#cameraTurnCue.visible{opacity:1}@keyframes pulseEdge{0%,100%{transform:scale(1)}50%{transform:scale(1.12)}}#mobileReadout{position:absolute;left:12px;right:12px;bottom:12px;display:flex;justify-content:space-between;gap:10px;align-items:end;color:#e5e7eb;font-weight:700;text-shadow:0 2px 8px #000}#mobileDistance{font-size:18px}#mobileHint{font-size:12px;color:#cbd5e1;font-weight:500;text-align:right}#mobileMapWrap{height:40%;position:relative;border-top:1px solid #253044}#mobileLeaflet{height:100%}
    #view3d{height:calc(100vh - 116px);position:relative;background:#06111f} #canvas3d{width:100%;height:100%;display:block}#view3dTimeline{position:absolute;left:14px;right:14px;bottom:16px;z-index:10;display:none;align-items:center;gap:10px;background:#111827e8;border:1px solid #263348;border-radius:8px;padding:10px 12px;box-shadow:0 8px 24px #0007;backdrop-filter:blur(8px)}#view3dTimeline b{color:white;min-width:52px}#view3dTimeline input{flex:1;min-width:160px}#view3dTimeline span{color:#cbd5e1;font-size:13px;white-space:nowrap}
    #latest{font-size:12px;color:#6b7280;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;margin-top:8px}
    .leaflet-div-icon { background: transparent; border: none; }
    .rocket-marker{width:34px;height:34px;display:grid;place-items:center;color:#f8fafc;background:transparent;border:0;box-shadow:none}
    .rocket-marker i{font-size:28px;line-height:1;filter:drop-shadow(0 2px 3px #000c) drop-shadow(0 0 5px #38bdf8cc);transition:transform 0.1s linear;transform-origin:50% 50%}
    .flag-marker{filter:drop-shadow(0 3px 6px #0009)}
    .flag-marker svg{display:block}
    @media(max-width:850px){html,body{height:auto;min-height:100%;overflow:auto;-webkit-overflow-scrolling:touch}header{height:auto;min-height:54px;flex-wrap:wrap;position:relative}#status{margin-left:0}nav{height:auto;min-height:42px;overflow-x:auto;position:relative}.grid{grid-template-columns:1fr}.tab{height:auto;min-height:calc(100vh - 136px);overflow:visible}#graphs{overflow:visible}#map,#view3d,#mobile{height:auto;min-height:calc(100vh - 136px);overflow:visible}#leaflet,#canvas3d{height:calc(100vh - 156px);min-height:520px;touch-action:none}#mobile{display:none;background:#020617}#mobile.active{display:block}#mobileCameraPane{height:62vh;min-height:420px}#mobileMapWrap{height:38vh;min-height:280px}#mobileLeaflet{height:100%}#mapTimeline,#view3dTimeline{position:sticky;bottom:10px;margin:10px 12px;left:auto;right:auto}}
  </style>
</head>
<body>
<header>
  <b>Fast Rocket Dashboard</b>
  <button id="zeroAllBtn" onclick="confirmClick(this,'Zero All Sensors',()=>sendCommand('ZERO_ALL'))" style="background:#8b5cf6">Zero All</button>
  <button id="zeroAltBtn" onclick="confirmClick(this,'Zero Alt',()=>sendCommand('ZERO_ALT'))">Zero Alt</button>
  <button id="zeroAccelBtn" onclick="confirmClick(this,'Zero Accel',()=>sendCommand('ZERO_ACCEL'))">Zero Accel</button>
  <button id="startRecBtn" onclick="startRecording(true)">Start Rec</button>
  <button id="stopRecBtn" onclick="stopRecording(true)">Stop Rec</button>
  <select id="storageSelect" title="Payload recording destination" onchange="setPayloadStorage(this.value)">
    <option value="BOTH">Internal + SD</option><option value="INTERNAL">Internal only</option><option value="SD">SD only</option>
  </select>
  <button id="payloadWifiBtn" onclick="togglePayloadWifi()" title="Wi-Fi remains on until you turn it off">Wi-Fi On</button>
  <button id="payloadFilesBtn" onclick="openPayloadFiles()" title="Open the payload flight download page">Flight Files</button>
  <button id="importCsvBtn" onclick="confirmClick(this,'Import CSV',()=>csvInput.click())">Import CSV</button>
  <button id="myLocationBtn" onclick="confirmClick(this,'My Location',()=>requestMyLocation())">My Location</button>

  <button id="resetBtn" onclick="confirmClick(this,'Reset',()=>resetFlight(true))">Reset</button>
  <div style="display:flex; flex-direction:column; align-items:flex-start;">
    <div style="display:flex; align-items:center;">
      <select id="bindSelect" onchange="handleBindSelect(this.value)" title="Select bound Rocket" style="font-weight:bold; background-color:#1e293b; color:#86efac; padding:2px 8px; border-radius:4px"></select>
      <button id="renameBtn" onclick="renameBoundRocket()" title="Rename this rocket" style="display:none; padding:2px 6px; border:none; background:none; cursor:pointer; font-size:14px">✏️</button>
    </div>
    <div id="boundIdLabel" style="font-size:10px; color:#64748b; margin-top:2px; padding-left:2px; font-weight:600;"></div>
  </div>
  <select id="unitSelect" title="Units" onchange="setUnits(this.value)"><option value="metric">Metric</option><option value="imperial">Imperial</option></select>
  <input id="locationBox" class="copybox" readonly value="--, --" title="Rocket location" onclick="this.select()" onfocus="this.select()"/>
  <input id="csvInput" type="file" accept=".csv,text/csv,text/plain,application/csv,application/vnd.ms-excel" style="display:none"/>
  <span id="status">Connecting...</span>
  <span id="substatus"></span>
</header>
<nav>
    <button onclick="showTab('graphs')">Graphs</button>
    <button onclick="showTab('map')">Map</button>
    <button onclick="showTab('view3d')">3D</button>
    <button id="btnViewWind" onclick="showTab('viewWind')" style="display:none;">Wind Map</button>
    <button onclick="showTab('mobile')">Camera</button>
  </nav>
  <section id="viewWind" class="tab" style="position:relative;">
    <canvas id="canvasWind" style="width:100%; height:100%; display:block; background:#0f172a;"></canvas>
    <div id="windTimeline" style="position:absolute; bottom:20px; left:20px; right:20px; display:none; align-items:center; gap:10px; background:rgba(15,23,42,0.8); padding:10px; border-radius:8px;">
      <input type="range" id="windSlider" min="0" max="0" value="0" style="flex:1;">
      <span id="windTimeLabel" style="color:#cbd5e1; font-family:monospace; white-space:nowrap; min-width:180px; text-align:right;"></span>
    </div>
  </section>
<section id="graphs" class="tab active">
  <div class="stats">
    <div class="stat">Altitude<b id="sAlt">-- m</b></div><div class="stat">Acceleration<b id="sAcc">-- g</b></div>
    <div class="stat">Accel X<b id="sAccX">-- g</b></div><div class="stat">Accel Y<b id="sAccY">-- g</b></div><div class="stat">Accel Z<b id="sAccZ">-- g</b></div>
    <div class="stat">High-G Accel<b id="sHighAcc">-- g</b></div><div class="stat">High-G XYZ<b id="sHighAccXYZ">--/--/-- g</b></div>
    <div class="stat">Cam State<b id="sCamState">IDLE</b></div><div class="stat">SD Frames<b id="sSdFrames">0</b></div><div class="stat">SD Free<b id="sSdFree">0.0 MB</b></div>
    <div class="stat">Satellites<b id="sSats">--</b></div><div class="stat">RSSI<b id="sRssi">-- dBm</b></div>
    <div class="stat">Pressure<b id="sPressure">-- hPa</b></div>
    <div class="stat">MPX1<b id="sMpx1">--</b></div><div class="stat">MPX2<b id="sMpx2">--</b></div><div class="stat">MPX3<b id="sMpx3">--</b></div>
    <div class="stat">Wind Speed<b id="sWindSpeed">-- m/s</b></div><div class="stat">Wind Dir<b id="sWindDir">-- &deg;</b></div>
    <div class="stat">Latitude<b id="sLat">--</b></div><div class="stat">Longitude<b id="sLon">--</b></div>
    <div class="stat">Flight<b id="sFlight">IDLE</b></div><div class="stat">Avg V Speed<b id="sAvgSpeed">-- m/s</b></div><div class="stat">Packets<b id="sPackets">0</b></div>
  </div>
  <div id="csvReview" style="display:flex; align-items:center; flex-wrap:wrap; gap:10px;">
    <b>Graph Time</b>
    <input id="csvSlider" type="range" min="0" max="0" value="0" step="1" style="flex:1; min-width:150px;"/>
    <span id="csvPointLabel">No data yet</span>
  </div>
  <div style="display:flex; align-items:center; flex-wrap:wrap; gap:10px; margin-top: 10px; margin-bottom: 15px; padding: 10px; background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px; color: black;">
    <b>Y-Axis Zoom (Clip Spikes)</b>
    <input id="zoomYSlider" type="range" min="1" max="50" value="1" step="0.5" oninput="updateGlobalZoom(this.value)" style="flex:1; min-width:150px; height: 24px; cursor: pointer;"/>
    <span id="zoomYLabel" style="min-width: 40px; font-weight: bold;">1.0x</span>
    
    <b style="margin-left: 20px;">X-Axis Zoom (Time Window)</b>
    <input id="zoomXSlider" type="range" min="0" max="100" value="50" step="1" oninput="updateGlobalZoomX(this.value)" style="flex:1; min-width:150px; height: 24px; cursor: pointer;"/>
    <span id="zoomXLabel" style="min-width: 50px; font-weight: bold;">10.0 s</span>
  </div>
  <div class="grid">
    <div class="panel"><h3>Altitude</h3><canvas class="chart" id="altChart"></canvas></div>
    <div class="panel"><h3>Acceleration Total</h3><canvas class="chart" id="accTotalChart"></canvas></div>
    <div class="panel"><h3>Acceleration X</h3><canvas class="chart" id="accXChart"></canvas></div>
    <div class="panel"><h3>Acceleration Y</h3><canvas class="chart" id="accYChart"></canvas></div>
    <div class="panel"><h3>Acceleration Z</h3><canvas class="chart" id="accZChart"></canvas></div>
    <div class="panel"><h3>High-G Total</h3><canvas class="chart" id="highAccTotalChart"></canvas></div>
    <div class="panel"><h3>High-G XYZ</h3><canvas class="chart" id="highAccChart"></canvas></div>
    <div class="panel"><h3>Pressure</h3><canvas class="chart" id="pressureChart"></canvas></div>
    <div class="panel"><h3>Temperature</h3><canvas class="chart" id="tempChart"></canvas></div>
    <div class="panel" id="panelGyro"><h3>Gyroscope Together (DPS)</h3><canvas class="chart" id="gyroChart"></canvas></div>
    <div class="panel" id="panelGyroX"><h3>Gyroscope X (Roll)</h3><canvas class="chart" id="gyroXChart"></canvas></div>
    <div class="panel" id="panelGyroY"><h3>Gyroscope Y (Pitch)</h3><canvas class="chart" id="gyroYChart"></canvas></div>
    <div class="panel" id="panelGyroZ"><h3>Gyroscope Z (Yaw)</h3><canvas class="chart" id="gyroZChart"></canvas></div>
    <div class="panel" id="panelWindSpeed"><h3>Wind Speed</h3><canvas class="chart" id="windSpeedChart"></canvas></div>
    <div class="panel" id="panelMpx"><h3>MPXV7002 Raw</h3><canvas class="chart" id="mpxChart"></canvas></div>
  </div>
  <div id="latest">No packets yet</div>
</section>
<section id="map" class="tab">
  <div id="leaflet"></div>
  <div id="mapTimeline">
    <b>Timeline</b>
    <input id="mapSlider" type="range" min="0" max="0" value="0" step="1"/>
    <span id="mapTimeLabel">No path yet</span>
  </div>
</section>
<section id="view3d" class="tab">
  <canvas id="canvas3d"></canvas>
  <div id="view3dTimeline">
    <b>Color:</b>
    <select id="map3dColorSelect" onchange="request3d(true)" style="padding:2px 4px;font-size:12px;margin-right:10px;">
       <option value="altitude" selected>Altitude</option>
       <option value="wind">Wind Speed</option>
       <option value="speed">Vertical Speed</option>
       <option value="accel">Acceleration</option>
    </select>
    <b>3D Time</b>
    <input id="view3dSlider" type="range" min="0" max="0" value="0" step="1"/>
    <span id="view3dTimeLabel">No path yet</span>
  </div>
</section>
<section id="mobile" class="tab">
  <div id="mobileCameraPane">
    <video id="mobileVideo" autoplay muted playsinline></video>
    <div id="mobileOverlay">
      <div id="mobileHud">
        <button id="mobileStartBtn" onclick="startMobileSensors()">Start Camera</button>
        <div id="mobileLocationText">Rocket: --, --<br>Phone: --, --</div>
      </div>
      <div id="rocketDirection"><i class="fa-solid fa-rocket"></i></div>
      <div id="cameraTurnCue"></div>
      <div id="mobileReadout">
        <div id="mobileDistance">No rocket GPS</div>
        <div id="mobileHint">Use phone camera toward sky</div>
      </div>
    </div>
  </div>
  <div id="mobileMapWrap"><div id="mobileLeaflet"></div></div>
</section>
<script>
let ws, activeTab='graphs', all=[], flight=[], recorded=[], graphBase=null, packetCount=0, lastGraph=0, lastMap=0, last3d=0, lastWind=0, lastMobile=0, lastIdleUi=0;
let boundRocketId = localStorage.getItem('boundRocketId') || '';
let knownRockets = JSON.parse(localStorage.getItem('knownRockets') || '{}');
let reconnectTimer=null, reconnectDelay=800;
let latestLive=null, viewingImport=false, importIndex=-1;
let launched=false, landed=false, map, rocketMarker, launchMarker, pathLine, mapActiveLine, gapLines=[], myMarker, myAccuracyCircle;
let recording=false, csvExported=false, recordStartedAt=0, importing=false, stopCommandSent=false, stopRequestedAt=0, stopFinalizeTimer=null, lastPayloadRecording=null, localAltZero=null, suppressAutoRecording=false;
let graphIndex=-1, graphFollow=true, mapIndex=-1, mapFollow=true, view3dIndex=-1, view3dFollow=true;
let timelineLastTime=null, timelineOffset=0;
let lastPacketWall=null, lastUiPacketAt=0;
let pendingDisconnectStart=null;
let telemetryQueue=[], processingTelemetry=false;
let myLocation=null, myWatchId=null, mapCenteredOnMe=false;
let mobileMap=null, mobileRocketMarker=null, mobileMeMarker=null, mobilePathLine=null, mobileActiveLine=null, mobileCameraStream=null, mobileHeading=null, mobilePitch=null, mobileOrientationReady=false, hasAbsoluteOrientation=false;
let mobileGapLines=[];
let currentRoll=0, currentPitch=0, currentYaw=0, lastGyroTimeMs=null;
let unitSystem='metric';
let serverStatusMessage='';
let cam={azim:-58,elev:24,zoom:1,panX:0,panY:0,drag:false,button:0,x:0,y:0};
const statusEl=document.getElementById('status'), substatusEl=document.getElementById('substatus');
const locationBox=document.getElementById('locationBox');
const bindSelect=document.getElementById('bindSelect'), renameBtn=document.getElementById('renameBtn'), boundIdLabel=document.getElementById('boundIdLabel');
const mobileVideo=document.getElementById('mobileVideo'), mobileStartBtn=document.getElementById('mobileStartBtn'), mobileLocationText=document.getElementById('mobileLocationText'), rocketDirection=document.getElementById('rocketDirection'), cameraTurnCue=document.getElementById('cameraTurnCue'), mobileDistance=document.getElementById('mobileDistance'), mobileHint=document.getElementById('mobileHint');
const pendingClicks=new Map();
const DOUBLE_CLICK_MS=3000;
const PACKET_GAP_S=1.5, DISCONNECT_MARKER_MIN_S=3.0, DEVICE_TIME_GAP_MS=2500, SHOW_TIMED_GAPS=true, UI_STALE_MS=15000, IDLE_UI_STALE_MS=15000, UI_FLUSH_MS=100, ACTIVE_RENDER_MS=100, IDLE_GRAPH_MS=100, MAX_QUEUE_PER_FLUSH=500, STOP_ACK_TIMEOUT_MS=4000, LOW_ALT_MAX_M=10, GRAPH_LOW_ALT_KEEP_S=2, LOW_ALT_KEEP_S=3;
const MAX_3D_ZOOM=18, MIN_3D_ZOOM=.25, CAMERA_CENTER_HALF_DEG=30;
const DISCONNECT_MARKER_COLOR='#eab308', DISCONNECT_MARKER_FADED='rgba(234,179,8,.48)';
const MAP_TILE_SIZE=256, MAP_TEXTURE_SIZE=768, MAP_TILE_TEMPLATE='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}';
const ROCKET_ICON_HEADING_OFFSET_DEG=-45;
const ROCKET_GLYPH_DEFAULT_ANGLE_RAD=-Math.PI/4;
const mapTileCache=new Map();
const flagIconHtml=`<div class="flag-marker"><svg width="32" height="32" viewBox="0 0 32 32" aria-label="Launch flag"><path d="M8 30V4" stroke="#111827" stroke-width="3" stroke-linecap="round"/><path d="M10 5h17l-4 6 4 6H10Z" fill="#ef4444" stroke="#7f1d1d" stroke-width="1.5"/><circle cx="8" cy="30" r="3" fill="#111827"/></svg></div>`;
const rocketMapIcon=()=>L.divIcon({className:'', html:'<span class="rocket-marker" aria-label="Rocket"><i class="fa-solid fa-rocket"></i></span>', iconSize:[34,34], iconAnchor:[17,17]});
const flagMapIcon=()=>L.divIcon({className:'', html:flagIconHtml, iconSize:[32,32], iconAnchor:[8,30], popupAnchor:[8,-26]});
const charts=[
  {id:'altChart', lines:[{label:()=>`Alt ${altUnit()}`, get:p=>altValue(displayAltitude(p)), color:'#0284c7'}], pad:5},
  {id:'accTotalChart', lines:[{label:'Total', get:p=>p.acc_total_g??Math.hypot(p.acc_x_g||0,p.acc_y_g||0,p.acc_z_g||0), color:'#7c3aed'}], pad:.2},
  {id:'accXChart', lines:[{label:'X', get:p=>p.acc_x_g??0, color:'#2563eb'}], pad:.2},
  {id:'accYChart', lines:[{label:'Y', get:p=>p.acc_y_g??0, color:'#16a34a'}], pad:.2},
  {id:'accZChart', lines:[{label:'Z', get:p=>p.acc_z_g??0, color:'#dc2626'}], pad:.2},
  {id:'highAccTotalChart', lines:[{label:'Total High-G', get:p=>p.high_acc_total_g??0, color:'#a855f7'}], pad:1},
  {id:'highAccChart', lines:[{label:'X', get:p=>p.high_acc_x_g??0, color:'#2563eb'},{label:'Y', get:p=>p.high_acc_y_g??0, color:'#16a34a'},{label:'Z', get:p=>p.high_acc_z_g??0, color:'#dc2626'}], pad:1},
  {id:'pressureChart', lines:[{label:()=>pressureUnit(), get:p=>pressureValue(p.pressure_hpa), color:'#16a34a'}], pad:1},
  {id:'tempChart', lines:[{label:()=>tempUnit(), get:p=>tempValue(p.temp_c), color:'#d97706'}], pad:1},
  {id:'gyroChart', lines:[{label:'X', get:p=>p.gyro_x_dps??0, color:'#2563eb'},{label:'Y', get:p=>p.gyro_y_dps??0, color:'#16a34a'},{label:'Z', get:p=>p.gyro_z_dps??0, color:'#dc2626'}], pad:5},
  {id:'gyroXChart', lines:[{label:'X', get:p=>p.gyro_x_dps??0, color:'#2563eb'}], pad:5},
  {id:'gyroYChart', lines:[{label:'Y', get:p=>p.gyro_y_dps??0, color:'#16a34a'}], pad:5},
  {id:'gyroZChart', lines:[{label:'Z', get:p=>p.gyro_z_dps??0, color:'#dc2626'}], pad:5},
  {id:'windSpeedChart', lines:[{label:()=>`Wind Speed (${speedUnit()})`, get:p=>speedValue(p.wind_speed_m_s||0), color:'#06b6d4'}], pad:.5},
  {id:'mpxChart', lines:[{label:'MPX 1', get:p=>p.mpx1??512, color:'#ec4899'},{label:'MPX 2', get:p=>p.mpx2??512, color:'#8b5cf6'},{label:'MPX 3', get:p=>p.mpx3??512, color:'#14b8a6'}], pad:10},
];
function altValue(m){return unitSystem==='imperial'?m*3.28084:m}
function altUnit(){return unitSystem==='imperial'?'ft':'m'}
function formatAltMeters(m,d=1){return `${altValue(Number.isFinite(+m)?+m:0).toFixed(d)} ${altUnit()}`}
function speedValue(ms){return unitSystem==='imperial'?ms*3.28084:ms}
function speedUnit(){return unitSystem==='imperial'?'ft/s':'m/s'}
function formatVerticalSpeed(ms,d=1){return `${speedValue(ms||0).toFixed(d)} ${speedUnit()}`}
function pressureValue(hpa){return unitSystem==='imperial'?(hpa||0)*0.029529983:(hpa||0)}
function pressureUnit(){return unitSystem==='imperial'?'inHg':'hPa'}
function formatPressure(hpa,d=1){return `${pressureValue(hpa).toFixed(d)} ${pressureUnit()}`}
function tempValue(c){return unitSystem==='imperial'?(c||0)*9/5+32:(c||0)}
function tempUnit(){return unitSystem==='imperial'?'F':'C'}
function formatTemp(c,d=1){return `${tempValue(c).toFixed(d)} ${tempUnit()}`}
function setButtonDisabled(id, disabled){
  let button=document.getElementById(id);
  if(button) button.disabled=!!disabled;
}
function updateControls(){
  let hasLiveData=all.length>0||flight.length>0||recorded.length>0||latestLive;
  setButtonDisabled('startRecBtn', viewingImport||recording||stopCommandSent);
  setButtonDisabled('stopRecBtn', viewingImport||!recording||stopCommandSent);

  let resetBtn = document.getElementById('resetBtn');
  if(resetBtn && !resetBtn.textContent.includes('Confirm')) {
    resetBtn.textContent = viewingImport ? 'Close CSV' : 'Reset';
  }
  setButtonDisabled('resetBtn', !viewingImport&&(!hasLiveData&&!recording&&!launched&&!landed));
  let connected=(ws&&ws.readyState===1)||!!window.AndroidBridge;
  setButtonDisabled('zeroAllBtn', viewingImport||!connected);
  setButtonDisabled('zeroAltBtn', viewingImport||!connected);
  setButtonDisabled('zeroAccelBtn', viewingImport||!connected);
  setButtonDisabled('importCsvBtn', recording);
}
function setUnits(value){
  unitSystem=value==='imperial'?'imperial':'metric';
  let pts=graphSeries();
  let p=graphIndex>=0&&pts.length?pts[Math.min(graphIndex,pts.length-1)]:(latestLive||series().at(-1));
  if(p) updateStats(p);
  requestGraph(true); requestMap(true); request3d(true); requestMobile(false);
}
function formatGraphTime(seconds){
  if(!Number.isFinite(seconds)) return '0s';
  if(seconds<60) return `${seconds.toFixed(seconds<10?1:0)}s`;
  let m=Math.floor(seconds/60), s=Math.round(seconds%60).toString().padStart(2,'0');
  return `${m}:${s}`;
}
function accelTotal(p){return p.raw_total_g??p.acc_total_g??0}
function activePoint(p){return !!p.payload_recording}
function confirmClick(button,label,action){
  if(button.disabled)return;
  let now=Date.now(), pending=pendingClicks.get(label);
  if(pending && now-pending.time<=DOUBLE_CLICK_MS){
    clearTimeout(pending.timer);
    pendingClicks.delete(label);
    button.textContent=pending.original;
    action();
    return;
  }
  for(let [l, p] of pendingClicks.entries()){
    clearTimeout(p.timer);
    if(p.button) p.button.textContent = p.original;
  }
  pendingClicks.clear();
  let original=button.dataset.originalText||button.textContent||label;
  button.dataset.originalText=original;
  button.textContent='Again';
  substatusEl.textContent=`Click ${label} again within 3 seconds.`;
  let timer=setTimeout(()=>{
    let current=pendingClicks.get(label);
    if(current && current.time===now){
      pendingClicks.delete(label);
      button.textContent=current.original;
    }
  },DOUBLE_CLICK_MS);
  pendingClicks.set(label,{time:now,timer,original,button});
}
function stoppedPoint(p){let s=(p.flight_state||'').toUpperCase(); return !landed && (launched||recording||flight.length>0||recorded.length>0) && ['LANDED','STOPPED','RECOVERY'].includes(s) && !p.payload_recording && accelTotal(p)<2}
function displayFlightState(p){return p?.flight_state||'IDLE'}
function displayAltitude(p){return Number.isFinite(+p?.alt_m)?+p.alt_m:0}
function averageVerticalSpeed(){
  let pts=series().filter(p=>p&&p.flight_state!=='connect'&&Number.isFinite(+xTime(p))&&Number.isFinite(+p.alt_m));
  if(pts.length<2)return null;
  let last=pts[pts.length-1], tLast=xTime(last);
  let first=pts.find(p=>tLast-xTime(p)<=5000) || pts[0];
  let elapsed=(xTime(last)-xTime(first))/1000;
  if(elapsed<=0)return null;
  let totalChange=(last.alt_m||0)-(first.alt_m||0);
  return totalChange/elapsed;
}
function landedClone(p){return {...p, received_at:Date.now()/1000, alt_m:0, flight_state:'LANDED', payload_recording:false}}
function validLatLon(p){return p && Math.abs(p.lat)>1e-6 && Math.abs(p.lon)>1e-6}
function selectedImportPoint(){return viewingImport&&importIndex>=0?series()[Math.min(importIndex,series().length-1)]:null}
function xTime(p){return p.plot_time_ms??p.time_ms}
function previousTelemetryPoint(pts,index){
  for(let i=index-1;i>=0;i--){
    if(pts[i] && pts[i].flight_state!=='connect')return pts[i];
  }
  return null;
}
function previousValidTelemetryPoint(pts,index){
  for(let i=index-1;i>=0;i--){
    if(pts[i] && pts[i].flight_state!=='connect' && validLatLon(pts[i]))return pts[i];
  }
  return null;
}
function nearestValidPoint(pts,index){
  if(!pts.length)return null;
  index=Math.max(0,Math.min(index,pts.length-1));
  for(let i=index;i>=0;i--) if(validLatLon(pts[i])) return pts[i];
  for(let i=index+1;i<pts.length;i++) if(validLatLon(pts[i])) return pts[i];
  return null;
}
function gapDurationSeconds(pts,index){
  let p=pts[index], prev=previousTelemetryPoint(pts,index);
  if(!p||!prev)return 0;
  let gapStart=Number.isFinite(+p.disconnect_start_ms)?+p.disconnect_start_ms:xTime(prev);
  let plotGap=(xTime(p)-gapStart)/1000;
  let wallGap=Number.isFinite(+p.received_at)&&Number.isFinite(+prev.received_at)?(+p.received_at)-(+prev.received_at):NaN;
  return Number.isFinite(wallGap)&&wallGap>0?Math.max(plotGap,wallGap):plotGap;
}
function shouldDrawDisconnectGap(pts,index){
  if(!SHOW_TIMED_GAPS)return false;
  let p=pts[index];
  return !!(p&&p.flight_state!=='connect'&&gapDurationSeconds(pts,index)>=DISCONNECT_MARKER_MIN_S);
}
function activeTimelineMode(){
  return viewingImport||recording||launched||landed||flight.length>0||recorded.length>0;
}
function applyLocalAltitude(p){
  return p;
}
function sanitizeAltitude(p){
  return p;
}
function shouldSkipLowAltRepeat(list,p){
  if(!list.length)return false;
  let alt=Math.max(0,p.alt_m||0);
  if(alt>LOW_ALT_MAX_M)return false;
  if(Math.max(0,list[list.length-1].alt_m||0)>LOW_ALT_MAX_M)return false;
  let runStart=list[list.length-1];
  for(let i=list.length-1;i>=0;i--){
    if(Math.max(0,list[i].alt_m||0)>LOW_ALT_MAX_M)break;
    runStart=list[i];
  }
  return (xTime(p)-xTime(runStart))/1000>LOW_ALT_KEEP_S;
}
function normalizeIncomingPoint(p){
  let raw=Number.isFinite(p.time_ms)?p.time_ms:0;
  let received=Number.isFinite(+p.received_at)?+p.received_at:Date.now()/1000;
  if(timelineLastTime!==null){
    let rawPlot = raw + timelineOffset;
    let wallAdvance = Math.max(0, received - (lastPacketWall || received)) * 1000;
    if(rawPlot <= timelineLastTime || rawPlot - timelineLastTime > wallAdvance + 5000){
      timelineOffset=timelineLastTime-raw+Math.min(1000, Math.max(20, wallAdvance));
    }
  }
  p.plot_time_ms=raw+timelineOffset;
  timelineLastTime=p.plot_time_ms;
  lastPacketWall=received;
  lastUiPacketAt=Date.now();
}
function addPoint(p){
  normalizeIncomingPoint(p);
  const payloadIsRecording=activePoint(p);
  const payloadJustStopped=lastPayloadRecording===true && !payloadIsRecording;
  lastPayloadRecording=payloadIsRecording;
  all.push(p); if(all.length>120000) all.shift();
  let launchedThisPoint=false;
  if(!launched && activePoint(p) && !stopCommandSent && !suppressAutoRecording){
    launched=true; landed=false; graphBase=p.time_ms; launchedThisPoint=true;
    startRecording(false,p);
  }
  if(landed && activePoint(p) && !stopCommandSent && !suppressAutoRecording && !stoppedPoint(p)){
    landed=false;
    if(!launched) launched=true;
    if(recording && recorded[recorded.length-1]!==p) recorded.push(p);
    substatusEl.textContent='Telemetry resumed after reconnect';
  }
  if(launched && !landed && flight[flight.length-1]!==p){ flight.push(p); if(flight.length>240000) flight.shift(); }
  if(recording && recorded[recorded.length-1]!==p){ recorded.push(p); if(recorded.length>240000) recorded.shift(); }
  if(((stopCommandSent && !payloadIsRecording)||payloadJustStopped) && recording) stopRecording(false,p,true);
  // Old landing packets are not acknowledgements of this recording session.
  // Only the recording flag transition above may end an active recording.
  if(!recording && !suppressAutoRecording && !launchedThisPoint && stoppedPoint(p)){
    let wasLanded=landed;
    let hadActiveFlight=launched || recording || flight.length>0;
    landed=true;
    if(!wasLanded){
      let freezePoint=landedClone(p);
      if(flight.length){
        if(flight[flight.length-1]!==p) flight.push(freezePoint);
      }else{
        flight.push(freezePoint);
      }
      if(recorded.length){
        if(recorded[recorded.length-1]!==p) recorded.push(freezePoint);
      }else{
        recorded.push(freezePoint);
      }
      if(hadActiveFlight) stopRecording(false, freezePoint);
      else { recording=false; requestGraph(true); }
    }
  }
  if(!processingTelemetry){
    updateStats(p);
    if(activeTab==='graphs') requestGraph();
    if(activeTab==='map') requestMap();
    if(activeTab==='view3d') request3d();
    if(activeTab==='viewWind') requestWind();
    if(activeTab==='mobile') requestMobile();
  }
  updateControls();
}
function setRecordingIndicator(active){
  sCamState.textContent=active?'RECORDING':'IDLE';
  sCamState.style.color=active?'#ef4444':'#22c55e';
}
function updateStats(p){
    let shownAlt=displayAltitude(p);
    let avgSpeed=averageVerticalSpeed();
    sAlt.textContent=formatAltMeters(shownAlt); sAcc.textContent=p.acc_total_g.toFixed(2)+' g'; sSats.textContent=p.sats;
    sAccX.textContent=(p.acc_x_g??0).toFixed(2)+' g'; sAccY.textContent=(p.acc_y_g??0).toFixed(2)+' g'; sAccZ.textContent=(p.acc_z_g??0).toFixed(2)+' g';
    sHighAcc.textContent=(p.high_acc_total_g??0).toFixed(2)+' g';
    sHighAccXYZ.textContent=`${(p.high_acc_x_g??0).toFixed(1)}/${(p.high_acc_y_g??0).toFixed(1)}/${(p.high_acc_z_g??0).toFixed(1)} g`;
    setRecordingIndicator(recording && !stopCommandSent);
    sSdFrames.textContent=p.sd_frames??0;
    sSdFree.textContent=((p.sd_free_kb??0)/1024).toFixed(1)+' MB';
    sPressure.textContent=formatPressure(p.pressure_hpa);
    sMpx1.textContent=p.mpx1??512; sMpx2.textContent=p.mpx2??512; sMpx3.textContent=p.mpx3??512;
    sWindSpeed.textContent=formatVerticalSpeed(p.wind_speed_m_s||0); sWindDir.textContent=(p.wind_dir_deg||0).toFixed(1)+' \u00B0';
    sLat.textContent=validLatLon(p)?p.lat.toFixed(6):'--'; sLon.textContent=validLatLon(p)?p.lon.toFixed(6):'--';
    locationBox.value=validLatLon(p)?`${p.lat.toFixed(6)}, ${p.lon.toFixed(6)}`:'--, --';
    sRssi.textContent=(p.rssi??'--')+' dBm'; sFlight.textContent=displayFlightState(p); sAvgSpeed.textContent=avgSpeed===null?`-- ${speedUnit()}`:formatVerticalSpeed(avgSpeed); sPackets.textContent=packetCount;
    statusEl.textContent=viewingImport?`Imported ${series().length} row${series().length===1?'':'s'}`:`Receiving ${packetCount} packet${packetCount===1?'':'s'}`;
    let windStr = '';
    if (series().some(d =>
        d.wind_speed_m_s > 0 ||
        [d.mpx1,d.mpx2,d.mpx3].some(v => Number.isFinite(v) && v > 0 && Math.abs(v - 512) > 10))) {
      windStr = ` | wind ${formatVerticalSpeed(p.wind_speed_m_s||0, 1)} `;
    }
    latest.textContent=`${p.time_ms.toFixed(0)}, ${p.lat.toFixed(6)}, ${p.lon.toFixed(6)}, alt ${formatAltMeters(shownAlt,2)} | pressure ${formatPressure(p.pressure_hpa)} | temp ${formatTemp(p.temp_c)} | acc ${(p.acc_x_g??0).toFixed(2)}/${(p.acc_y_g??0).toFixed(2)}/${(p.acc_z_g??0).toFixed(2)} | high-g ${(p.high_acc_total_g??0).toFixed(1)}g | SD ${p.sd_frames??0} fr ${windStr}| ${displayFlightState(p)}`;
    updateControls();
  }
function updateTelemetryStatusFromServer(message=''){
  if(message)serverStatusMessage=message;
  if(viewingImport)return;
  let lower=(serverStatusMessage||'').toLowerCase();
  if(lower.includes('no serial')||lower.includes('missing')||lower.includes('disconnected')||lower.includes('no telemetry')){
    statusEl.textContent='Rocket link not receiving telemetry';
  }else if(!lastUiPacketAt){
    statusEl.textContent='Waiting for telemetry';
  }
}
function updateConnectionHealth(){
  if(viewingImport||(!ws||ws.readyState!==1)&&!window.AndroidBridge)return;
  if(!lastUiPacketAt){
    updateTelemetryStatusFromServer();
    return;
  }
  let age=Date.now()-lastUiPacketAt;
  if(!activeTimelineMode()){
    if(age>IDLE_UI_STALE_MS){
      statusEl.textContent='Idle, waiting for telemetry';
      if(!substatusEl.textContent.includes('Auto-saved')) substatusEl.textContent='No active flight. Waiting for launch or recording.';
    }
    return;
  }
  if(age>UI_STALE_MS){
    statusEl.textContent=`No packets for ${(age/1000).toFixed(1)}s`;
    if(!substatusEl.textContent.includes('Auto-saved')) substatusEl.textContent='Check ESP32 power, USB cable, LoRa antenna, baud rate, and COM port.';
  }
}
function updateCsvReview(index){
  updateGraphTimeline(index);
  requestGraph(true); requestMap(false); request3d(true);
}

function updateBindSelect() {
  bindSelect.innerHTML = '';
  if(!boundRocketId || !knownRockets[boundRocketId]) {
     let unb = document.createElement('option');
     unb.value = '';
     unb.textContent = 'Unbound';
     unb.selected = true;
     bindSelect.appendChild(unb);
  }
  for (let id in knownRockets) {
    let opt = document.createElement('option');
    opt.value = id;
    let name = knownRockets[id];
    opt.textContent = `${name}`; // Removed (id) from the name since it's displayed below now
    if (id === boundRocketId) opt.selected = true;
    bindSelect.appendChild(opt);
  }
  let newOpt = document.createElement('option');
  newOpt.value = '_new';
  newOpt.textContent = '+ Add New ID...';
  bindSelect.appendChild(newOpt);
  
  if(boundRocketId && knownRockets[boundRocketId]){
    renameBtn.style.display = 'inline-block';
    boundIdLabel.textContent = `ID: ${boundRocketId}`;
  } else {
    renameBtn.style.display = 'none';
    boundIdLabel.textContent = '';
  }
}
updateBindSelect();

function renameBoundRocket() {
  if (!boundRocketId) return;
  let currentName = knownRockets[boundRocketId] || 'Rocket';
  let newName = prompt("Enter new name for this rocket. It will be sent over LoRa to rename the hardware permanently:", currentName);
  if (newName !== null) {
    newName = newName.trim().substring(0, 12);
    if (newName) {
       sendCommand('RENAME,' + boundRocketId + ',' + newName);
       substatusEl.textContent = `Rename command sent to ${boundRocketId}. Waiting for payload confirmation...`;
    }
  }
}

function handleBindSelect(val) {
  if (val === '_new') {
    let id = prompt("Enter new Rocket ID to bind:");
    if (id && id.trim()) {
      id = id.trim();
      if(!knownRockets[id]) knownRockets[id] = `Rocket`;
      bindRocketTo(id);
    } else {
      updateBindSelect();
    }
  } else if (val) {
    bindRocketTo(val);
  }
}

function bindRocketTo(id) {
  boundRocketId = id;
  if(ws && ws.readyState===1) ws.send(JSON.stringify({type:'bind', id: id}));
  localStorage.setItem('boundRocketId', id);
  localStorage.setItem('knownRockets', JSON.stringify(knownRockets));
  updateBindSelect();
  latestLive = null;
  resetDashboard('Switched rocket. Waiting for telemetry...', true);
}

function enqueueTelemetry(point,total){
  packetCount=total;
  let id = point.rocket_id || '12345678';
  if(boundRocketId && id !== boundRocketId) return; // Drop unbound packets
  if(id === boundRocketId) {
    let name = point.rocket_name || 'Rocket';
    if (knownRockets[id] !== name) {
      knownRockets[id] = name;
      localStorage.setItem('knownRockets', JSON.stringify(knownRockets));
      updateBindSelect();
    }
  }

  latestLive=point;
  if(viewingImport)return;
  if((telemetryQueue.length && telemetryQueue.at(-1).time_ms===point.time_ms) || (!telemetryQueue.length && all.length && all.at(-1).time_ms===point.time_ms))return;
  telemetryQueue.push(point);
}

function flushTelemetryQueue(forceAll=false){
  if(viewingImport)return;
  if(!telemetryQueue.length){
    if(substatusEl.textContent.startsWith('Processing ')&&substatusEl.textContent.endsWith(' packets...')) substatusEl.textContent='';
    return;
  }
  processingTelemetry=true;
  let last=null, count=forceAll?telemetryQueue.length:Math.min(MAX_QUEUE_PER_FLUSH,telemetryQueue.length);
  let batch=telemetryQueue.splice(0,count);
  for(let point of batch){
    last=point;
    addPoint(last);
  }
  processingTelemetry=false;
  if(!last)return;
  updateStats(last);
  if(telemetryQueue.length) substatusEl.textContent=`Processing ${telemetryQueue.length} packets...`;
  else if(substatusEl.textContent.startsWith('Processing ')&&substatusEl.textContent.endsWith(' packets...')) substatusEl.textContent='';
  if(activeTab==='graphs') requestGraph();
  if(activeTab==='map') requestMap(false);
  if(activeTab==='view3d') request3d(true);
  if(activeTab==='viewWind') requestWind();
  if(activeTab==='mobile') requestMobile(false);
  updateControls();
}
function connect(){
  if(!location.host) return;
  if(ws && (ws.readyState===0 || ws.readyState===1)) return;
  ws=new WebSocket(`ws://${location.host}/ws`);
  ws.onopen=()=>{reconnectDelay=800; statusEl.textContent=latestLive?`Dashboard ready, last seen ${packetCount} packet${packetCount===1?'':'s'}`:'Waiting for telemetry'; if(boundRocketId) ws.send(JSON.stringify({type:'bind', id: boundRocketId})); updateControls()};
  ws.onerror=()=>statusEl.textContent='WebSocket error';
  ws.onclose=()=>{statusEl.textContent='Disconnected, continuing when it reconnects...'; updateControls(); clearTimeout(reconnectTimer); reconnectTimer=setTimeout(connect,reconnectDelay); reconnectDelay=Math.min(5000,reconnectDelay*1.5)};
  ws.onmessage=e=>{
    let m;
    try{m=JSON.parse(e.data)}catch{ substatusEl.textContent='Ignored malformed dashboard message'; return }
    if(m.type==='telemetry'&&m.point) enqueueTelemetry(m.point,m.total);
    else if(m.type==='status'){ substatusEl.textContent=m.message||''; updateTelemetryStatusFromServer(m.message||''); }
    else if(m.type==='hello') statusEl.textContent=m.total?`Dashboard ready (${m.total} packet${m.total===1?'':'s'} seen)`:'Waiting for telemetry';
  };
}
function sendCommand(c){
  try {
    if(window.AndroidBridge && window.AndroidBridge.sendCommand) return window.AndroidBridge.sendCommand(c)!==false;
    if(ws&&ws.readyState===1){ws.send(JSON.stringify({type:'command',command:c})); return true;}
    statusEl.textContent='Not connected';
  } catch(e) {substatusEl.textContent='Command failed: '+e.message;}
  return false;
}
const PAYLOAD_WIFI_SSID='XenoCam-1333877';
const PAYLOAD_WIFI_PASSWORD='XenoCam915';
const PAYLOAD_FILES_URL='http://192.168.4.1/';
let payloadWifiRequested=false;
function setPayloadStorage(mode){
  if(!['INTERNAL','SD','BOTH'].includes(mode)) return;
  try{localStorage.setItem('payloadStorageMode',mode)}catch{}
  sendCommand('STORAGE,'+mode);
  const label=mode==='BOTH'?'internal flash and SD card':mode==='INTERNAL'?'internal flash only':'SD card only';
  substatusEl.textContent='Payload recording destination set to '+label;
}
function showPayloadWifiState(on){
  payloadWifiRequested=on;
  const button=document.getElementById('payloadWifiBtn');
  if(button){button.textContent=on?'Wi-Fi Off':'Wi-Fi On';button.style.background=on?'#b45309':''}
}
function togglePayloadWifi(){
  if(payloadWifiRequested){
    sendCommand('WIFI_OFF');
    showPayloadWifiState(false);
    substatusEl.textContent='Payload Wi-Fi off requested';
    return;
  }
  sendCommand('WIFI_ON');
  showPayloadWifiState(true);
  substatusEl.textContent=`Connect to ${PAYLOAD_WIFI_SSID} (password ${PAYLOAD_WIFI_PASSWORD}), then click Flight Files`;
}
function openPayloadFiles(){
  substatusEl.textContent=`Opening ${PAYLOAD_FILES_URL}; first connect this device to ${PAYLOAD_WIFI_SSID}`;
  window.location.href=PAYLOAD_FILES_URL;
}
window.androidReceiveTelemetry = function(point, total) {
  if (statusEl.textContent === 'Connecting...') statusEl.textContent = 'Waiting for telemetry';
  enqueueTelemetry(point, total);
};
window.androidUsbStatus = function(message) {
  substatusEl.textContent = message;
  if (statusEl.textContent === 'Connecting...') statusEl.textContent = 'Waiting for telemetry';
};
function startRecording(manual=false, seed=null){
  window.lastBackupCount=0;
  if(manual) {
    if(recording) return;
    flushTelemetryQueue(true);
    if(recording) return;
    const destination=document.getElementById('storageSelect')?.value||'BOTH';
    // Keep compatibility with payloads already in the field: they understand
    // these two established commands but not a combined Start command.
    if(!sendCommand('STORAGE,'+destination) || !sendCommand('START_RECORD')) return;
    suppressAutoRecording=false;
    if(stopFinalizeTimer){clearTimeout(stopFinalizeTimer);stopFinalizeTimer=null;}
    recording=true; csvExported=false; stopCommandSent=false; stopRequestedAt=0; recordStartedAt=Date.now();
    lastPayloadRecording=false;
    setRecordingIndicator(true);
    recorded=[]; flight=[]; graphBase=latestLive?latestLive.time_ms:null; landed=false;
    let baseline=latestLive || all.at(-1);
    if(baseline){ recorded.push(baseline); flight.push(baseline); }
    substatusEl.textContent='Start sent; collecting live packets while waiting for payload confirmation.';
    updateControls();
    return;
  }
  if(importing) return;
  if(recording) return;
  recording=true; csvExported=false; stopCommandSent=false; stopRequestedAt=0; recordStartedAt=Date.now();
  setRecordingIndicator(true);
  recorded=[]; flight=[]; graphBase=seed?seed.time_ms:null; landed=false;
  let baseline=seed || latestLive || all.at(-1);
  if(seed){ recorded.push(seed); flight.push(seed); }
  substatusEl.textContent='Payload recording started';
  updateControls();
}
function resetAfterManualStop(){
  telemetryQueue=[]; currentRoll=0; currentPitch=0; currentYaw=0; lastGyroTimeMs=null; processingTelemetry=false; all=[]; flight=[]; recorded=[]; graphBase=null; launched=false; landed=false; recording=false; stopCommandSent=false; graphIndex=-1; graphFollow=true; mapIndex=-1; mapFollow=true; view3dIndex=-1; view3dFollow=true; timelineLastTime=null; timelineOffset=0; lastPacketWall=null; pendingDisconnectStart=null; localAltZero=null;
  if(pathLine) pathLine.setLatLngs([]);
  if(mapActiveLine) mapActiveLine.setLatLngs([]);
  if(mobilePathLine) mobilePathLine.setLatLngs([]);
  if(mobileActiveLine) mobileActiveLine.setLatLngs([]);
  gapLines.forEach(line=>map&&map.removeLayer(line)); gapLines=[];
  if(rocketMarker && map){map.removeLayer(rocketMarker); rocketMarker=null}
  if(launchMarker && map){map.removeLayer(launchMarker); launchMarker=null}
  sLat.textContent='--'; sLon.textContent='--'; locationBox.value='--, --';
  sFlight.textContent='IDLE';
  latest.textContent='IDLE';
  requestGraph(true); requestMap(true); request3d(true);
  updateControls();
}
function stopRecording(manual=false, seed=null, payloadConfirmed=false, forceFinalize=false){
  if(importing){ recording=false; return; }
  if(manual){
    if(stopCommandSent)return;
    flushTelemetryQueue(true);
    if(!sendCommand('STOP_RECORD')) return;
    suppressAutoRecording=true;
    stopCommandSent=true;
    stopRequestedAt=Date.now();
    setRecordingIndicator(false);
    substatusEl.textContent='Stop sent; collecting final LoRa packets until payload confirms.';
    updateControls();
    if(stopFinalizeTimer)clearTimeout(stopFinalizeTimer);
    stopFinalizeTimer=setTimeout(()=>{
      if(stopCommandSent&&recording) stopRecording(false,latestLive||all.at(-1)||null,false,true);
    },STOP_ACK_TIMEOUT_MS);
    return;
  }
  if(!manual && !seed){
    let base=recorded.at(-1)||flight.at(-1)||all.at(-1);
    if(base) seed=landedClone(base);
  }
  if(seed && recording && recorded[recorded.length-1]!==seed) recorded.push(seed);
  if(seed && flight[flight.length-1]!==seed) flight.push(seed);
  if(seed && all[all.length-1]!==seed) all.push(seed);
  landed=!(payloadConfirmed||forceFinalize);
  recording=false;
  setRecordingIndicator(false);
  stopCommandSent=false;
  stopRequestedAt=0;
  if(stopFinalizeTimer){clearTimeout(stopFinalizeTimer);stopFinalizeTimer=null}
  if(payloadConfirmed || forceFinalize){
    launched=false;
    csvExported=true;
    let exportRows=recorded.filter(p=>p&&p.flight_state!=='connect');
    if(!exportRows.length) exportRows=flight.filter(p=>p&&p.flight_state!=='connect');
    if(!exportRows.length) exportRows=all.filter(p=>p&&p.flight_state!=='connect');
    let savedRows=exportRows.length;
    exportCsv(exportRows,'lora_stop');
    substatusEl.textContent=savedRows
      ? `${payloadConfirmed?'Payload confirmed stopped':'Stop acknowledgement timed out'}. CSV downloaded from ${savedRows} live LoRa packet${savedRows===1?'':'s'}; payload sensor log finalization was requested.`
      : 'Recording stopped. Diagnostic CSV downloaded: no LoRa telemetry rows were received.';
    requestGraph(true);
    updateControls();
    return;
  }
  requestGraph(true);
  csvExported=true;
  substatusEl.textContent=manual?`Recording stopped, back to IDLE. ${recorded.length} row${recorded.length===1?'':'s'} saved live on the server`:`Recording stopped, ${recorded.length} row${recorded.length===1?'':'s'} saved live on the server`;
  if(seed) updateStats(seed); else sFlight.textContent=manual?'IDLE':'LANDED';
}
function csvEscape(v){
  let s=String(v??'');
  return /[",\r\n]/.test(s)?`"${s.replaceAll('"','""')}"`:s;
}
function exportCsv(rows, reason){
  let head=['time_ms','latitude','longitude','altitude_m','satellites','temp_c','pressure_hpa','acc_x_g','acc_y_g','acc_z_g','gyro_x_dps','gyro_y_dps','gyro_z_dps','high_accel_x_g','high_accel_y_g','high_accel_z_g','pitch_deg','roll_deg','received_at','rssi','flight_state','payload_recording','records_written','sd_free_kb','rocket_id','rocket_name','raw_line'];
  let aliases={latitude:'lat',longitude:'lon',altitude_m:'alt_m',satellites:'sats',high_accel_x_g:'high_acc_x_g',high_accel_y_g:'high_acc_y_g',high_accel_z_g:'high_acc_z_g',pitch_deg:'pitch',roll_deg:'roll',records_written:'sd_frames'};
  let body=rows.map(p=>head.map(k=>csvEscape(p[aliases[k]||k])).join(','));
  let csv=head.join(',')+'\r\n'+body.join('\r\n')+'\r\n';
  let stamp=new Date().toISOString().replace(/[:.]/g,'-');
  let filename=`rocket_${reason}_${stamp}.csv`;
  if(window.AndroidBridge && window.AndroidBridge.saveCsv) {
    window.AndroidBridge.saveCsv(filename, csv);
    return;
  }
  let a=document.createElement('a');
  a.href=URL.createObjectURL(new Blob([csv],{type:'text/csv'}));
  a.download=filename;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(()=>URL.revokeObjectURL(a.href),4000);
}

function resetFlight(manual=false){
  if(manual) sendCommand('RESET_FLIGHT');
  latestLive=null;
  resetDashboard(manual?'Reset sent. Automatic launch detection is disabled.':'Dashboard reset, live feed restored', true);
}
function resetDashboard(message='Dashboard reset, live feed restored', forceKick=false){
  if (viewingImport && !forceKick) {
    if(statusEl) statusEl.textContent = message;
    telemetryQueue=[]; currentRoll=0; currentPitch=0; currentYaw=0; lastGyroTimeMs=null; processingTelemetry=false; lastUiPacketAt=0;
    return;
  }
  viewingImport=false; importing=false; importIndex=-1; csvReview.style.display='none';
  suppressAutoRecording=false; lastPayloadRecording=null;
  if(stopFinalizeTimer){clearTimeout(stopFinalizeTimer);stopFinalizeTimer=null;}
  telemetryQueue=[]; currentRoll=0; currentPitch=0; currentYaw=0; lastGyroTimeMs=null; processingTelemetry=false; all=[]; flight=[]; recorded=[]; graphBase=null; launched=false; landed=false; recording=false; csvExported=false; stopCommandSent=false; mapCenteredOnMe=false; timelineLastTime=null; timelineOffset=0; lastPacketWall=null; pendingDisconnectStart=null; lastUiPacketAt=0; lastIdleUi=0; graphIndex=-1; graphFollow=true; mapIndex=-1; mapFollow=true; view3dIndex=-1; view3dFollow=true; localAltZero=null; mapTimeline.style.display='none'; view3dTimeline.style.display='none';
  if(pathLine) pathLine.setLatLngs([]);
  if(mapActiveLine) mapActiveLine.setLatLngs([]);
  gapLines.forEach(line=>map&&map.removeLayer(line)); gapLines=[];
  if(rocketMarker && map){map.removeLayer(rocketMarker); rocketMarker=null}
  if(launchMarker && map){map.removeLayer(launchMarker); launchMarker=null}
  sAlt.textContent='-- m'; sAcc.textContent='-- g'; sAccX.textContent='-- g'; sAccY.textContent='-- g'; sAccZ.textContent='-- g'; sSats.textContent='--'; sRssi.textContent='-- dBm'; sPressure.textContent='-- hPa'; sLat.textContent='--'; sLon.textContent='--'; locationBox.value='--, --'; sFlight.textContent='IDLE'; sAvgSpeed.textContent=`-- ${speedUnit()}`; sPackets.textContent='0';
  sHighAcc.textContent='-- g'; sHighAccXYZ.textContent='--/--/-- g'; sCamState.textContent='IDLE'; sSdFrames.textContent='0'; sSdFree.textContent='0.0 MB'; sCamState.style.color='#22c55e';
  latest.textContent='No packets yet';
  if(latestLive){
    all=[latestLive];
    updateStats(latestLive);
  }
  requestGraph(true);
  requestMap(true);
  request3d(true);
  requestMobile(true);
  if(!latestLive) statusEl.textContent=(ws&&ws.readyState===1)||window.AndroidBridge?'Waiting for telemetry':'Connecting...';
  substatusEl.textContent=message;
  updateControls();
}
function showTab(id){
  activeTab=id; document.querySelectorAll('.tab').forEach(t=>t.classList.toggle('active',t.id===id));
  if(id==='graphs') requestGraph(true); if(id==='map') requestMap(true); if(id==='view3d') request3d(true); if(id==='viewWind') requestWind(true); if(id==='mobile'){requestMobile(true); setTimeout(()=>mobileMap&&mobileMap.invalidateSize(),80)}
}
function series(){
  if(viewingImport) return recorded.length ? recorded : all;
  if(recording && recorded.length) return recorded;
  if(!launched && !landed) return all.length ? all : (recorded.length ? recorded : flight);
  if(flight.length) return flight;
  return recorded.length ? recorded : all;
}
function pathSeries(){
  if(viewingImport) return recorded.length ? recorded : flight;
  if(recording && recorded.length) return recorded;
  if(!launched && !landed) return all.length ? all : (recorded.length ? recorded : flight);
  return flight.length ? flight : recorded;
}
function resizeCanvas(c){let r=c.getBoundingClientRect(), d=devicePixelRatio||1; if(c.width!==r.width*d||c.height!==r.height*d){c.width=r.width*d;c.height=r.height*d} let ctx=c.getContext('2d'); ctx.setTransform(d,0,0,d,0,0); return [ctx,r.width,r.height]}
function idleGraphMode(){return !viewingImport && !recording && !launched && !landed}
function requestGraph(force=false){let now=performance.now(), wait=idleGraphMode()?IDLE_GRAPH_MS:ACTIVE_RENDER_MS; if(!force && now-lastGraph<wait)return; lastGraph=now; requestAnimationFrame(drawGraphs)}
function graphSeries(){return series();}
function updateGraphTimeline(index=null){
  let pts=graphSeries();
  if(!pts.length){csvReview.style.display='none'; graphIndex=-1; importIndex=-1; return null}
  let max=pts.length-1;
  if(index!==null){graphIndex=Math.max(0,Math.min(index,max)); graphFollow=graphIndex>=(max-2)}
  if(graphFollow||graphIndex<0||graphIndex>max){graphIndex=max; graphFollow=true}
  importIndex=graphIndex;
  csvReview.style.display='flex'; csvSlider.max=String(max); csvSlider.value=String(graphIndex);
  let p=pts[graphIndex];
  if(p.disconnect_gap && p.flight_state === 'connect') {
    csvPointLabel.textContent=`${graphIndex+1}/${pts.length}  --- DISCONNECTED ---`;
    return p;
  }
  let elapsed=(xTime(p)-xTime(pts[0]))/1000;
  csvPointLabel.textContent=`${graphIndex+1}/${pts.length}  t+${elapsed.toFixed(1)} s  ${xTime(p).toFixed(0)} ms  alt ${formatAltMeters(displayAltitude(p))}  pressure ${formatPressure(p.pressure_hpa)}  lat ${validLatLon(p)?p.lat.toFixed(6):'--'}  lon ${validLatLon(p)?p.lon.toFixed(6):'--'}  ${displayFlightState(p)}`;
  updateStats(p);
  return p;
}
let globalZoomY = 1.0;
function updateGlobalZoom(val) {
  globalZoomY = parseFloat(val);
  let lbl = document.getElementById('zoomYLabel');
  if(lbl) lbl.textContent = globalZoomY.toFixed(1) + 'x';
  requestGraph(true);
}

let globalZoomX = 10000;
function updateGlobalZoomX(val) {
  let v = parseInt(val);
  let lbl = document.getElementById('zoomXLabel');
  if (v === 100) {
      globalZoomX = 1e12; // Infinity / All Data
      if(lbl) lbl.textContent = 'All';
  } else {
      globalZoomX = 100 * Math.pow(10, v / 25.0);
      if(lbl) lbl.textContent = (globalZoomX / 1000).toFixed(1) + ' s';
  }
  requestGraph(true);
}

function drawGraphs(){
  if(activeTab!=='graphs')return; 
  let pts=graphSeries(); 
  updateGraphTimeline(); 
  
  let showWind = false, showMpx = false;
  for(let i=0; i<pts.length; i++) {
    if(pts[i].wind_speed_m_s) showWind = true;
    if([pts[i].mpx1,pts[i].mpx2,pts[i].mpx3].some(
        v => Number.isFinite(v) && v > 0 && Math.abs(v - 512) > 10)) showMpx = true;
  }
  document.getElementById('panelWindSpeed').style.display = showWind ? 'block' : 'none';
  document.getElementById('panelMpx').style.display = showMpx ? 'block' : 'none';

  charts.forEach(chart=>{
    let el = document.getElementById(chart.id);
    if(el && el.offsetParent !== null) {
      drawChart(el, pts, chart.lines, chart.pad, chart.id);
    }
  });
}
function drawChart(c, pts, lines, pad, chartId=''){
  let [ctx,w,h]=resizeCanvas(c); ctx.clearRect(0,0,w,h); ctx.fillStyle='white'; ctx.fillRect(0,0,w,h);
  if(!pts.length){ctx.fillStyle='#9ca3af';ctx.fillText('Waiting for telemetry',w/2-55,h/2);return}
  let data = [];
  let startIndex = 0;
  if (pts.length > 0) {
      let gIdx = graphIndex >= 0 ? Math.min(graphIndex, pts.length - 1) : pts.length - 1;
      let targetTime = xTime(pts[gIdx]);
      let windowDuration = globalZoomX; // dynamically updated true time window
      let windowStart, windowEnd;
      
      if (graphFollow || gIdx >= pts.length - 1) {
          windowEnd = xTime(pts[pts.length - 1]);
          windowStart = windowEnd - windowDuration;
      } else {
          windowStart = targetTime - windowDuration / 2;
          windowEnd = targetTime + windowDuration / 2;
          let minTime = xTime(pts[0]);
          let maxTime = xTime(pts[pts.length - 1]);
          if (windowStart < minTime) { windowStart = minTime; windowEnd = minTime + windowDuration; }
          if (windowEnd > maxTime) { windowEnd = maxTime; windowStart = maxTime - windowDuration; }
      }
      
      let sIdx = 0;
      let eIdx = pts.length - 1;
      for (let i = gIdx; i >= 0; i--) {
          if (xTime(pts[i]) <= windowStart) { sIdx = i; break; }
      }
      for (let i = gIdx; i < pts.length; i++) {
          if (xTime(pts[i]) >= windowEnd) { eIdx = i; break; }
      }
      startIndex = sIdx;
      data = pts.slice(sIdx, eIdx + 1);
  }
  let t0=xTime(data[0]), globalT0=xTime(pts[0]), xs=data.map(p=>(xTime(p)-globalT0)/1000);
  let visibleData=data.filter(p=>p.flight_state!=='connect');
  let allYs=lines.flatMap(line=>(visibleData.length?visibleData:data).map(line.get)), minY=Math.min(...allYs), maxY=Math.max(...allYs);
  
  if (globalZoomY > 1.0) {
      let range = maxY - minY;
      let newRange = range / globalZoomY;
      if (minY < 0 && maxY > 0) {
          let scale = 1.0 / globalZoomY;
          minY = minY * scale;
          maxY = maxY * scale;
      } else if (minY >= 0) {
          maxY = minY + newRange;
      } else {
          minY = maxY - newRange;
      }
  }
  
  if(Math.abs(maxY-minY)<1e-6){minY-=pad;maxY+=pad}else{let s=maxY-minY;minY-=s*.12;maxY+=s*.12}
  let minX=xs[0], maxX=xs[xs.length-1]||1, L=48,R=8,T=8,B=22, pw=w-L-R, ph=h-T-B;
  if(maxX-minX<5.0) maxX=minX+5.0;
  ctx.strokeStyle='#e5e7eb';ctx.lineWidth=1;
  ctx.fillStyle='#9ca3af'; ctx.font='10px Segoe UI';
  for(let i=0;i<5;i++){
      let y=T+ph*i/4;
      ctx.beginPath();ctx.moveTo(L,y);ctx.lineTo(w-R,y);ctx.stroke();
      let val = maxY - (i/4)*(maxY-minY);
      ctx.fillText(val.toFixed(1), 4, y + (i===0 ? 8 : i===4 ? -2 : 4));
  }
  ctx.strokeStyle='#f3f4f6'; for(let i=0;i<5;i++){let x=L+pw*i/4;ctx.beginPath();ctx.moveTo(x,T);ctx.lineTo(x,h-B);ctx.stroke()}
  let px=x=>L+(x-minX)/(maxX-minX||1)*pw, py=y=>T+(1-(y-minY)/(maxY-minY||1))*ph;
  lines.forEach(line=>{
    let ys=data.map(line.get);
    ctx.strokeStyle=line.color;ctx.lineWidth=2.5;ctx.setLineDash([]);ctx.beginPath();
    let isDrawing = false;
    data.forEach((p,i)=>{
      let x=px(xs[i]),y=py(ys[i]);
      if(p.disconnect_gap && p.flight_state === 'connect') {
        return;
      }
      let breaksLine=i&&shouldDrawDisconnectGap(pts,startIndex+i);
      if(!isDrawing || breaksLine){ctx.moveTo(x,y); isDrawing = true;} else ctx.lineTo(x,y);
    });
    ctx.stroke();
    let lastDrawable=data.length-1;
    while(lastDrawable>0&&data[lastDrawable].flight_state==='connect')lastDrawable--;
    ctx.fillStyle=line.color;ctx.beginPath();ctx.arc(px(xs[lastDrawable]),py(ys[lastDrawable]),3.5,0,7);ctx.fill();
  });

  ctx.textAlign='center'; ctx.fillText(formatGraphTime(minX),L,h-6); ctx.fillText(formatGraphTime((minX+maxX)/2),L+pw/2,h-6); ctx.fillText(formatGraphTime(maxX),w-R-6,h-6);
  ctx.textAlign='right'; ctx.fillStyle=lines[0].color; ctx.fillText(`t+${formatGraphTime(maxX)}`,w-R,T+12); ctx.textAlign='left';
  let lx=L+6;
  lines.forEach(line=>{let label=typeof line.label==='function'?line.label():line.label;ctx.fillStyle=line.color;ctx.fillRect(lx,T+8,10,3);ctx.fillText(label,lx+14,T+13);lx+=Math.max(42,label.length*7+20)});
  if(graphIndex>=startIndex&&graphIndex<startIndex+data.length){
    let local=graphIndex-startIndex, x=px(xs[local]);
    if (!data[local].disconnect_gap || data[local].flight_state !== 'connect') {
      ctx.strokeStyle='#111827';ctx.lineWidth=1.5;ctx.setLineDash([5,4]);ctx.beginPath();ctx.moveTo(x,T);ctx.lineTo(x,h-B);ctx.stroke();ctx.setLineDash([]);
      ctx.fillStyle='#111827';ctx.beginPath();ctx.arc(x,T+8,4,0,7);ctx.fill();
    }
  }
  let hasDrawnGap = new Set();
  data.forEach((p,i)=>{
    let sourceIndex=startIndex+i;
    if(shouldDrawDisconnectGap(pts,sourceIndex)){
      let disconnectedPoint = previousTelemetryPoint(pts,sourceIndex);
      let startMs=Number.isFinite(+p.disconnect_start_ms)?+p.disconnect_start_ms:(disconnectedPoint?xTime(disconnectedPoint):xTime(p)-500);
      let actPrevX = (startMs-globalT0)/1000;

      // Ensure we don't draw multiple connection lines for the same gap in the current view
      let roundedX = Math.round(actPrevX / 2);
      if (hasDrawnGap.has(roundedX)) return;
      hasDrawnGap.add(roundedX);
      
      let xOff=px(actPrevX), xOn=px(xs[i]);
      if(xOn<L || xOff>w-R) return; // Completely off screen

      ctx.strokeStyle=DISCONNECT_MARKER_COLOR;ctx.lineWidth=1.5;ctx.setLineDash([3,3]);
      if(xOff>=L&&xOff<=w-R){ctx.beginPath();ctx.moveTo(xOff,T);ctx.lineTo(xOff,h-B);ctx.stroke();}
      if(xOn>=L&&xOn<=w-R){ctx.beginPath();ctx.moveTo(xOn,T);ctx.lineTo(xOn,h-B);ctx.stroke();}
      ctx.setLineDash([]);
    }
  });
}
function initMap(){
  if(map || !window.L)return; map=L.map('leaflet',{zoomControl:true,scrollWheelZoom:true,touchZoom:true,doubleClickZoom:true,boxZoom:true,keyboard:true}).setView([36.7783,-119.4179],7);
  map.on('dragstart', () => { mapFollow = false; });
  L.tileLayer('https://a.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png',{maxZoom:19}).addTo(map);
  pathLine=L.polyline([], {color:'#64748b',weight:5,opacity:.28}).addTo(map);
  mapActiveLine=L.polyline([], {color:'#0284c7',weight:5,opacity:.95}).addTo(map);
  if(myLocation) drawMyLocation(false);
}
function requestMap(force=false){let now=performance.now(); if(!force&&now-lastMap<ACTIVE_RENDER_MS)return; lastMap=now; requestAnimationFrame(()=>drawMap(force))}
function updateMapTimeline(pts){
  if(!pts.length){mapTimeline.style.display='none'; mapIndex=-1; return null}
  let max=pts.length-1;
  if(mapFollow||mapIndex<0||mapIndex>max){mapIndex=max; mapFollow=true}
  mapTimeline.style.display='flex';
  mapSlider.max=String(max); mapSlider.value=String(mapIndex);
  let p=pts[mapIndex];
  if(p.disconnect_gap && p.flight_state === 'connect') {
    mapTimeLabel.textContent=`${mapIndex+1}/${pts.length}  --- DISCONNECTED ---`;
    return p;
  }
  let elapsed=(xTime(p)-xTime(pts[0]))/1000;
  let gpsText=validLatLon(p)?`${p.lat.toFixed(6)}, ${p.lon.toFixed(6)}`:'--, --';
  mapTimeLabel.textContent=`${mapIndex+1}/${pts.length}  t+${elapsed.toFixed(1)} s  alt ${formatAltMeters(displayAltitude(p))}  pressure ${formatPressure(p.pressure_hpa)}  ${gpsText}  ${displayFlightState(p)}`;
  return p;
}
function getBearing(p1, p2) {
    if (!p1 || !p2) return 0;
    const toRad = (deg) => deg * Math.PI / 180;
    const lat1 = toRad(p1.lat), lon1 = toRad(p1.lon), lat2 = toRad(p2.lat), lon2 = toRad(p2.lon);
    const dLon = lon2 - lon1;
    const y = Math.sin(dLon) * Math.cos(lat2);
    const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
    let brng = Math.atan2(y, x);
    return (brng * 180 / Math.PI + 360) % 360;
}
function rocketMapBearing(pts,current,index){
  if (current && typeof current.yaw === 'number') return current.yaw;
  let prev=null, next=null;
  for(let i=Math.min(index-1,pts.length-1);i>=0;i--){
    if(validLatLon(pts[i])&&gpsDistanceM(pts[i],current)>0.3){prev=pts[i]; break}
  }
  for(let i=Math.max(index+1,0);i<pts.length;i++){
    if(validLatLon(pts[i])&&gpsDistanceM(current,pts[i])>0.3){next=pts[i]; break}
  }
  if(prev)return getBearing(prev,current);
  if(next)return getBearing(current,next);
  return 0;
}
function mapLatLngSegments(pts,maxIndex=pts.length-1){
  let segments=[], currentSegment=[];
  for(let i=0;i<pts.length&&i<=maxIndex;i++){
    let p=pts[i];
    if(!validLatLon(p)){
      if(currentSegment.length){segments.push(currentSegment); currentSegment=[]}
      continue;
    }
    if(shouldDrawDisconnectGap(pts,i)&&currentSegment.length){segments.push(currentSegment); currentSegment=[]}
    currentSegment.push([p.lat,p.lon]);
  }
  if(currentSegment.length)segments.push(currentSegment);
  return segments;
}
function drawMap(force=false){if(activeTab!=='map')return; initMap(); if(!map)return; let rawPts=repairLatLonTeleports(pathSeries()); let pts=rawPts; let selected=selectedImportPoint(); let live=validLatLon(latestLive)?latestLive:null; let hasMapPoint=pts.some(validLatLon); if(!pts.length&&!live&&!selected){mapTimeline.style.display='none';return}
  let timelinePoint=updateMapTimeline(pts);

  if(pathLine) pathLine.setLatLngs(mapLatLngSegments(pts));
  
  let activeSegments=mapLatLngSegments(pts,Math.max(0,mapIndex));
  
  gapLines.forEach(line=>map.removeLayer(line)); gapLines=[];

  for(let i=0;i<pts.length&&i<=mapIndex;i++){
    let p=pts[i];
    if(shouldDrawDisconnectGap(pts,i)&&validLatLon(p)){
      let prevValid=previousValidTelemetryPoint(pts,i);
      if(prevValid){
        let gapLine=L.polyline([[prevValid.lat,prevValid.lon],[p.lat,p.lon]],{color:DISCONNECT_MARKER_COLOR,weight:3,opacity:.9,dashArray:'5, 10'}).addTo(map);
        gapLines.push(gapLine);
      }
    }
  }
  
  if(mapActiveLine) mapActiveLine.setLatLngs(activeSegments);
  
  let current=(validLatLon(timelinePoint)?timelinePoint:null) || nearestValidPoint(pts,mapIndex) || (validLatLon(selected)?selected:null) || live;
  let coords=pts.filter(validLatLon).map(p=>[p.lat,p.lon]);
  if(coords.length && current && current.flight_state !== 'IDLE'){
    let first=coords[0];
    if(!launchMarker) launchMarker=L.marker(first,{icon:flagMapIcon(),title:'Launch flag'}).addTo(map); else launchMarker.setLatLng(first);
    launchMarker.bindPopup(`Launch<br>${first[0].toFixed(6)}, ${first[1].toFixed(6)}`);
  } else {
    if(launchMarker && map){map.removeLayer(launchMarker); launchMarker=null}
  }
  if(!current){mapTimeline.style.display=pts.length?'flex':'none'; return}
  let last=[current.lat,current.lon];
  if(!rocketMarker) rocketMarker=L.marker(last,{icon:rocketMapIcon(),title:'Rocket'}).addTo(map); else rocketMarker.setLatLng(last);
  
  let currentIndex=pts.indexOf(current);
  if(currentIndex<0)currentIndex=Math.max(0,Math.min(mapIndex,pts.length-1));
  let bearing = rocketMapBearing(pts,current,currentIndex);
  if (rocketMarker && rocketMarker._icon) {
    let rocketIcon = rocketMarker._icon.querySelector('.rocket-marker i');
    if (rocketIcon) rocketIcon.style.transform = `rotate(${bearing+ROCKET_ICON_HEADING_OFFSET_DEG}deg)`;
  }
  
  rocketMarker.bindPopup(`Rocket<br>${current.lat.toFixed(6)}, ${current.lon.toFixed(6)}<br>launch + ${(pts.length?(xTime(current)-xTime(pts[0]))/1000:0).toFixed(1)} s<br>${formatAltMeters(displayAltitude(current))}<br>${displayFlightState(current)}`);
  let windSpeed = current.wind_speed_m_s || 0;
  let windDir = current.wind_dir_deg || 0;
  if(windSpeed > 0.5 && map){
     if(windMarker) map.removeLayer(windMarker);
     let arrowHtml = `<div style="transform: rotate(${windDir}deg); transform-origin: center; font-size: 24px; font-weight: bold; color: #06b6d4; filter: drop-shadow(0 2px 4px #000); text-align:center;">&uarr;</div>`;
     let icon = L.divIcon({className:'', html: arrowHtml, iconSize: [24,24], iconAnchor: [12,12]});
     windMarker = L.marker(last, {icon, title:`Wind: ${windSpeed.toFixed(1)} m/s`}).addTo(map);
  } else if (windMarker && map) {
     map.removeLayer(windMarker); windMarker=null;
  }
  if(mapFollow) {
    if(force) map.setView(last, 16);
    else map.setView(last, map.getZoom());
  }
}
function requestMyLocation(){
  if(!navigator.geolocation){substatusEl.textContent='Browser geolocation is not supported'; return}
  substatusEl.textContent='Requesting location permission...';
  navigator.geolocation.getCurrentPosition(
    pos=>{updateMyLocation(pos); if(activeTab==='map') drawMyLocation(true)},
    err=>{substatusEl.textContent='Location failed: '+err.message},
    {enableHighAccuracy:true, maximumAge:1000, timeout:10000}
  );
  if(myWatchId===null){
    myWatchId=navigator.geolocation.watchPosition(
      pos=>{updateMyLocation(pos); if(activeTab==='map') drawMyLocation(false)},
      err=>{substatusEl.textContent='Location watch failed: '+err.message},
      {enableHighAccuracy:true, maximumAge:1000, timeout:20000}
    );
  }
}
function updateMyLocation(pos){
  myLocation={lat:pos.coords.latitude, lon:pos.coords.longitude, accuracy:pos.coords.accuracy||0, heading:pos.coords.heading};
  substatusEl.textContent=`My location ${myLocation.lat.toFixed(6)}, ${myLocation.lon.toFixed(6)}`;
  if(activeTab==='mobile') requestMobile(false);
}
function drawMyLocation(center=false){
  initMap(); if(!map||!myLocation)return;
  let latlng=[myLocation.lat,myLocation.lon];
  let icon=L.divIcon({className:'', html:'<div style="width:18px;height:18px;border-radius:50%;background:#2563eb;border:3px solid white;box-shadow:0 2px 8px #0008"></div>', iconSize:[24,24], iconAnchor:[12,12]});
  if(!myMarker) myMarker=L.marker(latlng,{icon,title:'My location'}).addTo(map); else myMarker.setLatLng(latlng);
  if(myLocation.accuracy){
    if(!myAccuracyCircle) myAccuracyCircle=L.circle(latlng,{radius:myLocation.accuracy,color:'#2563eb',weight:1,fillColor:'#60a5fa',fillOpacity:.13}).addTo(map);
    else myAccuracyCircle.setLatLng(latlng).setRadius(myLocation.accuracy);
  }
  if(center || (!mapCenteredOnMe && !series().some(p=>Math.abs(p.lat)>1e-6&&Math.abs(p.lon)>1e-6))){
    map.setView(latlng,16); mapCenteredOnMe=true;
  }
}
function initMobileMap(){
  if(mobileMap||!window.L)return;
  mobileMap=L.map('mobileLeaflet',{zoomControl:true,attributionControl:false,scrollWheelZoom:true,touchZoom:true,doubleClickZoom:true,boxZoom:true,keyboard:true}).setView([36.7783,-119.4179],15);
  L.tileLayer('https://a.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png',{maxZoom:19}).addTo(mobileMap);
  mobilePathLine=L.polyline([], {color:'#64748b',weight:5,opacity:.3}).addTo(mobileMap);
  mobileActiveLine=L.polyline([], {color:'#0284c7',weight:5,opacity:.95}).addTo(mobileMap);
}
function phoneHeadingFromEvent(e){
  if(Number.isFinite(e.webkitCompassHeading))return e.webkitCompassHeading;
  if(Number.isFinite(e.alpha))return (360-e.alpha+360)%360;
  return null;
}
function handleMobileOrientation(e){
  if(e.type==='deviceorientationabsolute'||e.absolute===true) hasAbsoluteOrientation=true;
  else if(hasAbsoluteOrientation&&!Number.isFinite(e.webkitCompassHeading)) return;
  let heading=phoneHeadingFromEvent(e);
  if(heading!==null) mobileHeading=heading;
  
  if(Number.isFinite(e.beta) && Number.isFinite(e.gamma)){
    let angle = screen.orientation ? screen.orientation.angle : (window.orientation || 0);
    if(angle === 90) mobilePitch = e.gamma;
    else if(angle === -90 || angle === 270) mobilePitch = -e.gamma;
    else if(angle === 180) mobilePitch = e.beta - 90;
    else mobilePitch = 90 - e.beta;
  }
  
  mobileOrientationReady=true;
  requestMobile(false);
}
async function startMobileSensors(){
  requestMyLocation();
  if(window.DeviceOrientationEvent&&typeof DeviceOrientationEvent.requestPermission==='function'){
    try{
      let result=await DeviceOrientationEvent.requestPermission();
      if(result!=='granted') substatusEl.textContent='Phone orientation permission was denied';
    }catch(err){substatusEl.textContent='Orientation permission failed: '+err.message}
  }
  if(!mobileOrientationReady){
    addEventListener('deviceorientation',handleMobileOrientation,true);
    addEventListener('deviceorientationabsolute',handleMobileOrientation,true);
    mobileOrientationReady=true;
  }
  try{
    if(!mobileCameraStream){
      mobileCameraStream=await navigator.mediaDevices.getUserMedia({video:{facingMode:{ideal:'environment'}},audio:false});
      mobileVideo.srcObject=mobileCameraStream;
    }
    mobileStartBtn.textContent='Camera On';
    substatusEl.textContent='Camera, GPS, and heading active';
  }catch(err){
    mobileStartBtn.textContent='Start Camera';
    substatusEl.textContent='Camera failed: '+err.message;
  }
  requestMobile(true);
}
let camDrag = {active: false, x: 0, y: 0, headingOff: 0};
mobileCameraPane.addEventListener('mousedown', e => { camDrag.active=true; camDrag.x=e.clientX; camDrag.y=e.clientY; });
addEventListener('mousemove', e => {
  if(!camDrag.active||activeTab!=='mobile')return;
  camDrag.headingOff += (e.clientX-camDrag.x)*0.2;
  camDrag.x=e.clientX; camDrag.y=e.clientY;
  requestMobile(true);
});
addEventListener('mouseup', () => camDrag.active=false);

mobileCameraPane.addEventListener('touchstart', e => {
  if(e.touches.length===1){ camDrag.active=true; camDrag.x=e.touches[0].clientX; camDrag.y=e.touches[0].clientY; }
}, {passive:true});
mobileCameraPane.addEventListener('touchmove', e => {
  if(!camDrag.active||activeTab!=='mobile')return;
  let dx = e.touches[0].clientX - camDrag.x;
  let dy = e.touches[0].clientY - camDrag.y;
  if(Math.abs(dx) > Math.abs(dy) * 1.2 && Math.abs(dx) > 2){
      e.preventDefault();
      camDrag.headingOff += dx * 0.2;
  }
  camDrag.x=e.touches[0].clientX; 
  camDrag.y=e.touches[0].clientY;
  requestMobile(true);
}, {passive:false});
mobileCameraPane.addEventListener('touchend', () => camDrag.active=false);

function normalizeAngle(a){return ((a%360)+360)%360}
function angleDelta(a,b){return ((a-b+540)%360)-180}
function formatDistanceM(m){
  if(!Number.isFinite(m))return '--';
  return m>=1000?`${(m/1000).toFixed(2)} km`:`${Math.round(m)} m`;
}
function rocketIndicatorScale(m){
  if(!Number.isFinite(m))return 1;
  if(m<=50)return 1.2;
  if(m>=3000)return .35;
  let t=(Math.log10(m)-Math.log10(50))/(Math.log10(3000)-Math.log10(50));
  return Math.max(.35,Math.min(1.2,1.2-t*.85));
}
function rocketPointForMobile(){
  let pts=repairLatLonTeleports(pathSeries()).filter(validLatLon);
  return pts.at(-1)||validLatLon(latestLive)&&latestLive||null;
}
function requestMobile(force=false){let now=performance.now(); if(!force&&now-lastMobile<ACTIVE_RENDER_MS)return; lastMobile=now; requestAnimationFrame(()=>drawMobile(force))}
function drawMobile(force=false){
  if(activeTab!=='mobile')return;
  initMobileMap();
  let rawPts=repairLatLonTeleports(pathSeries());
  let pts=rawPts.filter(validLatLon);
  let rocket=rocketPointForMobile();
  let me=myLocation?{lat:myLocation.lat,lon:myLocation.lon}:null;
  mobileLocationText.innerHTML=`Rocket: ${rocket?`${rocket.lat.toFixed(6)}, ${rocket.lon.toFixed(6)}`:'--, --'}<br>Phone: ${me?`${me.lat.toFixed(6)}, ${me.lon.toFixed(6)}`:'--, --'}`;

  if(mobileMap){
    mobileMap.invalidateSize();
    
    mobileGapLines.forEach(l=>l.remove()); mobileGapLines=[];
    let segments = mapLatLngSegments(rawPts);
    
    mobilePathLine.setLatLngs(segments);
    mobileActiveLine.setLatLngs(segments);
    
    for(let i=0;i<rawPts.length;i++){
      let p=rawPts[i];
      if(shouldDrawDisconnectGap(rawPts,i)&&validLatLon(p)){
        let prevValid=previousValidTelemetryPoint(rawPts,i);
        if(prevValid){
          let gapLine=L.polyline([[prevValid.lat,prevValid.lon],[p.lat,p.lon]],{color:DISCONNECT_MARKER_COLOR,weight:3,opacity:.9,dashArray:'5, 10'}).addTo(mobileMap);
          mobileGapLines.push(gapLine);
        }
      }
    }
    if(rocket){
      let latlng=[rocket.lat,rocket.lon];
      if(!mobileRocketMarker) mobileRocketMarker=L.marker(latlng,{icon:rocketMapIcon(),title:'Rocket'}).addTo(mobileMap); else mobileRocketMarker.setLatLng(latlng);
    }
    if(me){
      let latlng=[me.lat,me.lon];
      let icon=L.divIcon({className:'', html:'<div style="width:18px;height:18px;border-radius:50%;background:#2563eb;border:3px solid white;box-shadow:0 2px 8px #0008"></div>', iconSize:[24,24], iconAnchor:[12,12]});
      if(!mobileMeMarker) mobileMeMarker=L.marker(latlng,{icon,title:'Phone'}).addTo(mobileMap); else mobileMeMarker.setLatLng(latlng);
    }
    if(force){
      let bounds=[];
      if(rocket)bounds.push([rocket.lat,rocket.lon]);
      if(me)bounds.push([me.lat,me.lon]);
      if(bounds.length>1)mobileMap.fitBounds(bounds,{padding:[36,36],maxZoom:17});
      else if(bounds.length)mobileMap.setView(bounds[0],16);
    }
  }

  if(!rocket||!me){
    rocketDirection.style.opacity='.45';
    rocketDirection.classList.remove('offscreen');
    rocketDirection.style.left='50%';
    rocketDirection.style.top='50%';
    cameraTurnCue.classList.remove('visible');
    cameraTurnCue.textContent='';
    mobileDistance.textContent=rocket?'Waiting for phone GPS':'No rocket GPS';
    mobileHint.textContent='Tap Start Camera and allow GPS';
    return;
  }
  let dist=gpsDistanceM(me,rocket), bearing=getBearing(me,rocket);
  let heading=Number.isFinite(mobileHeading)?mobileHeading:(Number.isFinite(myLocation?.heading)?myLocation.heading:0);
  heading += camDrag.headingOff;
  let relative=angleDelta(bearing,heading);
  let elevation=Math.atan2(rocket.alt_m||0,Math.max(1,dist))*(180/Math.PI);
  let pitch=Number.isFinite(mobilePitch)?mobilePitch:0;
  let relativePitch=elevation-pitch;
  
  if(Math.abs(elevation)<3 && Math.abs(relativePitch)<20) relativePitch=0;
  
  let scale=rocketIndicatorScale(dist);
  let offscreen=Math.abs(relative)>CAMERA_CENTER_HALF_DEG || Math.abs(relativePitch)>35;
  
  // Horizontal is inverted, Vertical is standard
  let leftPct=Math.max(14,Math.min(86,50-(relative/CAMERA_CENTER_HALF_DEG)*36));
  let topPct=Math.max(15,Math.min(85,50-(relativePitch/35)*35));
  
  rocketDirection.style.left=leftPct+'%';
  rocketDirection.style.top=topPct+'%';

  if(offscreen){
    rocketDirection.classList.add('offscreen');
    cameraTurnCue.classList.add('visible');
    cameraTurnCue.style.left=leftPct+'%';
    cameraTurnCue.style.top='calc('+topPct+'% + 52px)';
    let turnTxt=[];
    if(relative>15) turnTxt.push('Left'); // Inverted X
    else if(relative<-15) turnTxt.push('Right'); // Inverted X
    if(relativePitch>15) turnTxt.push('Up'); // Standard Y
    else if(relativePitch<-15) turnTxt.push('Down'); // Standard Y
    cameraTurnCue.textContent=turnTxt.length?turnTxt.join('/'):'';
  }else{
    rocketDirection.classList.remove('offscreen');
    cameraTurnCue.classList.remove('visible');
    cameraTurnCue.textContent='';
  }
  rocketDirection.style.opacity='1';
  // Invert the X rotation, standard Y rotation
  let screenAngle=Math.atan2(-relative,relativePitch)*(180/Math.PI);
  rocketDirection.querySelector('i').style.transform=`rotate(${screenAngle}deg) scale(${scale.toFixed(3)})`;
  mobileDistance.textContent=`Rocket ${formatDistanceM(dist)}`;
  let debugPitch=Math.round(pitch), debugElev=Math.round(elevation), debugRel=Math.round(relativePitch);
  mobileHint.textContent=offscreen?`Tilt ${cameraTurnCue.textContent} (${Math.round(dist)}m)`:`Centered! dist ${Math.round(dist)}m alt ${Math.round(rocket.alt_m||0)}m`;
}
function request3d(force=false){let now=performance.now(); if(!force&&now-last3d<ACTIVE_RENDER_MS)return; last3d=now; requestAnimationFrame(draw3d)}
function toMeters(pts){
  let gpsOk=p=>Number.isFinite(p.lat)&&Number.isFinite(p.lon)&&Math.abs(p.lat)>1e-6&&Math.abs(p.lon)>1e-6&&Math.abs(p.lat)<=90&&Math.abs(p.lon)<=180;
  let o=pts.find(gpsOk);
  if(!o)return pts.map((p,i)=>[i*3,0,Number.isFinite(+p.alt_m)?+p.alt_m:0]);
  let ml=111320, mn=111320*Math.cos(o.lat*Math.PI/180), last=[0,0], haveLast=false;
  return pts.map((p,i)=>{
    let z=Number.isFinite(+p.alt_m)?+p.alt_m:0;
    if(gpsOk(p)){
      let next=[(p.lon-o.lon)*mn,(p.lat-o.lat)*ml];
      if(haveLast&&Math.hypot(next[0]-last[0],next[1]-last[1])>600) return [last[0],last[1],z];
      last=next; haveLast=true; return [next[0],next[1],z];
    }
    if(haveLast)return [last[0],last[1],z];
    return [i*3,0,z];
  });
}
function truncateAtGround(pts){
  let maxAlt=0;
  for(let i=0;i<pts.length;i++){
    maxAlt=Math.max(maxAlt,pts[i].alt_m||0);
    if(i>4&&maxAlt>2&&(pts[i].alt_m||0)<=0.5)return pts.slice(0,i+1);
  }
  return pts;
}
function compactLowAltitudeRuns(pts,keepSeconds=LOW_ALT_KEEP_S){
  let out=[], lowStart=null;
  for(let i=0; i<pts.length; i++){
    let p = pts[i];
    sanitizeAltitude(p);
    if((p.alt_m||0)<=LOW_ALT_MAX_M){
      if(lowStart===null)lowStart=xTime(p);
      if((xTime(p)-lowStart)/1000>keepSeconds)continue;
    }else{
      lowStart=null;
    }
    out.push(p);
  }
  return out;
}
function gpsDistanceM(a,b){
  if(!validLatLon(a)||!validLatLon(b))return Infinity;
  let ml=111320, mn=111320*Math.cos(a.lat*Math.PI/180);
  return Math.hypot((b.lat-a.lat)*ml,(b.lon-a.lon)*mn);
}
function repairLatLonTeleports(pts){
  let out=[], lastGood=null;
  for(let original of pts){
    let p={...original};
    sanitizeAltitude(p);
    if(validLatLon(p)){
      if(lastGood){
        let dt=Math.max(.2,(xTime(p)-xTime(lastGood))/1000);
        let dist=gpsDistanceM(lastGood,p);
        if(dist>250 || (dist>75 && dist/dt>35)){
          p.lat=lastGood.lat; p.lon=lastGood.lon; p.gps_corrected=true; p.raw_line=(p.raw_line||'')+' | corrected:gps_teleport';
        }else{
          lastGood=p;
        }
      }else{
        lastGood=p;
      }
    }else if(lastGood){
      p.lat=lastGood.lat; p.lon=lastGood.lon; p.gps_corrected=true; p.raw_line=(p.raw_line||'')+' | corrected:bad_gps';
    }
    out.push(p);
  }
  return out;
}
function update3dTimeline(pts, startOffset){
  if(!pts.length){view3dTimeline.style.display='none'; view3dIndex=-1; return 0}
  let max=pts.length-1;
  if(view3dFollow||view3dIndex<0||view3dIndex>max){view3dIndex=max; view3dFollow=true}
  view3dTimeline.style.display='flex';
  view3dSlider.max=String(max); view3dSlider.value=String(view3dIndex);
  let p=pts[view3dIndex];
  if(p.disconnect_gap && p.flight_state === 'connect') {
    view3dTimeLabel.textContent=`--- DISCONNECTED ---`;
    return view3dIndex;
  }
  let elapsed=(xTime(p)-xTime(pts[0]))/1000;
  view3dTimeLabel.textContent=`launch + ${elapsed.toFixed(1)} s  alt ${formatAltMeters(p.alt_m)}`;
  return view3dIndex;
}
function drawCanvasRocket(ctx){
  ctx.save();
  ctx.shadowColor='rgba(56,189,248,.85)';
  ctx.shadowBlur=10;
  ctx.fillStyle='#f8fafc';
  ctx.font='900 34px "Font Awesome 6 Free"';
  ctx.textAlign='center';
  ctx.textBaseline='middle';
  ctx.fillText(String.fromCharCode(0xf135),0,0);
  ctx.restore();
}
function rocketPathAngle3d(raw,selectedLocal,sx,sy,pts){
  if (pts && pts[selectedLocal] && typeof pts[selectedLocal].yaw === 'number') {
    let yawRad = -pts[selectedLocal].yaw * Math.PI / 180;
    let projOffset = [Math.sin(yawRad), Math.cos(yawRad), 0];
    // To properly map the yaw rotation through the camera's azimuth, 
    // we take the difference of a projected unit vector and a zero vector.
    // However, since sx/sy depend on bounding box offsets, the angle of the line 
    // is simply atan2(-projOffset_y, projOffset_x) if the screen projection is roughly 1:1.
    // A simpler approach: The 3D map's isometric azimuth rotates the map.
    // The screen angle = map_angle + camera_azimuth.
    let azimRad = cam.azim * Math.PI / 180;
    return yawRad + azimRad;
  }
  let current=raw[selectedLocal], prev=null, next=null;
  for(let i=selectedLocal-1;i>=0;i--){
    if(Math.hypot(raw[i][0]-current[0],raw[i][1]-current[1])>.01){prev=raw[i]; break}
  }
  for(let i=selectedLocal+1;i<raw.length;i++){
    if(Math.hypot(raw[i][0]-current[0],raw[i][1]-current[1])>.01){next=raw[i]; break}
  }
  let from=prev||current, to=prev?current:(next||current);
  let dx=sx(to)-sx(from), dy=sy(to)-sy(from);
  if(Math.hypot(dx,dy)<.5)return -Math.PI/2;
  return Math.atan2(dy,dx)-ROCKET_GLYPH_DEFAULT_ANGLE_RAD;
}
function gpsOrigin(pts){
  return pts.find(p=>validLatLon(p))||null;
}
function clampLat(lat){return Math.max(-85.05113,Math.min(85.05113,lat))}
function lonToTileX(lon,z){return (lon+180)/360*Math.pow(2,z)}
function latToTileY(lat,z){
  let r=clampLat(lat)*Math.PI/180;
  return (1-Math.log(Math.tan(r)+1/Math.cos(r))/Math.PI)/2*Math.pow(2,z);
}
function choose3dMapZoom(origin,hspan){
  let metersPerPixel=Math.max(.15,hspan/MAP_TEXTURE_SIZE);
  let z=Math.round(Math.log2((156543.03392*Math.cos(origin.lat*Math.PI/180))/metersPerPixel));
  return Math.max(12,Math.min(19,z));
}
function getMapTile(z,x,y){
  let n=1<<z, wrapped=((x%n)+n)%n;
  if(y<0||y>=n)return null;
  let key=`${z}/${wrapped}/${y}`, cached=mapTileCache.get(key);
  if(cached)return cached.ready?cached.img:null;
  let img=new Image();
  img.crossOrigin='anonymous';
  let entry={img,ready:false};
  mapTileCache.set(key,entry);
  img.onload=()=>{entry.ready=true; request3d(true)};
  img.onerror=()=>{entry.ready=false};
  img.src=MAP_TILE_TEMPLATE.replace('{z}',z).replace('{x}',wrapped).replace('{y}',y);
  return null;
}
function draw3dMapPlane(ctx,project,sx,sy,x0,x1,y0,y1,m,origin){
  let size=MAP_TEXTURE_SIZE;
  let pNW=project([x0,y1,0],true), pNE=project([x1,y1,0],true), pSW=project([x0,y0,0],true), pSE=project([x1,y0,0],true);
  let tex=document.createElement('canvas'), t=tex.getContext('2d');
  tex.width=size; tex.height=size;
  t.fillStyle='#172536'; t.fillRect(0,0,size,size);

  if(origin){
    let ml=111320, mn=111320*Math.cos(origin.lat*Math.PI/180);
    let west=origin.lon+x0/mn, east=origin.lon+x1/mn, south=origin.lat+y0/ml, north=origin.lat+y1/ml;
    let zoom=choose3dMapZoom(origin,Math.max(x1-x0,y1-y0));
    let minPxX=lonToTileX(west,zoom)*MAP_TILE_SIZE, maxPxX=lonToTileX(east,zoom)*MAP_TILE_SIZE;
    let minPxY=latToTileY(north,zoom)*MAP_TILE_SIZE, maxPxY=latToTileY(south,zoom)*MAP_TILE_SIZE;
    let spanX=Math.max(1,maxPxX-minPxX), spanY=Math.max(1,maxPxY-minPxY);
    let tx0=Math.floor(minPxX/MAP_TILE_SIZE), tx1=Math.floor(maxPxX/MAP_TILE_SIZE);
    let ty0=Math.floor(minPxY/MAP_TILE_SIZE), ty1=Math.floor(maxPxY/MAP_TILE_SIZE);
    for(let ty=ty0;ty<=ty1;ty++){
      for(let tx=tx0;tx<=tx1;tx++){
        let img=getMapTile(zoom,tx,ty);
        let dx=(tx*MAP_TILE_SIZE-minPxX)/spanX*size, dy=(ty*MAP_TILE_SIZE-minPxY)/spanY*size;
        let dw=MAP_TILE_SIZE/spanX*size+1, dh=MAP_TILE_SIZE/spanY*size+1;
        if(img)t.drawImage(img,dx,dy,dw,dh);
        else{
          t.fillStyle='#203247'; t.fillRect(dx,dy,dw,dh);
          t.strokeStyle='rgba(148,163,184,.2)'; t.strokeRect(dx,dy,dw,dh);
        }
      }
    }
  }else{
    t.fillStyle='#cbd5e1'; t.font='24px Segoe UI'; t.textAlign='center'; t.fillText('GPS map unavailable',size/2,size/2);
  }

  ctx.save();
  ctx.beginPath();
  ctx.moveTo(sx(pNW),sy(pNW)); ctx.lineTo(sx(pNE),sy(pNE)); ctx.lineTo(sx(pSE),sy(pSE)); ctx.lineTo(sx(pSW),sy(pSW)); ctx.closePath(); ctx.clip();
  ctx.globalAlpha=.95;
  ctx.transform((sx(pNE)-sx(pNW))/size,(sy(pNE)-sy(pNW))/size,(sx(pSW)-sx(pNW))/size,(sy(pSW)-sy(pNW))/size,sx(pNW),sy(pNW));
  ctx.drawImage(tex,0,0);
  ctx.restore();
}
function mapBoundsWithLandingClearance(m,clearanceM=500){
  let xs=m.map(v=>v[0]), ys=m.map(v=>v[1]);
  let x0=Math.min(...xs), x1=Math.max(...xs), y0=Math.min(...ys), y1=Math.max(...ys);
  let pathPadding=80;
  x0-=pathPadding; x1+=pathPadding; y0-=pathPadding; y1+=pathPadding;
  let landing=m.at(-1);
  if(landing){
    x0=Math.min(x0,landing[0]-clearanceM); x1=Math.max(x1,landing[0]+clearanceM);
    y0=Math.min(y0,landing[1]-clearanceM); y1=Math.max(y1,landing[1]+clearanceM);
  }
  let minSpan=clearanceM*2, spanX=x1-x0, spanY=y1-y0;
  if(spanX<minSpan){let c=(x0+x1)/2; x0=c-minSpan/2; x1=c+minSpan/2}
  if(spanY<minSpan){let c=(y0+y1)/2; y0=c-minSpan/2; y1=c+minSpan/2}
  return {x0,x1,y0,y1};
}
function draw3d(){
  if(activeTab!=='view3d')return;
  let rawSrc=repairLatLonTeleports(pathSeries());
  let src=rawSrc;
  let c=canvas3d,[ctx,w,h]=resizeCanvas(c);
  let sliderLocal=update3dTimeline(src,0);
  let pts=src;
  ctx.clearRect(0,0,w,h);
  let sky=ctx.createLinearGradient(0,0,0,h); sky.addColorStop(0,'#071426'); sky.addColorStop(.62,'#0f2742'); sky.addColorStop(1,'#06111f'); ctx.fillStyle=sky; ctx.fillRect(0,0,w,h);
  ctx.fillStyle='#0a1726'; ctx.beginPath(); ctx.moveTo(0,h*.78); ctx.lineTo(w,h*.66); ctx.lineTo(w,h); ctx.lineTo(0,h); ctx.fill();
  ctx.fillStyle='#94a3b8'; ctx.font='14px Segoe UI'; if(!pts.length){ctx.fillText('Waiting for flight path',18,30);return}

  let m=toMeters(pts), xs=m.map(v=>v[0]), ys=m.map(v=>v[1]), zs=m.map(v=>v[2]);
  let cx=(Math.min(...xs)+Math.max(...xs))/2, cy=(Math.min(...ys)+Math.max(...ys))/2;
  let az=cam.azim*Math.PI/180, el=cam.elev*Math.PI/180, ca=Math.cos(az), sa=Math.sin(az), ce=Math.cos(el), se=Math.sin(el);
  let footprint=Math.max(Math.max(...xs)-Math.min(...xs),Math.max(...ys)-Math.min(...ys),1);
  let peakAlt=Math.max(...zs,1), zspan=Math.max(Math.max(...zs)-Math.min(...zs),1);
  let hspan=Math.max(footprint*1.35, 120);
  let zg=1.0;
  
  let colorMetric = document.getElementById('map3dColorSelect') ? document.getElementById('map3dColorSelect').value : 'altitude';
  let metricVals = [];
  if(colorMetric === 'altitude') metricVals = zs;
  else if(colorMetric === 'wind') metricVals = pts.map(p=>p.wind_speed_m_s||0);
  else if(colorMetric === 'speed') {
     for(let i=0;i<pts.length;i++){
       if(i===0) metricVals.push(0);
       else metricVals.push(((pts[i].alt_m||0)-(pts[i-1].alt_m||0))/((xTime(pts[i])-xTime(pts[i-1]))/1000||1));
     }
  }
  else if(colorMetric === 'accel') metricVals = pts.map(p=>p.acc_total_g||0);
  let minM = Math.min(...metricVals), maxM = Math.max(...metricVals);
  function project(v,ground=false){let x=v[0]-cx,y=v[1]-cy,z=(ground?0:v[2])*zg,rx=x*ca-y*sa,ry=x*sa+y*ca;return [rx,z*ce-ry*se,v[2]]}
  let raw=m.map(v=>project(v,false)), ground=m.map(v=>project(v,true));
  let {x0,x1,y0,y1}=mapBoundsWithLandingClearance(m,500);
  let mapCorners=[project([x0,y0,0],true),project([x1,y0,0],true),project([x1,y1,0],true),project([x0,y1,0],true)];
  let all=raw.concat(ground).concat(mapCorners), minx=Math.min(...all.map(v=>v[0])), maxx=Math.max(...all.map(v=>v[0])), miny=Math.min(...all.map(v=>v[1])), maxy=Math.max(...all.map(v=>v[1]));
  let sc=Math.min((w-120)/(maxx-minx||1),(h-120)/(maxy-miny||1))*cam.zoom, sx=p=>w/2+(p[0]-(minx+maxx)/2)*sc + cam.panX, sy=p=>h*.58-(p[1]-(miny+maxy)/2)*sc + cam.panY;
  function strokeSegment(a,b,color,width,dash=[],cap='round'){ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.lineCap=cap;ctx.beginPath();ctx.moveTo(sx(a),sy(a));ctx.lineTo(sx(b),sy(b));ctx.stroke();ctx.setLineDash([])}
  let selectedLocal=Math.max(0,Math.min(sliderLocal,pts.length-1));
  draw3dMapPlane(ctx,project,sx,sy,x0,x1,y0,y1,m,gpsOrigin(src));

  // Draw flag at the origin
  let origin = raw[0];
  if(origin) {
    ctx.save();
    ctx.translate(sx(origin), sy(origin));
    ctx.beginPath();
    ctx.moveTo(0,0); ctx.lineTo(0,-24);
    ctx.strokeStyle='#f8fafc'; ctx.lineWidth=2; ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0,-24); ctx.lineTo(12,-18); ctx.lineTo(0,-12);
    ctx.fillStyle='#ef4444'; ctx.fill();
    ctx.restore();
  }

  for(let i=1;i<raw.length;i++){
    let t=Math.max(0,Math.min(1,(metricVals[i]-minM)/(maxM-minM||1))), hue=205-t*165, color=`hsl(${hue}, 85%, ${55+t*12}%)`;
    let dim=i>selectedLocal;
    
    if(shouldDrawDisconnectGap(pts,i) && pts[i-1]){
      strokeSegment(raw[i-1],raw[i],dim?DISCONNECT_MARKER_FADED:DISCONNECT_MARKER_COLOR,dim?2.5:3,[6,6],'butt');
    } else {
      strokeSegment(raw[i-1],raw[i],dim?'rgba(148,163,184,.32)':color,dim?2.5:6,[],'butt');
    }
    
    // Draw white wind particles every 100ft (30.48m)
    let altLevel = Math.floor((pts[i].alt_m||0)/30.48);
    let prevAltLevel = i>0 ? Math.floor((pts[i-1].alt_m||0)/30.48) : -1;
    if(altLevel > prevAltLevel && altLevel > 0){
        let wind_speed = pts[i].wind_speed_m_s || 0;
        let wind_dir = pts[i].wind_dir_deg || 0;
        if (wind_speed > 0) {
            let wx = Math.cos((wind_dir - 90) * Math.PI/180) * wind_speed * 10;
            let wy = Math.sin((wind_dir - 90) * Math.PI/180) * wind_speed * 10;
            let pEnd = project([m[i][0] + wx, m[i][1] - wy, m[i][2]], false);
            strokeSegment(raw[i], pEnd, 'rgba(255,255,255,0.9)', 3);
            ctx.fillStyle = 'white';
            ctx.beginPath();
            ctx.arc(sx(pEnd), sy(pEnd), 4, 0, Math.PI*2);
            ctx.fill();
        }
    }
  }
  let r=raw[selectedLocal], p=pts[selectedLocal];
  ctx.save();ctx.translate(sx(r),sy(r));ctx.rotate(rocketPathAngle3d(raw,selectedLocal,sx,sy,pts));drawCanvasRocket(ctx);ctx.restore();
  ctx.textAlign='left';ctx.textBaseline='alphabetic';
  let elapsed=(xTime(p)-xTime(src[0]))/1000;
  ctx.fillStyle='#e5e7eb';ctx.font='15px Segoe UI';ctx.fillText(`ALT ${formatAltMeters(p.alt_m)}   TIME ${elapsed.toFixed(1)} s   ACC ${p.acc_total_g.toFixed(2)} g   ${displayFlightState(p)}`,18,30);
  ctx.fillStyle='#94a3b8';ctx.font='12px Segoe UI';ctx.fillText(`points ${pts.length}   peak ${formatAltMeters(Math.max(...zs))}   left-click pan, right-click rotate, wheel zoom`,18,50);
  let ax=26, ay=h-70;
  ctx.lineCap='round';ctx.lineWidth=4;
  ctx.strokeStyle='#38bdf8';ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(ax+46,ay);ctx.stroke();
  ctx.strokeStyle='#22c55e';ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(ax,ay-36);ctx.stroke();
  ctx.strokeStyle='#f97316';ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(ax+24,ay-24);ctx.stroke();
  ctx.font='12px Segoe UI';ctx.fillStyle='#cbd5e1';ctx.fillText('X',ax+52,ay+4);ctx.fillStyle='#86efac';ctx.fillText('Y',ax-2,ay-43);ctx.fillStyle='#fdba74';ctx.fillText('ALT',ax+28,ay-29);

  // Draw small graph of selected metric
  let gw = 180, gh = 80, gx = w - gw - 20, gy = 20;
  ctx.fillStyle = 'rgba(15, 23, 42, 0.7)';
  ctx.beginPath(); ctx.roundRect(gx, gy, gw, gh, 8); ctx.fill();
  ctx.strokeStyle = '#334155'; ctx.lineWidth=1; ctx.stroke();
  
  ctx.fillStyle = '#cbd5e1'; ctx.font = '11px Segoe UI';
  ctx.textAlign = 'left'; ctx.fillText(colorMetric.toUpperCase() + ` (Max: ${maxM.toFixed(1)})`, gx + 8, gy + 16);
  
  if (metricVals && metricVals.length > 1) {
      ctx.beginPath();
      for(let i=0; i<metricVals.length; i++){
          let px = gx + 8 + (i/(metricVals.length-1)) * (gw - 16);
          let py = gy + gh - 8 - ((metricVals[i] - minM)/(maxM - minM || 1)) * (gh - 24);
          if(i===0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      }
      ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 2; ctx.stroke();
      
      // Draw selected dot
      if(selectedLocal >= 0 && selectedLocal < metricVals.length) {
          let px = gx + 8 + (selectedLocal/(metricVals.length-1)) * (gw - 16);
          let py = gy + gh - 8 - ((metricVals[selectedLocal] - minM)/(maxM - minM || 1)) * (gh - 24);
          ctx.beginPath(); ctx.arc(px, py, 4, 0, Math.PI*2);
          ctx.fillStyle = '#f8fafc'; ctx.fill();
          ctx.strokeStyle = '#000'; ctx.lineWidth=1; ctx.stroke();
      }
  }
}
canvas3d.addEventListener('mousedown',e=>{cam.drag=true;cam.button=e.button;cam.x=e.clientX;cam.y=e.clientY});
addEventListener('mouseup',()=>cam.drag=false); 
canvas3d.addEventListener('contextmenu',e=>e.preventDefault());
addEventListener('mousemove',e=>{
  if(!cam.drag)return; 
  if(cam.button === 0) { // Left click pans
    cam.panX += (e.clientX-cam.x);
    cam.panY += (e.clientY-cam.y);
  } else { // Right click rotates
    cam.azim+=(e.clientX-cam.x)*.45; 
    cam.elev=cam.elev-(e.clientY-cam.y)*.25; 
  }
  cam.x=e.clientX;cam.y=e.clientY; request3d(true);
});
canvas3d.addEventListener('wheel',e=>{e.preventDefault(); cam.zoom=Math.max(MIN_3D_ZOOM,Math.min(MAX_3D_ZOOM,cam.zoom*(e.deltaY>0?.86:1.16))); request3d(true)},{passive:false});
let touch3d={active:false,mode:'rotate',x:0,y:0,dist:0,zoom:1};
function touchDistance(a,b){return Math.hypot(a.clientX-b.clientX,a.clientY-b.clientY)}
canvas3d.addEventListener('touchstart',e=>{
  if(e.touches.length===1){
    touch3d={active:true,mode:'rotate',x:e.touches[0].clientX,y:e.touches[0].clientY,dist:0,zoom:cam.zoom};
  }else if(e.touches.length>=2){
    touch3d={active:true,mode:'pinch',x:0,y:0,dist:touchDistance(e.touches[0],e.touches[1]),zoom:cam.zoom};
  }
},{passive:false});
canvas3d.addEventListener('touchmove',e=>{
  if(!touch3d.active)return;
  e.preventDefault();
  if(touch3d.mode==='pinch'&&e.touches.length>=2){
    let dist=Math.max(20,touchDistance(e.touches[0],e.touches[1]));
    cam.zoom=Math.max(MIN_3D_ZOOM,Math.min(MAX_3D_ZOOM,touch3d.zoom*dist/Math.max(20,touch3d.dist)));
  }else if(touch3d.mode==='rotate'&&e.touches.length===1){
    cam.azim+=(e.touches[0].clientX-touch3d.x)*0.45;
    cam.elev-=(e.touches[0].clientY-touch3d.y)*0.25;
    touch3d.x=e.touches[0].clientX;
    touch3d.y=e.touches[0].clientY;
  }
  request3d(true);
},{passive:false});
canvas3d.addEventListener('touchend',()=>{touch3d.active=false},{passive:false});
mapSlider.addEventListener('input',e=>{mapIndex=+e.target.value; mapFollow=mapIndex>=+mapSlider.max; requestMap(mapFollow)});
view3dSlider.addEventListener('input',e=>{view3dIndex=+e.target.value; view3dFollow=view3dIndex>=+view3dSlider.max; request3d(true)});
csvSlider.addEventListener('input',e=>updateCsvReview(+e.target.value));
csvInput.addEventListener('change',async e=>{
  let f=e.target.files&&e.target.files[0];
  if(!f)return;
  try{
    substatusEl.textContent=`Importing ${f.name||'CSV'}...`;
    let text=await f.text();
    importCsv(text);
  }catch(err){
    substatusEl.textContent='CSV import failed: '+(err&&err.message?err.message:String(err));
  }finally{
    e.target.value='';
  }
});
function parseCsvLine(line){
  let out=[], cur='', quoted=false;
  for(let i=0;i<line.length;i++){
    let ch=line[i];
    if(ch==='"' && quoted && line[i+1]==='"'){cur+='"'; i++}
    else if(ch==='"'){quoted=!quoted}
    else if(ch===',' && !quoted){out.push(cur); cur=''}
    else cur+=ch;
  }
  out.push(cur);
  return out;
}
function pointFromCsvRow(r, val){
  let ax=+val(r,'acc_x_g','accx_zeroed(g)')||0, ay=+val(r,'acc_y_g','accy_zeroed(g)')||0, az=+val(r,'acc_z_g','accz_zeroed(g)')||0;
  let total=+val(r,'acc_total_g','rawtotalg','raw_total_g')||Math.hypot(ax,ay,az);
  let rawMpx1=val(r,'mpx1'), rawMpx2=val(r,'mpx2'), rawMpx3=val(r,'mpx3');
  let mpx1=rawMpx1===''?null:(+rawMpx1||0);
  let mpx2=rawMpx2===''?null:(+rawMpx2||0);
  let mpx3=rawMpx3===''?null:(+rawMpx3||0);
  
  let windSpeed = +val(r,'wind_speed_m_s')||0, windDir = +val(r,'wind_dir_deg')||0;
  if(windSpeed === 0 && windDir === 0 && (mpx1 !== 512 || mpx2 !== 512 || mpx3 !== 512) && (mpx1 > 0 || mpx2 > 0 || mpx3 > 0)) {
    let rho = 1.225;
    let v_out1 = (mpx1 / 1024.0) * 5.0; let pY = (((v_out1 / 5.0) - 0.5) / 0.2) * 1000.0;
    let v_out2 = (mpx2 / 1024.0) * 5.0; let pX = (((v_out2 / 5.0) - 0.5) / 0.2) * 1000.0;
    
    let vy = Math.sign(pY)*Math.sqrt((2.0 * Math.abs(pY)) / rho);
    let vx = Math.sign(pX)*Math.sqrt((2.0 * Math.abs(pX)) / rho);
    
    windSpeed = Math.sqrt(vx*vx + vy*vy);
    windDir = (Math.atan2(vy, vx) * 180.0 / Math.PI + 360.0) % 360.0;
  }
  
  return sanitizeAltitude({
    received_at:+val(r,'received_at')||Date.now()/1000,
    time_ms:+val(r,'time_ms','time(ms)')||0,
    lat:+val(r,'lat','latitude')||0,
    lon:+val(r,'lon','longitude')||0,
    alt_m:+val(r,'alt_m','altitude_m','altitude','alt_relative(m)')||0,
    sats:+val(r,'sats','satellites')||0,
    temp_c:+val(r,'temp_c','temp(c)')||0,
    pressure_hpa:+val(r,'pressure_hpa','pressure','pressure(hpa)','press(hpa)','baro_hpa')||0,
    acc_x_g:ax,
    acc_y_g:ay,
    acc_z_g:az,
    acc_total_g:total,
    rssi:+val(r,'rssi')||null,
    flight_state:val(r,'flight_state','flightstate'),
    payload_recording:['1','true','yes','recording'].includes(String(val(r,'payload_recording','recording')).toLowerCase()),
    raw_total_g:+val(r,'raw_total_g','rawtotalg')||total,
    sd_frames:+val(r,'sd_frames','records_written')||0,
    sd_free_kb:+val(r,'sd_free_kb')||0,
    gyro_x_dps: +val(r, 'gyrox(dps)', 'gyro_x_dps', 'gyrox') || 0,
    gyro_y_dps: +val(r, 'gyroy(dps)', 'gyro_y_dps', 'gyroy') || 0,
    gyro_z_dps: +val(r, 'gyroz(dps)', 'gyro_z_dps', 'gyroz') || 0,
    high_acc_x_g:+val(r,'high_acc_x_g','high_accel_x_g')||0,
    high_acc_y_g:+val(r,'high_acc_y_g','high_accel_y_g')||0,
    high_acc_z_g:+val(r,'high_acc_z_g','high_accel_z_g')||0,
    pitch:+val(r,'pitch','pitch_deg')||0,
    roll:+val(r,'roll','roll_deg')||0,
    mpx1: mpx1, mpx2: mpx2, mpx3: mpx3, wind_speed_m_s: windSpeed, wind_dir_deg: windDir,
    rocket_id:val(r,'rocket_id','rocketid')||'Unknown',
    rocket_name:val(r,'rocket_name','rocketname')||'Imported',
    raw_line:val(r,'raw_line')
  });
}
function normalizeImportedTimeline(pts){
  if(!pts.length)return [];
  let offset=0;
  let lastPlot=null;
  let lastRaw=null;
  pts.forEach(p=>{
    let raw=Number.isFinite(p.time_ms)?p.time_ms:0;
    if(lastRaw!==null){
      let diff=raw-lastRaw;
      if(diff<0){
        // reboot / backward jump
        offset += (-diff + 100);
      }else if(diff>5000){
        // huge gap, compress to 1000ms
        offset -= (diff - 1000);
      }
    }
    lastRaw=raw;
    p.plot_time_ms=raw+offset;
    lastPlot=p.plot_time_ms;
  });
  return pts;
}
function trimImportedZeroAltitudeTail(pts){
  let trimmed=truncateAtGround(pts);
  if(trimmed.length<pts.length)return trimmed;
  let lastNonZero=-1;
  for(let i=0;i<pts.length;i++){
    if((pts[i].alt_m||0)>0.5) lastNonZero=i;
  }
  if(lastNonZero>=0 && lastNonZero<pts.length-1) return pts.slice(0,lastNonZero+1);
  return pts;
}
function importCsv(text){let rows=String(text||'').trim().split(/\r?\n/).filter(Boolean).map(parseCsvLine); if(!rows.length){substatusEl.textContent='CSV import failed: empty file';return} let head=rows[0].map(h=>h.replace(/[^a-z0-9_]/gi,'').toLowerCase()), body=rows.slice(1), idx=n=>head.indexOf(n.replace(/[^a-z0-9_]/gi,'').toLowerCase());
  function val(r,...names){for(let n of names){let i=idx(n); if(i>=0&&r[i]!==undefined&&r[i]!=='')return r[i]} return ''}
  let importedRaw = body.map(r=>pointFromCsvRow(r,val)).filter(p=>p.flight_state !== 'connect' && Number.isFinite(p.time_ms) && p.time_ms > 0);
  if(!importedRaw.length){substatusEl.textContent='CSV import failed: no telemetry rows found'; return}
  let imported=importedRaw;
  mapIndex=-1; mapFollow=true; view3dIndex=-1; view3dFollow=true; localAltZero=null; lastPacketWall=null; pendingDisconnectStart=null; lastUiPacketAt=0;
  viewingImport=true; importing=true; all=imported; flight=imported; recorded=imported; telemetryQueue=[]; currentRoll=0; currentPitch=0; currentYaw=0; lastGyroTimeMs=null; launched=imported.some(activePoint); landed=imported.some(p=>['LANDED','STOPPED','RECOVERY'].includes((p.flight_state||'').toUpperCase())); recording=false; csvExported=true; stopCommandSent=false; graphBase=imported[0]?.time_ms??null; importing=false;
  packetCount=imported.length; if(imported.length) updateCsvReview(imported.length-1); else {csvReview.style.display='none'; latest.textContent='Imported CSV had no telemetry rows'}
  substatusEl.textContent=`Imported CSV, ${imported.length} row${imported.length===1?'':'s'}`;
  showTab('graphs'); requestGraph(true); requestMap(true); request3d(true); if(typeof requestWind==='function') requestWind(true); requestMobile(true);
  let hasWindData = imported.some(d =>
      d.wind_speed_m_s > 0 ||
      [d.mpx1,d.mpx2,d.mpx3].some(v => Number.isFinite(v) && v > 0 && Math.abs(v - 512) > 10));
  let windButton = document.getElementById('btnViewWind');
  if(windButton) windButton.style.display = hasWindData ? 'inline-block' : 'none';
}

  // --- WIND MAP LOGIC ---
  let windCanvas = document.getElementById('canvasWind');
  let windSlider = document.getElementById('windSlider');
  let windTimeLabel = document.getElementById('windTimeLabel');
  let windTimeline = document.getElementById('windTimeline');
  let windAutoFit = true;
  let windUserScale = 1.0;
  let windCenterX = 0, windCenterY = 0;
  let isWindDragging = false;
  let lastWindMouseX = 0, lastWindMouseY = 0;

  if (windCanvas) {
      windCanvas.addEventListener('mousedown', e => { isWindDragging = true; lastWindMouseX = e.clientX; lastWindMouseY = e.clientY; });
      windCanvas.addEventListener('mousemove', e => {
          if (isWindDragging) {
              let dx = e.clientX - lastWindMouseX;
              let dy = e.clientY - lastWindMouseY;
              lastWindMouseX = e.clientX;
              lastWindMouseY = e.clientY;
              windCenterX -= dx / windUserScale;
              windCenterY += dy / windUserScale;
              windAutoFit = false;
              requestWind(true);
          }
      });
      windCanvas.addEventListener('mouseup', () => isWindDragging = false);
      windCanvas.addEventListener('mouseleave', () => isWindDragging = false);
      windCanvas.addEventListener('wheel', e => {
          e.preventDefault();
          windUserScale *= e.deltaY < 0 ? 1.15 : 0.87;
          windAutoFit = false;
          requestWind(true);
      });
      windCanvas.addEventListener('dblclick', () => { windAutoFit = true; requestWind(true); });
      windSlider.addEventListener('input', () => requestWind(true));
  }

  function requestWind(force=false){
      if (!windCanvas) return;
      let now = performance.now();
      if (!force && now - lastWind < ACTIVE_RENDER_MS) return;
      lastWind = now;
      requestAnimationFrame(drawWind);
  }

  function drawWind() {
      if (activeTab !== 'viewWind' || !windCanvas) return;
      let dpr = devicePixelRatio || 1;
      let rect = windCanvas.getBoundingClientRect();
      let w = rect.width, h = rect.height;
      if (windCanvas.width !== w*dpr || windCanvas.height !== h*dpr) {
          windCanvas.width = w * dpr;
          windCanvas.height = h * dpr;
      }
      let ctx = windCanvas.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#0f172a';
      ctx.fillRect(0, 0, w, h);

      let pts = graphSeries();
      if (!pts.length) {
          ctx.fillStyle = '#64748b';
          ctx.font = '16px sans-serif';
          ctx.textAlign = 'center';
          ctx.fillText('No data — import a CSV with wind sensor columns', w/2, h/2);
          return;
      }

      let maxIdx = pts.length - 1;
      if (viewingImport) {
          windTimeline.style.display = 'flex';
          windSlider.max = maxIdx;
          if (+windSlider.value > maxIdx) windSlider.value = maxIdx;
      } else {
          windTimeline.style.display = 'none';
          windSlider.value = maxIdx;
      }

      let parsedIndex = Number.parseInt(windSlider.value, 10);
      let currentIdx = Number.isFinite(parsedIndex) ? parsedIndex : maxIdx;
      let pCurrent = pts[Math.min(currentIdx, maxIdx)];

      if (viewingImport && windTimeLabel) {
          windTimeLabel.textContent = formatGraphTime((pCurrent.time_ms - pts[0].time_ms)/1000.0) + ' / ' + formatGraphTime((pts[maxIdx].time_ms - pts[0].time_ms)/1000.0);
      }

      let launchLat = pts[0].lat;
      let launchLon = pts[0].lon;

      // Build path in meters from launch
      let path = [];
      for (let i = 0; i < pts.length; i++) {
          let p = pts[i];
          let dy = (p.lat - launchLat) * 111320;
          let dx = (p.lon - launchLon) * 111320 * Math.cos(launchLat * Math.PI / 180);
          path.push({x: dx, y: dy, alt: p.alt_m||0, windSpeed: p.wind_speed_m_s||0, windDir: p.wind_dir_deg||0, t: p.time_ms});
      }

      // Compute bounds
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      for (let p of path) {
          if (p.x < minX) minX = p.x;
          if (p.x > maxX) maxX = p.x;
          if (p.y < minY) minY = p.y;
          if (p.y > maxY) maxY = p.y;
      }
      let rangeX = maxX - minX || 1;
      let rangeY = maxY - minY || 1;
      let midX = (minX + maxX) / 2;
      let midY = (minY + maxY) / 2;

      // Auto-fit: compute scale to fit path in 80% of canvas
      let fitScale;
      if (windAutoFit) {
          let sx = (w * 0.8) / rangeX;
          let sy = (h * 0.8) / rangeY;
          fitScale = Math.min(sx, sy);
          if (!isFinite(fitScale) || fitScale <= 0) fitScale = 2;
          windUserScale = fitScale;
          windCenterX = midX;
          windCenterY = midY;
      }
      let scale = windUserScale;

      // Transform: canvas center = (windCenterX, windCenterY) in meters
      function toScreen(mx, my) {
          return [(mx - windCenterX) * scale + w/2, -(my - windCenterY) * scale + h/2];
      }

      // Draw grid (every 50m or 100m depending on scale)
      let gridM = 100;
      if (scale > 3) gridM = 50;
      if (scale > 8) gridM = 20;
      if (scale < 0.5) gridM = 500;
      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 0.5;
      let gMinX = Math.floor(minX / gridM - 2) * gridM;
      let gMaxX = Math.ceil(maxX / gridM + 2) * gridM;
      let gMinY = Math.floor(minY / gridM - 2) * gridM;
      let gMaxY = Math.ceil(maxY / gridM + 2) * gridM;
      for (let gx = gMinX; gx <= gMaxX; gx += gridM) {
          let [sx1, sy1] = toScreen(gx, gMinY);
          let [sx2, sy2] = toScreen(gx, gMaxY);
          ctx.beginPath(); ctx.moveTo(sx1, sy1); ctx.lineTo(sx2, sy2); ctx.stroke();
      }
      for (let gy = gMinY; gy <= gMaxY; gy += gridM) {
          let [sx1, sy1] = toScreen(gMinX, gy);
          let [sx2, sy2] = toScreen(gMaxX, gy);
          ctx.beginPath(); ctx.moveTo(sx1, sy1); ctx.lineTo(sx2, sy2); ctx.stroke();
      }

      // Grid labels
      ctx.fillStyle = '#475569';
      ctx.font = '10px monospace';
      ctx.textAlign = 'center';
      for (let gx = gMinX; gx <= gMaxX; gx += gridM) {
          let [sx, sy] = toScreen(gx, windCenterY);
          ctx.fillText(gx + 'm', sx, h - 4);
      }

      // Draw launch marker
      let [lx, ly] = toScreen(0, 0);
      ctx.fillStyle = '#ef4444';
      ctx.beginPath(); ctx.arc(lx, ly, 8, 0, Math.PI*2); ctx.fill();
      ctx.fillStyle = 'white';
      ctx.font = 'bold 11px sans-serif';
      ctx.textAlign = 'left';
      ctx.fillText('LAUNCH', lx + 12, ly + 4);

      // Draw Flight Path up to current index
      ctx.beginPath();
      ctx.strokeStyle = '#f97316';
      ctx.lineWidth = 2.5;
      for (let i = 0; i <= currentIdx && i < path.length; i++) {
          let [sx, sy] = toScreen(path[i].x, path[i].y);
          if (i === 0) ctx.moveTo(sx, sy);
          else ctx.lineTo(sx, sy);
      }
      ctx.stroke();

      // Draw Wind Particles (every 100ft / 30.48m altitude)
      let lastAltMark = -999;
      for (let i = 0; i <= currentIdx && i < path.length; i++) {
          let p = path[i];
          if (p.alt > lastAltMark + 30.48 && p.windSpeed > 0.1) {
              let [px, py] = toScreen(p.x, p.y);

              let windRad = (90 - p.windDir) * Math.PI / 180;
              let driftLen = p.windSpeed * 3;
              let [tx, ty] = toScreen(p.x + Math.cos(windRad)*driftLen, p.y + Math.sin(windRad)*driftLen);

              // Dashed line from path to particle
              ctx.beginPath();
              ctx.strokeStyle = 'rgba(255,255,255,0.25)';
              ctx.setLineDash([3, 3]);
              ctx.lineWidth = 1;
              ctx.moveTo(px, py);
              ctx.lineTo(tx, ty);
              ctx.stroke();
              ctx.setLineDash([]);

              // White particle dot
              ctx.fillStyle = 'white';
              ctx.beginPath();
              ctx.arc(tx, ty, 4, 0, Math.PI*2);
              ctx.fill();

              // Alt label
              ctx.fillStyle = '#94a3b8';
              ctx.font = '9px monospace';
              ctx.textAlign = 'left';
              let altFt = Math.round(p.alt * 3.28084);
              ctx.fillText(altFt + 'ft', tx + 6, ty + 3);

              lastAltMark = p.alt;
          }
      }

      // Draw Current Rocket Position
      let cp = path[Math.min(currentIdx, path.length-1)];
      if (cp) {
          let [cx, cy] = toScreen(cp.x, cp.y);

          // Glow
          let glow = ctx.createRadialGradient(cx, cy, 0, cx, cy, 16);
          glow.addColorStop(0, 'rgba(16,185,129,0.6)');
          glow.addColorStop(1, 'rgba(16,185,129,0)');
          ctx.fillStyle = glow;
          ctx.fillRect(cx-16, cy-16, 32, 32);

          // Dot
          ctx.fillStyle = '#10b981';
          ctx.beginPath();
          ctx.arc(cx, cy, 6, 0, Math.PI*2);
          ctx.fill();
          ctx.strokeStyle = 'white';
          ctx.lineWidth = 1.5;
          ctx.stroke();

          // Wind vector arrow
          if (cp.windSpeed > 0.1) {
              let windRad = (90 - cp.windDir) * Math.PI / 180;
              let arrowLen = cp.windSpeed * 6;
              let [ax, ay] = toScreen(cp.x + Math.cos(windRad)*arrowLen, cp.y + Math.sin(windRad)*arrowLen);

              ctx.beginPath();
              ctx.strokeStyle = '#06b6d4';
              ctx.lineWidth = 3;
              ctx.moveTo(cx, cy);
              ctx.lineTo(ax, ay);
              ctx.stroke();

              // Arrowhead
              let angle = Math.atan2(ay - cy, ax - cx);
              ctx.beginPath();
              ctx.fillStyle = '#06b6d4';
              ctx.moveTo(ax, ay);
              ctx.lineTo(ax - 10*Math.cos(angle - 0.4), ay - 10*Math.sin(angle - 0.4));
              ctx.lineTo(ax - 10*Math.cos(angle + 0.4), ay - 10*Math.sin(angle + 0.4));
              ctx.closePath();
              ctx.fill();

              // Wind speed label
              ctx.fillStyle = '#22d3ee';
              ctx.font = 'bold 12px sans-serif';
              ctx.textAlign = 'left';
              ctx.fillText(cp.windSpeed.toFixed(1) + ' m/s', ax + 8, ay - 4);
          }
      }

      // HUD overlay: altitude + info
      ctx.fillStyle = 'rgba(15,23,42,0.75)';
      ctx.fillRect(8, 8, 200, 60);
      ctx.strokeStyle = '#334155';
      ctx.lineWidth = 1;
      ctx.strokeRect(8, 8, 200, 60);
      ctx.fillStyle = '#e2e8f0';
      ctx.font = '12px monospace';
      ctx.textAlign = 'left';
      if (cp) {
          ctx.fillText('Alt: ' + formatAltMeters(cp.alt), 16, 28);
          ctx.fillText('Wind: ' + (cp.windSpeed||0).toFixed(1) + ' m/s @ ' + (cp.windDir||0).toFixed(0) + '°', 16, 44);
          ctx.fillText('Pt ' + (currentIdx+1) + ' / ' + path.length, 16, 60);
      }
  }

try{
  const savedPayloadStorage=localStorage.getItem('payloadStorageMode');
  if(['INTERNAL','SD','BOTH'].includes(savedPayloadStorage)) document.getElementById('storageSelect').value=savedPayloadStorage;
}catch{}
connect();
updateControls();
setInterval(updateConnectionHealth, 100);
setInterval(flushTelemetryQueue, UI_FLUSH_MS);


</script>

</body>
</html>"""


async def index(_: web.Request) -> web.Response:
    return web.Response(
        text=HTML,
        content_type="text/html",
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate",
            "Pragma": "no-cache",
        },
    )


async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    hub: TelemetryHub = request.app["hub"]
    ws = web.WebSocketResponse(heartbeat=15.0)
    await ws.prepare(request)
    hub.websockets.add(ws)
    await ws.send_str(json.dumps({"type": "hello", "connected": True, "total": hub.total_packets}))
    await ws.send_str(json.dumps({"type": "status", "message": hub.status_message}))
    if hub.latest and time.time() - hub.latest.received_at <= 5.0:
        await ws.send_str(json.dumps({"type": "telemetry", "point": hub.latest.to_json(), "total": hub.total_packets}))
    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                if data.get("type") == "command" and data.get("command"):
                    await hub.handle_command(str(data["command"]))
                elif data.get("type") == "bind":
                    global bound_rocket_id
                    bound_rocket_id = data.get("id")
    finally:
        hub.websockets.discard(ws)
    return ws


async def start_background(app: web.Application) -> None:
    args: argparse.Namespace = app["args"]
    hub: TelemetryHub = app["hub"]
    stop_event: threading.Event = app["stop_event"]
    loop = asyncio.get_running_loop()
    if args.simulate:
        app["reader_task"] = asyncio.create_task(simulation_task(hub))
    else:
        thread = threading.Thread(target=serial_thread, args=(loop, hub, args, stop_event), daemon=True)
        thread.start()
        app["reader_thread"] = thread


async def cleanup_background(app: web.Application) -> None:
    app["stop_event"].set()
    task = app.get("reader_task")
    if task:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    thread = app.get("reader_thread")
    if thread:
        thread.join(timeout=1.5)
    app["hub"].flush_csv_buffer()


def port_is_available(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.bind((host, port))
        except OSError:
            return False
    return True


def choose_http_port(host: str, preferred_port: int, attempts: int = 25) -> int:
    for port in range(preferred_port, preferred_port + attempts):
        if port_is_available(host, port):
            return port
    return preferred_port


def main() -> None:
    args = parse_args()
    selected_port = choose_http_port(args.host, args.http_port)
    if selected_port != args.http_port:
        print(f"Port {args.http_port} is busy. Using {selected_port} instead.")
        args.http_port = selected_port
    hub = TelemetryHub()
    app = web.Application()
    app["hub"] = hub
    app["args"] = args
    app["stop_event"] = threading.Event()
    app.router.add_get("/", index)
    app.router.add_get("/ws", websocket_handler)
    app.on_startup.append(start_background)
    app.on_cleanup.append(cleanup_background)
    url = f"http://{args.host}:{args.http_port}"
    if not args.no_open:
        def open_browser():
            try:
                if hasattr(os, 'startfile'):
                    os.startfile(url)
                else:
                    webbrowser.open(url)
            except Exception as e:
                print(f"Could not open browser: {e}")
        threading.Timer(0.8, open_browser).start()
    print(f"Fast dashboard running at {url}")
    web.run_app(app, host=args.host, port=args.http_port)


if __name__ == "__main__":
    main()
