#!/usr/bin/env python3
from __future__ import annotations

# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "rich",
# ]
# ///
from argparse import ArgumentParser
from dataclasses import dataclass
from ipaddress import IPv4Address
from ipaddress import IPv4Network
from ipaddress import ip_interface
import json
import re
import socket
import struct
import sys
from typing import Any
from urllib.parse import quote
from urllib.request import urlopen

try:
    from rich import print  # noqa: A004
    from rich.syntax import Syntax
    from rich.table import Table
except ModuleNotFoundError:
    from builtins import print  # noqa: A004

    class Syntax(str):
        def __new__(cls, code: str, lexer: str):
            return str.__new__(cls, code)

    class Table:
        def __init__(self, *headers: str, title: str | None = None, **_: Any):
            self.headers = headers
            self.title = title
            self.rows: list[tuple[str, ...]] = []

        def add_row(self, *values: str) -> None:
            self.rows.append(tuple(values))

        def __str__(self) -> str:
            lines: list[str] = []
            if self.title:
                lines.append(self.title)
            if self.headers:
                lines.append(" | ".join(self.headers))
            for row in self.rows:
                lines.append(" | ".join(row))
            return "\n".join(lines)


# Port Auto-Detection State
DETECTED_HTTP_PORT = 80
DETECTED_CONTROL_PORT = 8002

CONTROL_PORT = 8002
HTTP_PORT = 8010
HTTP_TIMEOUT_SEC = 5
CONTROL_TIMEOUT_SEC = 5


RETURN_MODE_NAME_TO_VALUE = {
    "Single": 1,
    "1": 1,
    "Dual": 2,
    "2": 2,
    "StrongestFurthest": 3,
    "3": 3,
}
RETURN_MODE_VALUE_TO_NAME = {
    1: "Single",
    2: "Dual",
    3: "StrongestFurthest",
}

REFLECTANCE_MODE_NAME_TO_VALUE = {
    "None": 0,
    "0": 0,
    "Intensity": 1,
    "1": 1,
    "Reflectivity": 2,
    "2": 2,
}
REFLECTANCE_MODE_VALUE_TO_NAME = {
    0: "None",
    1: "Intensity",
    2: "Reflectivity",
}

SYNC_MODE_NAME_TO_VALUE = {
    "Host": 0,
    "0": 0,
    "PTP": 1,
    "1": 1,
    "GPS": 2,
    "2": 2,
    "File": 3,
    "3": 3,
    "NTP": 4,
    "4": 4,
}
SYNC_MODE_VALUE_TO_NAME = {
    0: "Host",
    1: "PTP",
    2: "GPS",
    3: "File",
    4: "NTP",
}

STATUS_FIELDS = (
    ("time_config", "time_sync"),
    ("up_time", "uptime"),
    ("err", "error_code"),
    ("stream_status", "stream_status"),
    ("stream_count", "stream_count"),
    ("data_sent", "data_sent"),
    ("count1", "idle_loop"),
    ("count2", "lose_counter_1"),
    ("count3", "lose_counter_2"),
)


class CommandBase:
    def parse(self) -> dict[str, Any]:
        raise NotImplementedError()

    def print(self, title: str) -> None:  # noqa: A003
        table = Table("Parameter", "Value", title=title, highlight=True, min_width=45)
        for key, value in self.parse().items():
            table.add_row(key, str(value))

        print(table)


@dataclass
class DeviceInfo(CommandBase):
    model: str
    serial_number: str
    fw_version: str

    def parse(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "serial_number": self.serial_number,
            "fw_version": self.fw_version,
        }


@dataclass
class ConfigInfo(CommandBase):
    sensor_ip: str
    mask: str
    gateway: str
    destination_ip: str
    data_port: int
    message_port: int
    status_port: int
    return_mode: str
    reflectance_mode: str
    time_sync: str
    frame_rate: float | str
    horizontal_roi: float | str
    vertical_roi: float | str
    udp_field_count: int = 4

    def parse(self) -> dict[str, Any]:
        frame_rate = self.frame_rate
        if isinstance(frame_rate, float):
            frame_rate = f"{frame_rate:.5f} Hz"

        horizontal_roi = self.horizontal_roi
        if isinstance(horizontal_roi, float):
            horizontal_roi = f"{horizontal_roi:.6f} deg"

        vertical_roi = self.vertical_roi
        if isinstance(vertical_roi, float):
            vertical_roi = f"{vertical_roi:.6f} deg"

        return {
            "sensor_ip": self.sensor_ip,
            "mask": self.mask,
            "gateway": self.gateway,
            "destination_ip": self.destination_ip,
            "data_port": self.data_port,
            "message_port": self.message_port,
            "status_port": self.status_port,
            "return_mode": self.return_mode,
            "reflectance_mode": self.reflectance_mode,
            "time_sync": self.time_sync,
            "frame_rate": frame_rate,
            "horizontal_roi": horizontal_roi,
            "vertical_roi": vertical_roi,
        }


@dataclass
class StatusInfo(CommandBase):
    status: dict[str, str]

    def parse(self) -> dict[str, Any]:
        return self.status


def download_binary_file(sensor_ip: str, cmd: str, filename: str) -> bool:
    ports = [DETECTED_CONTROL_PORT, 8002, 8010]
    ports = list(dict.fromkeys(ports))

    for port in ports:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(10)
                s.connect((sensor_ip, port))
                s.sendall(cmd.encode("utf-8") + b"\n")

                # Protocol: 4 bytes big-endian length
                header = s.recv(4)
                if len(header) < 4:
                    continue

                length = struct.unpack(">I", header)[0]
                if length == 0 or length > 20 * 1024 * 1024:  # 20MB limit
                    continue

                print(
                    f"Downloading {filename} ({length} bytes) via Port {port}...",
                    end=" ",
                    flush=True,
                )
                data = bytearray()
                while len(data) < length:
                    chunk = s.recv(min(length - len(data), 65536))
                    if not chunk:
                        break
                    data.extend(chunk)

                if len(data) < length:
                    print("Incomplete.")
                    continue

                # Optional: Read MD5 (32 bytes)
                try:
                    s.recv(32, socket.MSG_DONTWAIT)
                except Exception:
                    pass

                with open(filename, "wb") as f:
                    f.write(data)

                print("Done.")
                return True
        except Exception:
            continue
    return False


def raw_command(sensor_ip: str, cmd: str) -> str:
    global DETECTED_CONTROL_PORT
    ports_to_try = [DETECTED_CONTROL_PORT, 8002, 8010]
    ports_to_try = list(dict.fromkeys(ports_to_try))

    for port in ports_to_try:
        try:
            # For 8010/80, try HTTP first as it's cleaner for modern sensors
            if port in [80, 8010] and not cmd.startswith("set_network"):
                try:
                    return http_get(sensor_ip, f"/command/?{cmd}", port=port, as_bytes=False)
                except Exception:
                    pass

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(HTTP_TIMEOUT_SEC)
                s.connect((sensor_ip, port))
                s.sendall(cmd.encode("utf-8") + b"\n")

                reply = bytearray()
                while True:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    reply.extend(chunk)
                    if b"\n\n" in reply:
                        break

                if reply:
                    DETECTED_CONTROL_PORT = port
                    resp = reply.decode("utf-8", errors="ignore").strip()
                    return resp
        except Exception as e:
            _ = e  # suppress F841
            continue

    return ""


def http_get(
    sensor_ip: str, path_and_query: str, port: int | None = None, as_bytes: bool = False
) -> str | bytes:
    global DETECTED_HTTP_PORT
    if port:
        ports_to_try = [port]
    else:
        ports_to_try = [DETECTED_HTTP_PORT, 80, 8010]
        # Remove duplicates but keep order
        ports_to_try = list(dict.fromkeys(ports_to_try))

    for port in ports_to_try:
        url = f"http://{sensor_ip}:{port}{path_and_query}"
        try:
            with urlopen(url, timeout=HTTP_TIMEOUT_SEC) as response:
                data = response.read()

                # Filter out generic HTML (e.g., from port 80 index)
                if not as_bytes:
                    try:
                        text = data.decode("utf-8")
                    except UnicodeDecodeError:
                        text = data.decode("latin1")

                    if "<html" in text.lower() and "model:" not in text and "sn:" not in text:
                        continue

                    DETECTED_HTTP_PORT = port
                    return text
                else:
                    DETECTED_HTTP_PORT = port
                    return data
        except Exception as e:
            _ = e  # suppress F841
            continue

    if as_bytes:
        return b""
    return ""


def get_http_attribute(sensor_ip: str, name: str) -> str:
    return http_get(sensor_ip, f"/command/?{quote(name)}")


def trigger_streaming(sensor_ip: str, data_port: int = 2371) -> bool:
    print(f"Triggering active stream (port {data_port})...", end=" ", flush=True)
    try:
        # Try both TCP and pseudo-HTTP triggers found in SDK
        raw_command(sensor_ip, "start")
        raw_command(sensor_ip, f"start direct {data_port}")
        raw_command(sensor_ip, f"set_attribute udp_raw_port={data_port}")
        raw_command(sensor_ip, "GET /start/ HTTP/1.0\r\n\r\n")
        print("Done")
        return True
    except Exception as e:
        print(f"Skipped ({e})")
        return False


def set_http_attribute(sensor_ip: str, name: str, value: str) -> None:
    http_get(sensor_ip, f"/command/?set_{quote(name)}={quote(value)}")


def yesno(question: str) -> bool:
    prompt = f"{question} [y/n]: "
    answer = input(prompt).strip().lower()
    if answer not in ("y", "n"):
        print(f"{answer} is invalid, please try again...")
        return yesno(question)

    return answer == "y"


def is_valid_ip(ip: str | None) -> bool:
    if ip is None:
        return True

    try:
        IPv4Address(ip)
    except ValueError:
        return False
    else:
        return True


def is_valid_netmask(mask: str) -> bool:
    mask = mask.replace("/", "")

    try:
        IPv4Network("0.0.0.0/" + mask)
    except ValueError:
        return False
    else:
        return True


def normalize_netmask(mask: str) -> str:
    mask = mask.replace("/", "")
    return str(IPv4Network("0.0.0.0/" + mask).netmask)


def is_same_subnet(destination_ip: str, sensor_ip: str, mask: str) -> bool:
    if destination_ip == "255.255.255.255":
        return True

    return (
        ip_interface(f"{destination_ip}/{mask}").network
        == ip_interface(f"{sensor_ip}/{mask}").network
    )


def parse_network(raw_network: str, sensor_ip: str) -> tuple[str, str, str]:
    if not raw_network:
        return sensor_ip, "255.255.255.0", "0.0.0.0"

    # Handle JSON format (modern sensors like E1X)
    try:
        data = json.loads(raw_network)
        if "response" in data and "IPv4" in data["response"]:
            ipv4 = data["response"]["IPv4"]
            return (
                ipv4.get("IP", sensor_ip),
                ipv4.get("NETWORK", "255.255.255.0"),
                ipv4.get("GATEWAY", "0.0.0.0"),
            )
    except Exception:
        pass

    # Handle Key=Value space-separated (HTTP 8010 format)
    if "IP=" in raw_network:
        parts = {p.split("=")[0]: p.split("=")[1] for p in raw_network.split() if "=" in p}
        return (
            parts.get("IP", sensor_ip),
            parts.get("netmask", "255.255.255.0"),
            parts.get("gateway", "0.0.0.0"),
        )

    # Handle INI format
    ip = sensor_ip
    mask = "255.255.255.0"
    gateway = "0.0.0.0"
    for line in raw_network.splitlines():
        if "IP =" in line:
            ip = clean_value(line, "IP =")
        if "mask =" in line:
            mask = clean_value(line, "mask =")
        if "gateway =" in line:
            gateway = clean_value(line, "gateway =")
    return ip, mask, gateway


def parse_udp_ports_ip(reply: str) -> tuple[int, int, int, str, str, str, int]:
    # Formats:
    # 1. JSON (Robin E1X 8002)
    # 2. Key=Value
    # 3. Space/Comma separated (Legacy/PCS)

    if "{" in reply:
        try:
            data = json.loads(reply)
            if "response" in data:
                r = data["response"]
                return 0, 0, 0, r.get("IP", ""), "", "", 1
        except Exception:
            pass

    fields = reply.replace(",", " ").split()
    try:
        if len(fields) >= 6:
            return (
                int(fields[0]),
                int(fields[1]),
                int(fields[2]),
                fields[3],
                fields[4],
                fields[5],
                len(fields),
            )
        if len(fields) >= 4:
            return (
                int(fields[0]),
                int(fields[1]),
                int(fields[2]),
                fields[3],
                "0.0.0.0",
                "0.0.0.0",
                len(fields),
            )
    except (ValueError, IndexError):
        pass

    return 0, 0, 0, "", "", "", 0


def parse_roi(reply: str) -> tuple[float, float]:
    horizontal_roi, vertical_roi = reply.split(",", maxsplit=1)
    return float(horizontal_roi), float(vertical_roi)


def parse_i_config_value(reply: str, section: str, key: str) -> str:
    # Try JSON first (modern sensors like Robin E1X)
    try:
        data = json.loads(reply)
        if "response" in data and key in data["response"]:
            return str(data["response"][key])
    except (json.JSONDecodeError, KeyError, TypeError):
        pass

    # Fallback to INI pattern (older sensors)
    pattern = rf"\[{re.escape(section)}\]\s+{re.escape(key)}\s*=\s*(.+)"
    match = re.search(pattern, reply, flags=re.MULTILINE)
    if not match:
        raise ValueError(f"Unexpected {section}/{key} reply: {reply}")
    return match.group(1).strip()


def parse_status(reply: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for source_key, output_key in STATUS_FIELDS:
        match = re.search(rf"{re.escape(source_key)}=([^\s]+)", reply)
        if not match:
            continue
        value = match.group(1)
        if source_key == "time_config":
            try:
                value = SYNC_MODE_VALUE_TO_NAME.get(int(float(value)), value)
            except ValueError:
                pass
        elif source_key == "up_time":
            try:
                value = f"{int(float(value))} s"
            except ValueError:
                pass
        values[output_key] = value

    if not values:
        values["raw"] = reply
    return values


def parse_mode_status(reply: str) -> dict[str, str]:
    # Format: 3,2,2,379 (Mode, PreMode, Status, TransitionMs)
    fields = reply.split(",")
    if len(fields) >= 3:
        try:
            mode = int(fields[0])
            status = int(fields[2])

            # Mapping from SDK
            mode_name = {
                1: "Sleep",
                2: "Standby",
                3: "WorkNormal",
                4: "ShortRange",
                5: "Calibration",
                6: "Protection",
            }.get(mode, f"Mode({mode})")

            status_name = {0: "None", 1: "Transition", 2: "Normal", 3: "Failed"}.get(
                status, f"Status({status})"
            )

            # Map to standard numeric status for compatibility with main script logic
            # 1 = Streaming/Normal, 0 = Idle/Standby
            std_status = "1" if status == 2 else "0"

            return {
                "mode": mode_name,
                "stream_status": std_status,
                "status_text": status_name,
                "transition_ms": fields[3] if len(fields) > 3 else "0",
            }
        except (ValueError, IndexError):
            pass
    return {}


def clean_value(value: str, prefix: str) -> str:
    if value.lower().startswith(prefix.lower()):
        return value[len(prefix) :].strip()
    return value


def parse_version(raw_version: str) -> str:
    for line in raw_version.splitlines():
        if "App Version:" in line:
            return clean_value(line, "App Version:")
        if "Firmware Version:" in line:
            return clean_value(line, "Firmware Version:")
    return raw_version


def read_device_info(sensor_ip: str) -> DeviceInfo:
    # Prefer HTTP for basic info as it's more stable across firmware versions
    try:
        model_raw = get_http_attribute(sensor_ip, "model")
        sn_raw = get_http_attribute(sensor_ip, "sn")
        version_raw = get_http_attribute(sensor_ip, "fw_version")
    except Exception:
        # Fallback to raw TCP if HTTP fails
        model_raw = raw_command(sensor_ip, "get_model")
        sn_raw = raw_command(sensor_ip, "get_sn")
        version_raw = raw_command(sensor_ip, "get_version")

    return DeviceInfo(
        model=clean_value(model_raw, "model:"),
        serial_number=clean_value(sn_raw, "serial number:"),
        fw_version=parse_version(version_raw),
    )


def read_config_info(sensor_ip: str) -> ConfigInfo:
    current_sensor_ip, mask, gateway = parse_network(
        raw_command(sensor_ip, "get_network"), sensor_ip
    )
    raw_udp_reply = get_http_attribute(sensor_ip, "udp_ports_ip")
    data_port, message_port, status_port, destination_ip, dest_ip2, source_ip, field_count = (
        parse_udp_ports_ip(raw_udp_reply)
    )
    horizontal_roi, vertical_roi = parse_roi(get_http_attribute(sensor_ip, "roi"))
    reflectance_mode_raw = get_http_attribute(sensor_ip, "reflectance_mode")
    reflectance_mode = REFLECTANCE_MODE_VALUE_TO_NAME.get(
        int(float(reflectance_mode_raw)),
        reflectance_mode_raw,
    )
    return_mode_raw = get_http_attribute(sensor_ip, "return_mode")
    return_mode = RETURN_MODE_VALUE_TO_NAME.get(
        int(float(return_mode_raw)),
        return_mode_raw,
    )
    time_sync = SYNC_MODE_VALUE_TO_NAME.get(
        int(
            float(
                parse_i_config_value(
                    raw_command(sensor_ip, "get_i_config time time_stamping"),
                    "time",
                    "time_stamping",
                )
            )
        ),
        "Unknown",
    )
    device_model = clean_value(raw_command(sensor_ip, "get_model"), "model:")
    if device_model == "h":
        frame_rate_raw = parse_i_config_value(
            raw_command(sensor_ip, "get_i_config spad frame_rate"), "spad", "frame_rate"
        )
    else:
        frame_rate_raw = raw_command(sensor_ip, "get_framerate")

    if not frame_rate_raw:
        frame_rate = 10.0  # Fallback
    else:
        try:
            frame_rate = float(frame_rate_raw)
        except ValueError:
            frame_rate = 10.0

    return ConfigInfo(
        sensor_ip=current_sensor_ip,
        mask=mask,
        gateway=gateway,
        destination_ip=destination_ip,
        data_port=data_port,
        message_port=message_port,
        status_port=status_port,
        return_mode=return_mode,
        reflectance_mode=reflectance_mode,
        time_sync=time_sync,
        frame_rate=frame_rate,
        horizontal_roi=horizontal_roi,
        vertical_roi=vertical_roi,
        udp_field_count=field_count,
    )


def read_status_info(sensor_ip: str) -> StatusInfo:
    # Try get_status first (older sensors)
    reply = raw_command(sensor_ip, "get_status")
    if reply and "stream_status" in reply:
        return StatusInfo(parse_status(reply))

    # Try get_mode_status (modern sensors like Robin E1X)
    reply_mode = http_get(sensor_ip, "/command/?get_mode_status")
    if reply_mode:
        res = parse_mode_status(reply_mode)
        if res:
            return StatusInfo(res)

    # Fallback to whatever raw reply we got
    return StatusInfo({"raw": reply if reply else "No response"})


def validate_port(name: str, port: int | None) -> None:
    if port is not None and (port < 0 or 65535 < port):
        raise ValueError(f"Invalid {name} {port}. It must be in the range of 0 to 65535.")


if __name__ == "__main__":

    class NameSpace:
        sensor_ip: str
        destination_ip: str | None
        data_port: int | None
        message_port: int | None
        status_port: int | None
        new_sensor_ip: str | None
        mask: str | None
        gateway: str | None
        return_mode: str | None
        reflectance_mode: str | None
        time_sync: str | None
        frame_rate: float | None
        horizontal_roi: float | None
        vertical_roi: float | None
        model: str
        download: bool
        stop: bool  # Restored
        start: bool  # Ensure start is here too

    parser = ArgumentParser()
    parser.add_argument(
        "--sensor-ip", type=str, required=True, help="The current sensor IP address"
    )
    parser.add_argument(
        "--destination-ip",
        type=str,
        default=None,
        help="Change the current destination IP address to the given one",
    )
    parser.add_argument(
        "--data-port",
        type=int,
        default=None,
        help="Change the current destination data port to the given one",
    )
    parser.add_argument(
        "--message-port",
        type=int,
        default=None,
        help="Change the current destination message port to the given one",
    )
    parser.add_argument(
        "--status-port",
        type=int,
        default=None,
        help="Change the current destination status port to the given one",
    )
    parser.add_argument(
        "--new-sensor-ip",
        "--new-ip",
        type=str,
        dest="new_sensor_ip",
        default=None,
        help="Change the current sensor IP address to the given one",
    )
    parser.add_argument(
        "--mask",
        type=str,
        default=None,
        help=(
            "Change the current net mask to the given one. "
            "You can pass it in either a CIDR notation or dotted-decimal notation."
        ),
    )
    parser.add_argument(
        "--gateway",
        type=str,
        default=None,
        help="Change the current gateway to the given one",
    )
    parser.add_argument(
        "--return-mode",
        type=str,
        default=None,
        help="Change the current return mode to one of: Single, Dual, StrongestFurthest",
    )
    parser.add_argument(
        "--reflectance-mode",
        type=str,
        default=None,
        help="Change the reflectance mode to one of: None, Intensity, Reflectivity",
    )
    parser.add_argument(
        "--time-sync",
        type=str,
        default=None,
        help="Change the time sync mode to one of: Host, PTP, GPS, File, NTP",
    )
    parser.add_argument(
        "--frame-rate",
        type=float,
        default=None,
        help="Change the frame rate to the given value in Hz",
    )
    parser.add_argument(
        "--horizontal-roi",
        type=float,
        default=None,
        help="Change the horizontal ROI to the given value in degrees",
    )
    parser.add_argument(
        "--vertical-roi",
        type=float,
        default=None,
        help="Change the vertical ROI to the given value in degrees",
    )
    parser.add_argument(
        "--reboot",
        action="store_true",
        help="Reboot the sensor after applying changes",
    )
    parser.add_argument(
        "--download",
        action="store_true",
        help="Download calibration and angle table files from the sensor",
    )
    parser.add_argument(
        "--keep-alive",
        action="store_true",
        help="Keep TCP connection open after starting stream",
    )
    parser.add_argument(
        "--start",
        action="store_true",
        help="Enable PCS and set mode to WorkNormal (start streaming)",
    )
    parser.add_argument(
        "--stop",
        action="store_true",
        help="Disable PCS (stop streaming)",
    )

    args = parser.parse_args(namespace=NameSpace)

    if not is_valid_ip(args.sensor_ip):
        raise ValueError(f"Invalid sensor IP {args.sensor_ip}")
    if not is_valid_ip(args.destination_ip):
        raise ValueError(f"Invalid destination IP {args.destination_ip}")
    if not is_valid_ip(args.new_sensor_ip):
        raise ValueError(f"Invalid new sensor IP {args.new_sensor_ip}")
    if not is_valid_ip(args.gateway):
        raise ValueError(f"Invalid gateway {args.gateway}")

    if args.mask is not None:
        if is_valid_netmask(args.mask):
            args.mask = normalize_netmask(args.mask)
        else:
            raise ValueError(
                f"Invalid net mask {args.mask}. "
                "It must be in the range of 0.0.0.0 (/0) to 255.255.255.255 (/32)."
            )

    validate_port("data_port", args.data_port)
    validate_port("message_port", args.message_port)
    validate_port("status_port", args.status_port)

    if args.return_mode is not None and args.return_mode not in RETURN_MODE_NAME_TO_VALUE:
        raise ValueError("Invalid return mode. Use Single, Dual, StrongestFurthest, 1, 2, or 3.")
    if (
        args.reflectance_mode is not None
        and args.reflectance_mode not in REFLECTANCE_MODE_NAME_TO_VALUE
    ):
        raise ValueError("Invalid reflectance mode. Use None, Intensity, Reflectivity, 0, 1, or 2.")
    if args.time_sync is not None and args.time_sync not in SYNC_MODE_NAME_TO_VALUE:
        raise ValueError("Invalid time sync mode. Use Host, PTP, GPS, File, NTP, or 0-4.")
    if args.frame_rate is not None and not 5.0 <= args.frame_rate <= 20.0:
        raise ValueError("Invalid frame rate. It must be in the range of 5.0 to 20.0 Hz.")

    device_info = read_device_info(args.sensor_ip)
    device_info.print(title="Device Info")
    print()
    args.model = device_info.model  # Populate args.model

    config_info = read_config_info(args.sensor_ip)
    config_info.print(title="Config Info")
    print()

    status_info = read_status_info(args.sensor_ip)
    status_info.print(title="Lidar Status")
    print()

    current_sensor_ip = config_info.sensor_ip
    current_mask = config_info.mask
    current_gateway = config_info.gateway
    current_destination_ip = config_info.destination_ip
    current_data_port = config_info.data_port
    current_message_port = config_info.message_port
    current_status_port = config_info.status_port
    current_return_mode = config_info.return_mode
    current_reflectance_mode = config_info.reflectance_mode
    current_time_sync = config_info.time_sync
    current_frame_rate = config_info.frame_rate
    current_horizontal_roi = config_info.horizontal_roi
    current_vertical_roi = config_info.vertical_roi

    destination_ip = current_destination_ip if args.destination_ip is None else args.destination_ip
    data_port = current_data_port if args.data_port is None else args.data_port
    message_port = current_message_port if args.message_port is None else args.message_port
    status_port = current_status_port if args.status_port is None else args.status_port
    new_sensor_ip = current_sensor_ip if args.new_sensor_ip is None else args.new_sensor_ip
    mask = current_mask if args.mask is None else args.mask
    gateway = current_gateway if args.gateway is None else args.gateway
    return_mode = current_return_mode if args.return_mode is None else args.return_mode
    reflectance_mode = (
        current_reflectance_mode if args.reflectance_mode is None else args.reflectance_mode
    )
    time_sync = current_time_sync if args.time_sync is None else args.time_sync
    frame_rate = current_frame_rate if args.frame_rate is None else args.frame_rate
    horizontal_roi = current_horizontal_roi if args.horizontal_roi is None else args.horizontal_roi
    vertical_roi = current_vertical_roi if args.vertical_roi is None else args.vertical_roi

    # Start configuring sequence
    pcs_stopped = False
    config_needed = (
        args.destination_ip
        or args.data_port
        or args.message_port
        or args.status_port
        or args.new_sensor_ip
        or args.mask
        or args.gateway
        or args.return_mode
        or args.reflectance_mode
        or args.time_sync
        or args.frame_rate
        or args.horizontal_roi
        or args.vertical_roi
    )

    if config_needed and not args.stop and not args.start:
        print("Stopping PCS for configuration...", end=" ", flush=True)
        try:
            set_http_attribute(args.sensor_ip, "enabled", "0")
            pcs_stopped = True
            print("Done")
        except Exception:
            print("Failed (continuing anyway)")

    # 1. Network settings first (affects subnet validation)
    if args.new_sensor_ip is not None or args.mask is not None or args.gateway is not None:
        if not yesno(
            "Are you sure you want to change the sensor network settings from "
            f"{current_sensor_ip}/{current_mask} gw={current_gateway} to "
            f"{new_sensor_ip}/{mask} gw={gateway}?"
        ):
            print("Aborted")
            sys.exit(0)

        # Determine if we should use dotted decimal (E1X/E2X) or octets (Legacy)
        is_modern = (
            "ROBIN" in args.model.upper()
            or "E1X" in args.model.upper()
            or "E2X" in args.model.upper()
        )
        netmask = mask

        if is_modern:
            net_cmd = f"set_network {new_sensor_ip} {netmask} {gateway}"
        else:
            net_cmd = (
                "set_network "
                + " ".join(new_sensor_ip.split("."))
                + " "
                + " ".join(netmask.split("."))
                + " "
                + " ".join((gateway if gateway else "0.0.0.0").split("."))
            )

        print(f"Applying: {net_cmd}...", end=" ")
        raw_command(args.sensor_ip, net_cmd)

        # Follow up with network save if on port 8010 or 8002
        try:
            # Try multiple save variants for compatibility
            set_http_attribute(args.sensor_ip, "save_network", "1")
            raw_command(args.sensor_ip, "save_config")
            print("(Saved)", end=" ")
        except Exception as e:
            print(f"(Save attempted: {e})", end=" ")

        print("Done")

    # 4. Download Calibration Files
    if args.download:
        print("\n[bold blue]Downloading Calibration Files...[/bold blue]")
        is_modern = (
            "ROBIN" in args.model.upper()
            or "E1X" in args.model.upper()
            or "E2X" in args.model.upper()
        )

        # 1. Try anglehv_table via HTTP (prefer 8010)
        try:
            print("Checking anglehv_table...", end=" ", flush=True)
            table_data = http_get(
                args.sensor_ip,
                "/command/?get_anglehv_table",
                port=DETECTED_HTTP_PORT if DETECTED_HTTP_PORT != 80 else 8010,
                as_bytes=True,
            )
            if table_data:
                with open("anglehv_table.bin", "wb") as f:
                    # If it's pure binary, write it; if it's hex, decode it
                    is_hex = False
                    if len(table_data) < 1000:
                        try:
                            is_hex = all(
                                c in bytes(b"0123456789abcdefABCDEF \n\r") for c in table_data[:100]
                            )
                        except Exception:
                            pass

                    if is_hex and b" " in table_data:
                        f.write(
                            bytes.fromhex(
                                table_data.decode("ascii")
                                .replace(" ", "")
                                .replace("\n", "")
                                .replace("\r", "")
                            )
                        )
                    else:
                        f.write(table_data)
                print(f"Done (saved {len(table_data)} bytes to anglehv_table.bin)")
            else:
                print("Not found.")
        except Exception as e:
            print(f"Skipped ({e})")

        # 2. Try geo_yaml (cal file 0) via TCP
        success = download_binary_file(
            args.sensor_ip, "download_cal_file 0", "geo_calibration.yaml"
        )
        if not success:
            # Try variant
            download_binary_file(args.sensor_ip, "get_geo_yaml", "geo_calibration.yaml")

        # 3. Try sn.yaml (cal file 1)
        download_binary_file(args.sensor_ip, "download_cal_file 1", "sn_calibration.yaml")

    # 5. Summary
    # 2. Destination settings (can depend on subnet)
    if args.destination_ip or args.data_port or args.message_port or args.status_port:
        print(
            "Changing destination IP/data_port/message_port/status_port from "
            f"{current_destination_ip}:{current_data_port}:{current_message_port}:{current_status_port} to "
            f"{destination_ip}:{data_port}:{message_port}:{status_port}...",
            end=" ",
            flush=True,
        )

        # Subnet check for destination against current OR new network
        active_sensor_ip = new_sensor_ip
        active_mask = mask
        if not is_same_subnet(destination_ip, active_sensor_ip, active_mask):
            print(
                f"\n[bold yellow]Warning:[/bold yellow] Destination IP {destination_ip} is not in the same subnet as sensor {active_sensor_ip}/{active_mask}."
            )
            print("The sensor may reject this setting with HTTP 400 if it cannot route to it.\n")

        # Try preferred format based on current field count
        formats = []
        # Modern sensors (E1X) often prefer 6 fields,
        # but Falcon-based (Hummingbird/Robin) prefer 4 or even 3.
        # Force 4-field if not E1X to be safe (matching nebula driver)
        if config_info.udp_field_count >= 6:
            formats = [
                f"{data_port},{message_port},{status_port},{destination_ip},{destination_ip},{config_info.sensor_ip}",
                f"{data_port},{message_port},{status_port},{destination_ip}",
            ]
        else:
            formats = [f"{data_port},{message_port},{status_port},{destination_ip}"]

        success = False
        last_err = ""
        for params in formats:
            try:
                set_http_attribute(args.sensor_ip, "udp_ports_ip", params)
                success = True
                break
            except Exception as e:
                last_err = str(e)
                continue

        if success:
            print("Done")
        else:
            print(f"Failed: {last_err}")
            if "400" in last_err:
                print("Suggestion: Ensure the sensor and destination are in the same subnet.")

        print(
            f"Make sure the new sensor network settings are successfully applied for {new_sensor_ip}/{mask} gw={gateway} by the following command"
        )
        print(Syntax(f"python3 {sys.argv[0]} --sensor-ip {new_sensor_ip}", "console"))

    if pcs_stopped or args.start:
        print("Starting PCS sequence...", end=" ", flush=True)
        # Ensure destination is set BEFORE enabled=1 for Falcon sensors
        try:
            dp = args.data_port or 2371
            # Aggressive network push
            raw_command(
                args.sensor_ip,
                f"set_attribute udp_ports_ip={dp},{dp},{dp},{args.new_destination_ip or config_info.destination_ip}",
            )
        except Exception:
            pass

        # Aggressive enabled
        try:
            set_http_attribute(args.sensor_ip, "enabled", "1")
        except Exception:
            pass
        try:
            raw_command(args.sensor_ip, "set_attribute enabled=1")
        except Exception:
            pass

        print("Done")
        import time

        time.sleep(1)  # Give PCS time to initialize

        print("Setting WorkNormal mode...", end=" ", flush=True)
        # Try 3 (SDK standard), then 1 (Old/Nebula standard) if 3 fails
        try:
            set_http_attribute(args.sensor_ip, "mode", "3")
            print("Done (mode=3)")
        except Exception as e3:
            try:
                set_http_attribute(args.sensor_ip, "mode", "1")
                print("Done (mode=1 fallback)")
            except Exception as e1:
                # Try raw command fallback
                try:
                    raw_command(args.sensor_ip, "set_attribute mode=3")
                    print("Done (TCP mode=3)")
                except Exception:
                    print(f"Failed: SDK mode=3: {e3}, Nebula mode=1: {e1}")

        # Active trigger for Falcon-based sensors (Hummingbird)
        # Use data port from config if available
        dp = args.data_port or 2371
        trigger_streaming(args.sensor_ip, data_port=dp)

        if args.keep_alive:
            print(f"\n[KEEP-ALIVE] Maintaining control connection to {args.sensor_ip}...")
            print("[KEEP-ALIVE] Press Ctrl+C to stop streaming.")
            try:
                # We need to keep the socket that trigger_streaming used?

                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(10)
                    s.connect((args.sensor_ip, 8002))
                    # Periodically send a NOP or get_status to prevent timeout
                    while True:
                        s.sendall(b"get_status\n")
                        s.recv(1024)
                        time.sleep(2)
            except KeyboardInterrupt:
                print("\nStopping stream...")
                raw_command(args.sensor_ip, "stop")
            except Exception as e:
                print(f"[KEEP-ALIVE] Connection lost: {e}")

    if args.stop:
        print("Stopping PCS...", end=" ", flush=True)
        try:
            set_http_attribute(args.sensor_ip, "enabled", "0")
            print("Done")
        except Exception as e:
            print(f"Failed: {e}")

    print("\nFinal State Verification:")
    try:
        final_config = read_config_info(args.sensor_ip)
        final_status = read_status_info(args.sensor_ip)
        print(f"  Sensor IP:     {final_config.sensor_ip}")
        print(f"  Subnet Mask:   {final_config.mask}")
        print(f"  Destination:   {final_config.destination_ip}:{final_config.data_port}")
        print(
            f"  Stream Status: [bold cyan]{final_status.status.get('stream_status', 'Unknown')}[/bold cyan]"
        )
        print(f"  Data Sent:     {final_status.status.get('data_sent', '0')}")

        raw_status = final_status.status.get("stream_status")
        status_text = final_status.status.get("status_text", "")

        is_streaming = raw_status in ["1", "3"] or status_text == "Normal"
        is_ready = raw_status == "2" and status_text != "Normal"
        is_idle = raw_status == "0" or status_text in ["Standby", "Sleep"]

        if is_streaming:
            print("\n[bold green]Success: Sensor is reported as streaming![/bold green]")
        elif is_ready:
            print(
                "\n[bold yellow]Note:[/bold yellow] Stream status is [bold]READY[/bold]. Sensor is initialized but NOT yet transmitting."
            )
            if final_config.destination_ip == "0.0.0.0":
                print(
                    "[bold red]Action Required:[/bold red] Destination IP is still 0.0.0.0. The sensor requires a valid destination to stream."
                )
            else:
                print(
                    "Suggestion: Ensure the Lidar mode is 'WorkNormal' (mode=3) and check for any hardware errors."
                )
        elif is_idle:
            print("\n[bold yellow]Note:[/bold yellow] Stream status is [bold]IDLE[/bold].")
            print(
                "Troubleshooting: Check if the Lidar is in 'Standby' in the Web UI or try manually toggling PCS."
            )
        else:
            print(f"\nStream status is {raw_status} (Unknown).")
            if status_text:
                print(f"Status Text: {status_text}")
    except Exception as e:
        print(f"  Could not verify final status: {e}")

    if args.reboot:
        print(f"Rebooting sensor {args.sensor_ip}...", end=" ", flush=True)
        try:
            set_http_attribute(args.sensor_ip, "reboot", "1")
            print("Done")
        except Exception as e:
            print(f"Failed: {e}")
            # Try raw command on 8002 as fallback
            try:
                raw_command(args.sensor_ip, "reboot 1")
                print("Done (via raw command)")
            except Exception:
                print("Failed completely")

    if any(
        value is not None
        for value in (
            args.destination_ip,
            args.data_port,
            args.message_port,
            args.status_port,
            args.new_sensor_ip,
            args.mask,
            args.gateway,
            args.return_mode,
            args.reflectance_mode,
            args.time_sync,
            args.frame_rate,
            args.horizontal_roi,
            args.vertical_roi,
        )
    ):
        table = Table(
            "Parameter", "Old", "New", title="What's Changed", highlight=True, min_width=50
        )

        if new_sensor_ip != current_sensor_ip:
            table.add_row("sensor_ip", str(current_sensor_ip), str(new_sensor_ip))
        if destination_ip != current_destination_ip:
            table.add_row("destination_ip", str(current_destination_ip), str(destination_ip))
        if data_port != current_data_port:
            table.add_row("data_port", str(current_data_port), str(data_port))
        if message_port != current_message_port:
            table.add_row("message_port", str(current_message_port), str(message_port))
        if status_port != current_status_port:
            table.add_row("status_port", str(current_status_port), str(status_port))
        if mask != current_mask:
            table.add_row("mask", str(current_mask), str(mask))
        if gateway != current_gateway:
            table.add_row("gateway", str(current_gateway), str(gateway))
        if return_mode != current_return_mode:
            table.add_row("return_mode", str(current_return_mode), str(return_mode))
        if reflectance_mode != current_reflectance_mode:
            table.add_row("reflectance_mode", str(current_reflectance_mode), str(reflectance_mode))
        if time_sync != current_time_sync:
            table.add_row("time_sync", str(current_time_sync), str(time_sync))
        if frame_rate != current_frame_rate:
            table.add_row("frame_rate", f"{current_frame_rate:.5f}", f"{frame_rate:.5f}")
        if horizontal_roi != current_horizontal_roi:
            table.add_row(
                "horizontal_roi",
                f"{current_horizontal_roi:.6f}",
                f"{horizontal_roi:.6f}",
            )
        if vertical_roi != current_vertical_roi:
            table.add_row(
                "vertical_roi",
                f"{current_vertical_roi:.6f}",
                f"{vertical_roi:.6f}",
            )

        print()
        print(table)
