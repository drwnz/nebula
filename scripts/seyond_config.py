# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "rich",
# ]
# ///
from __future__ import annotations

from argparse import ArgumentParser
from dataclasses import dataclass
from ipaddress import IPv4Address
from ipaddress import IPv4Network
from ipaddress import ip_interface
import re
import socket
import sys
from typing import Any
from urllib.error import HTTPError
from urllib.error import URLError
from urllib.parse import quote
from urllib.request import urlopen

try:
    from rich import print  # noqa: A004
    from rich.syntax import Syntax
    from rich.table import Table
except ModuleNotFoundError:
    from builtins import print

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

CONTROL_PORT = 8010
HTTP_PORT = 80
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


def raw_command(sensor_ip: str, command: str, timeout_sec: float = CONTROL_TIMEOUT_SEC) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout_sec)
        sock.connect((sensor_ip, CONTROL_PORT))
        sock.sendall((command + "\n").encode())

        reply = bytearray()
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            reply.extend(chunk)
            if b"\n\n" in reply:
                break

    return reply.decode(errors="replace").strip()


def http_get(sensor_ip: str, endpoint: str) -> str:
    url = f"http://{sensor_ip}:{HTTP_PORT}{endpoint}"
    try:
        with urlopen(url, timeout=HTTP_TIMEOUT_SEC) as response:  # noqa: S310
            return response.read().decode(errors="replace").strip()
    except HTTPError as exc:
        raise RuntimeError(f"HTTP {exc.code} for {endpoint}") from exc
    except URLError as exc:
        raise RuntimeError(f"Failed to reach {url}") from exc


def get_http_attribute(sensor_ip: str, name: str) -> str:
    return http_get(sensor_ip, f"/command/?get_{quote(name)}")


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


def extract_ipv4s(reply: str) -> list[str]:
    return re.findall(r"\b(?:\d{1,3}\.){3}\d{1,3}\b", reply)


def parse_network(reply: str, fallback_sensor_ip: str) -> tuple[str, str, str]:
    ips = extract_ipv4s(reply)
    sensor_ip = fallback_sensor_ip
    mask = "255.255.255.0"
    gateway = "0.0.0.0"
    if len(ips) >= 1:
        sensor_ip = ips[0]
    if len(ips) >= 2:
        mask = ips[1]
    if len(ips) >= 3:
        gateway = ips[2]
    return sensor_ip, mask, gateway


def parse_udp_ports_ip(reply: str) -> tuple[int, int, int, str]:
    fields = [field.strip() for field in reply.split(",")]
    if len(fields) < 4:
        raise ValueError(f"Unexpected udp_ports_ip reply: {reply}")

    return int(fields[0]), int(fields[1]), int(fields[2]), fields[3]


def parse_roi(reply: str) -> tuple[float, float]:
    horizontal_roi, vertical_roi = reply.split(",", maxsplit=1)
    return float(horizontal_roi), float(vertical_roi)


def parse_i_config_value(reply: str, section: str, key: str) -> str:
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
            value = SYNC_MODE_VALUE_TO_NAME.get(int(value, 0), value)
        elif source_key == "up_time":
            value = f"{int(value, 0)} s"
        values[output_key] = value

    if not values:
        values["raw"] = reply
    return values


def read_device_info(sensor_ip: str) -> DeviceInfo:
    return DeviceInfo(
        model=raw_command(sensor_ip, "get_model"),
        serial_number=raw_command(sensor_ip, "get_sn"),
        fw_version=raw_command(sensor_ip, "get_fw_version"),
    )


def read_config_info(sensor_ip: str) -> ConfigInfo:
    current_sensor_ip, mask, gateway = parse_network(raw_command(sensor_ip, "get_network"), sensor_ip)
    data_port, message_port, status_port, destination_ip = parse_udp_ports_ip(
        get_http_attribute(sensor_ip, "udp_ports_ip")
    )
    horizontal_roi, vertical_roi = parse_roi(get_http_attribute(sensor_ip, "roi"))
    reflectance_mode_raw = get_http_attribute(sensor_ip, "reflectance_mode")
    reflectance_mode = REFLECTANCE_MODE_VALUE_TO_NAME.get(
        int(reflectance_mode_raw),
        reflectance_mode_raw,
    )
    return_mode_raw = get_http_attribute(sensor_ip, "return_mode")
    return_mode = RETURN_MODE_VALUE_TO_NAME.get(
        int(return_mode_raw),
        return_mode_raw,
    )
    time_sync = SYNC_MODE_VALUE_TO_NAME.get(
        int(parse_i_config_value(raw_command(sensor_ip, "get_i_config time time_stamping"), "time", "time_stamping")),
        "Unknown",
    )
    frame_rate = float(raw_command(sensor_ip, "get_framerate"))

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
    )


def read_status_info(sensor_ip: str) -> StatusInfo:
    return StatusInfo(parse_status(raw_command(sensor_ip, "get_status")))


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

    parser = ArgumentParser()
    parser.add_argument("--sensor-ip", type=str, required=True, help="The current sensor IP address")
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
        type=str,
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
    if args.reflectance_mode is not None and args.reflectance_mode not in REFLECTANCE_MODE_NAME_TO_VALUE:
        raise ValueError("Invalid reflectance mode. Use None, Intensity, Reflectivity, 0, 1, or 2.")
    if args.time_sync is not None and args.time_sync not in SYNC_MODE_NAME_TO_VALUE:
        raise ValueError("Invalid time sync mode. Use Host, PTP, GPS, File, NTP, or 0-4.")
    if args.frame_rate is not None and not 5.0 <= args.frame_rate <= 20.0:
        raise ValueError("Invalid frame rate. It must be in the range of 5.0 to 20.0 Hz.")

    device_info = read_device_info(args.sensor_ip)
    device_info.print(title="Device Info")
    print()

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

    if (
        args.destination_ip is not None
        or args.data_port is not None
        or args.message_port is not None
        or args.status_port is not None
    ):
        if not is_same_subnet(destination_ip, new_sensor_ip, mask):
            raise ValueError(
                f"Destination IP {destination_ip} is not in the same subnet as the sensor IP {new_sensor_ip}. "
                f"The net mask is {mask}."
            )

        print(
            "Changing destination IP/data_port/message_port/status_port from "
            f"{current_destination_ip}:{current_data_port}:{current_message_port}:{current_status_port} to "
            f"{destination_ip}:{data_port}:{message_port}:{status_port}...",
            end=" ",
            flush=True,
        )
        set_http_attribute(
            args.sensor_ip,
            "udp_ports_ip",
            f"{data_port},{message_port},{status_port},{destination_ip}",
        )
        print("Done")

    if args.return_mode is not None:
        print(
            f"Changing return_mode from {current_return_mode} to {return_mode}...",
            end=" ",
            flush=True,
        )
        set_http_attribute(args.sensor_ip, "return_mode", str(RETURN_MODE_NAME_TO_VALUE[return_mode]))
        print("Done")

    if args.reflectance_mode is not None:
        print(
            f"Changing reflectance_mode from {current_reflectance_mode} to {reflectance_mode}...",
            end=" ",
            flush=True,
        )
        set_http_attribute(
            args.sensor_ip,
            "reflectance_mode",
            str(REFLECTANCE_MODE_NAME_TO_VALUE[reflectance_mode]),
        )
        print("Done")

    if args.time_sync is not None:
        print(
            f"Changing time_sync from {current_time_sync} to {time_sync}...",
            end=" ",
            flush=True,
        )
        raw_command(
            args.sensor_ip,
            f"set_i_config time time_stamping {SYNC_MODE_NAME_TO_VALUE[time_sync]}",
        )
        print("Done")

    if args.frame_rate is not None:
        print(
            f"Changing frame_rate from {current_frame_rate:.5f} to {frame_rate:.5f}...",
            end=" ",
            flush=True,
        )
        raw_command(args.sensor_ip, f"set_i_config motor galvo_framerate {frame_rate:.6f}")
        print("Done")

    if args.horizontal_roi is not None or args.vertical_roi is not None:
        print(
            "Changing horizontal_roi/vertical_roi from "
            f"{current_horizontal_roi:.6f}:{current_vertical_roi:.6f} to "
            f"{horizontal_roi:.6f}:{vertical_roi:.6f}...",
            end=" ",
            flush=True,
        )
        set_http_attribute(args.sensor_ip, "roi", f"{horizontal_roi:.6f},{vertical_roi:.6f}")
        print("Done")

    if args.new_sensor_ip is not None or args.mask is not None or args.gateway is not None:
        if not yesno(
            "Are you sure you want to change the sensor network settings from "
            f"{current_sensor_ip}/{current_mask} gw={current_gateway} to "
            f"{new_sensor_ip}/{mask} gw={gateway}?"
        ):
            print("Aborted")
            sys.exit(0)

        print(
            f"Changing sensor network settings from {current_sensor_ip}/{current_mask} gw={current_gateway} "
            f"to {new_sensor_ip}/{mask} gw={gateway}...",
            end=" ",
            flush=True,
        )
        raw_command(
            args.sensor_ip,
            "set_network "
            + " ".join(new_sensor_ip.split("."))
            + " "
            + " ".join(mask.split("."))
            + " "
            + " ".join((gateway if gateway else "0.0.0.0").split(".")),
        )
        print("Done")

        print()
        print(
            f"Make sure the new sensor network settings are successfully applied for {new_sensor_ip}/{mask} gw={gateway} by the following command"
        )
        print(Syntax(f"python3 {sys.argv[0]} --sensor-ip {new_sensor_ip}", "console"))

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
        table = Table("Parameter", "Old", "New", title="What's Changed", highlight=True, min_width=50)

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
