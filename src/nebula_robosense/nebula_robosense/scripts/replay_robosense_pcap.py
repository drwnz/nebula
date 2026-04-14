#!/usr/bin/env python3

import argparse
import socket
import struct
import time

ETHERNET_LINKTYPE = 1
LINUX_SLL_LINKTYPE = 113
ETHER_TYPE_IPV4 = 0x0800
ETHER_TYPE_VLAN = 0x8100
UDP_PROTOCOL = 17


class FastPcapReader:
    def __init__(self, filename):
        self._file = open(filename, "rb")
        global_header = self._file.read(24)
        if len(global_header) != 24:
            raise ValueError("Invalid PCAP header")

        magic_bytes = global_header[:4]
        if magic_bytes == b"\xd4\xc3\xb2\xa1":
            self._endian = "<"
        elif magic_bytes == b"\xa1\xb2\xc3\xd4":
            self._endian = ">"
        else:
            raise ValueError("Unsupported PCAP format")

        self._record_header_fmt = self._endian + "IIII"
        _, _, _, _, _, _, self.link_type = struct.unpack(self._endian + "IHHIIII", global_header)

    def __iter__(self):
        while True:
            header_data = self._file.read(16)
            if len(header_data) == 0:
                break
            if len(header_data) != 16:
                raise ValueError("Truncated PCAP record header")

            ts_sec, ts_usec, incl_len, _ = struct.unpack(self._record_header_fmt, header_data)
            packet_data = self._file.read(incl_len)
            if len(packet_data) != incl_len:
                raise ValueError("Truncated PCAP packet data")

            yield ts_sec * 1_000_000 + ts_usec, packet_data

    def close(self):
        self._file.close()


def parse_port_maps(mappings):
    port_map = {}
    for mapping in mappings:
        old_port, new_port = mapping.split(":", 1)
        port_map[int(old_port)] = int(new_port)
    return port_map


def get_link_offset(packet_data, link_type):
    if link_type == ETHERNET_LINKTYPE:
        if len(packet_data) < 14:
            return None
        offset = 14
        ether_type = struct.unpack(">H", packet_data[12:14])[0]
        while ether_type == ETHER_TYPE_VLAN:
            if len(packet_data) < offset + 4:
                return None
            ether_type = struct.unpack(">H", packet_data[offset + 2 : offset + 4])[0]
            offset += 4
        if ether_type != ETHER_TYPE_IPV4:
            return None
        return offset

    if link_type == LINUX_SLL_LINKTYPE:
        if len(packet_data) < 16:
            return None
        if struct.unpack(">H", packet_data[14:16])[0] != ETHER_TYPE_IPV4:
            return None
        return 16

    return None


def extract_udp_payload(packet_data, link_type, fragments):
    link_offset = get_link_offset(packet_data, link_type)
    if link_offset is None or len(packet_data) < link_offset + 20:
        return None

    ipv4 = packet_data[link_offset:]
    version = ipv4[0] >> 4
    ihl = (ipv4[0] & 0x0F) * 4
    if version != 4 or ihl < 20 or len(ipv4) < ihl:
        return None

    total_length = struct.unpack(">H", ipv4[2:4])[0]
    if total_length < ihl or len(ipv4) < total_length:
        return None
    ipv4 = ipv4[:total_length]

    if ipv4[9] != UDP_PROTOCOL:
        return None

    identification = struct.unpack(">H", ipv4[4:6])[0]
    fragment_bits = struct.unpack(">H", ipv4[6:8])[0]
    more_fragments = bool(fragment_bits & 0x2000)
    fragment_offset = (fragment_bits & 0x1FFF) * 8

    src_ip = socket.inet_ntoa(ipv4[12:16])
    dst_ip = socket.inet_ntoa(ipv4[16:20])
    ip_payload = ipv4[ihl:]

    if fragment_offset == 0:
        if len(ip_payload) < 8:
            return None
        src_port, dst_port, udp_length, _ = struct.unpack(">HHHH", ip_payload[:8])
        udp_payload = ip_payload[8:]
        if udp_length < 8:
            return None
        expected_payload_length = udp_length - 8
        if not more_fragments:
            return {
                "src_ip": src_ip,
                "dst_ip": dst_ip,
                "src_port": src_port,
                "dst_port": dst_port,
                "payload": udp_payload[:expected_payload_length],
            }

        fragments[(src_ip, dst_ip, identification)] = {
            "src_port": src_port,
            "dst_port": dst_port,
            "expected_length": expected_payload_length,
            "parts": {0: udp_payload},
        }
        return None

    key = (src_ip, dst_ip, identification)
    state = fragments.get(key)
    if state is None:
        return None

    state["parts"][fragment_offset - 8] = ip_payload
    if more_fragments:
        return None

    payload = bytearray(state["expected_length"])
    written = 0
    for offset, chunk in sorted(state["parts"].items()):
        if offset >= state["expected_length"]:
            continue
        end = min(offset + len(chunk), state["expected_length"])
        payload[offset:end] = chunk[: end - offset]
        written += end - offset

    del fragments[key]
    if written < state["expected_length"]:
        return None

    return {
        "src_ip": src_ip,
        "dst_ip": dst_ip,
        "src_port": state["src_port"],
        "dst_port": state["dst_port"],
        "payload": bytes(payload),
    }


def should_send(packet, include_ports, exclude_ports):
    dst_port = packet["dst_port"]
    if include_ports and dst_port not in include_ports:
        return False
    if dst_port in exclude_ports:
        return False
    return True


def print_stream_summary(pcap_file):
    reader = FastPcapReader(pcap_file)
    streams = {}
    fragments = {}
    try:
        for _, packet_data in reader:
            parsed = extract_udp_payload(packet_data, reader.link_type, fragments)
            if not parsed:
                continue
            key = (
                parsed["src_ip"],
                parsed["src_port"],
                parsed["dst_ip"],
                parsed["dst_port"],
            )
            stats = streams.setdefault(key, {"count": 0, "min_len": None, "max_len": 0})
            payload_len = len(parsed["payload"])
            stats["count"] += 1
            stats["min_len"] = (
                payload_len if stats["min_len"] is None else min(stats["min_len"], payload_len)
            )
            stats["max_len"] = max(stats["max_len"], payload_len)

        for key, stats in sorted(streams.items(), key=lambda item: item[0]):
            src_ip, src_port, dst_ip, dst_port = key
            print(
                f"{src_ip}:{src_port} -> {dst_ip}:{dst_port} "
                f"count={stats['count']} payload_len={stats['min_len']}..{stats['max_len']}"
            )
    finally:
        reader.close()


def replay_pcap(
    pcap_file,
    target_ip,
    rate,
    loop,
    max_gap_ms,
    use_pcap_time,
    override_port,
    include_ports,
    exclude_ports,
    port_map,
):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        while True:
            reader = FastPcapReader(pcap_file)
            fragments = {}
            first_packet_time = None
            first_wall_time = None
            last_packet_time = None
            sent_packets = 0

            try:
                for pcap_ts_us, packet_data in reader:
                    parsed = extract_udp_payload(packet_data, reader.link_type, fragments)
                    if not parsed or not should_send(parsed, include_ports, exclude_ports):
                        continue

                    packet_time = pcap_ts_us if use_pcap_time else pcap_ts_us
                    if first_packet_time is None:
                        first_packet_time = packet_time
                        first_wall_time = time.perf_counter()
                        last_packet_time = packet_time

                    gap_us = packet_time - last_packet_time
                    if max_gap_ms > 0 and gap_us > max_gap_ms * 1000:
                        first_wall_time -= (gap_us - max_gap_ms * 1000) / 1_000_000.0 / rate

                    last_packet_time = packet_time
                    target_time = (
                        first_wall_time + ((packet_time - first_packet_time) / 1_000_000.0) / rate
                    )
                    while True:
                        remaining = target_time - time.perf_counter()
                        if remaining <= 0:
                            break
                        if remaining > 0.002:
                            time.sleep(remaining - 0.001)

                    dst_port = override_port or port_map.get(parsed["dst_port"], parsed["dst_port"])
                    sock.sendto(parsed["payload"], (target_ip, dst_port))
                    sent_packets += 1
                    if sent_packets % 1000 == 0:
                        print(f"Sent {sent_packets} packets...", end="\r")
            finally:
                reader.close()

            print(f"\nFinished replaying {sent_packets} packets from {pcap_file}")
            if not loop:
                break
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(
        description="Replay RoboSense UDP payloads from a PCAP into Nebula."
    )
    parser.add_argument("pcap", help="Path to a PCAP file")
    parser.add_argument("--ip", default="127.0.0.1", help="Target IP address (default: 127.0.0.1)")
    parser.add_argument(
        "--port",
        type=int,
        help="Override every destination UDP port with a single target port",
    )
    parser.add_argument(
        "--port-map",
        action="append",
        default=[],
        metavar="OLD:NEW",
        help="Remap destination UDP ports while preserving separate data/info streams",
    )
    parser.add_argument(
        "--include-port",
        action="append",
        default=[],
        type=int,
        help="Only replay packets whose destination port matches this value",
    )
    parser.add_argument(
        "--exclude-port",
        action="append",
        default=[],
        type=int,
        help="Skip packets whose destination port matches this value",
    )
    parser.add_argument("--rate", type=float, default=1.0, help="Playback rate")
    parser.add_argument("--loop", action="store_true", help="Loop the PCAP")
    parser.add_argument(
        "--max-gap-ms",
        type=float,
        default=100.0,
        help="Clamp large inter-packet gaps in milliseconds",
    )
    parser.add_argument(
        "--pcap-time",
        action="store_true",
        help="Use recorded PCAP timestamps for pacing",
    )
    parser.add_argument(
        "--list-streams",
        action="store_true",
        help="Print UDP streams discovered in the PCAP and exit",
    )
    args = parser.parse_args()

    if args.list_streams:
        print_stream_summary(args.pcap)
        return

    replay_pcap(
        pcap_file=args.pcap,
        target_ip=args.ip,
        rate=args.rate,
        loop=args.loop,
        max_gap_ms=args.max_gap_ms,
        use_pcap_time=args.pcap_time,
        override_port=args.port,
        include_ports=set(args.include_port),
        exclude_ports=set(args.exclude_port),
        port_map=parse_port_maps(args.port_map),
    )


if __name__ == "__main__":
    main()
