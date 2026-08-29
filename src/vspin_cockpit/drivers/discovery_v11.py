r"""Velocity11 / Agilent Ethernet device discovery (UDP broadcast).

The Agilent VWorks plugin family includes a "Find available device"
button on the Profiles tab of every Ethernet-enabled instrument
(Access2 plate loader, BlueWasher, etc.). It works by broadcasting a
5-byte query on UDP port 7611 and collecting responses from any
listening boards on the local network.

Wire protocol (ported verbatim from
`/Users/kelsorj/GitHub/Great-Conversion/DLLs/DeviceEnumerator/
DeviceList.cpp`):

  Query  (5 bytes + 2-byte CRC, 7 bytes total):
    [0x11, 0x02, 0x00, 0x00, 0x00, crc_hi, crc_lo]
         |     |    \________________/
       header  cmd      reserved
       (v11)  (broadcast query)

  Response (variable length, 5 + N + 2 bytes):
    [0x11, 0x03, len, port_hi, port_lo, type_string..., crc_hi, crc_lo]
         |     |     |       |
       header  cmd  data_len (= 2 + len(type_string))
              (broadcast respond)

The CRC is CRC-16-CCITT/XMODEM (polynomial 0x1021, init 0x0000, no
input/output reflection) appended in BIG-endian order. Both query and
response carry it on the last two bytes.

The device's IP comes from the UDP packet source address. The
`type_string` is something like "Access2" which the original code
mapped to a friendlier display string ("Centrifuge Loader") via
`LookupDeviceTypeToDisplay`. We surface both.

Usage:
    devices = await discover_v11_devices(timeout_s=2.0)
    # → [{'ip': '192.168.0.66', 'port': 7612, 'type': 'Access2'}, ...]
"""

from __future__ import annotations

import asyncio
import ipaddress
import socket
from dataclasses import dataclass

# ---- wire constants -------------------------------------------------------

V11_BROADCAST_PORT      = 7611
V11_PACKET_HEADER       = 0x11
V11_CMD_NAK             = 0x01
V11_CMD_BROADCAST_QUERY = 0x02
V11_CMD_BROADCAST_RESP  = 0x03

# Map raw device-type strings to friendlier display names (matches
# `LookupDeviceTypeToDisplay` in the C++ source — extend as we identify
# more Agilent Ethernet instruments).
TYPE_DISPLAY_MAP = {
    "Access2":     "Centrifuge Loader",
    "BlueWasher":  "BlueWasher",
    "Bravo":       "Bravo",
}


@dataclass
class DiscoveredDevice:
    ip: str
    port: int          # the TCP port to connect to (NOT the discovery port)
    type: str          # raw type string from the device, e.g. "Access2"
    display_name: str  # human label, e.g. "Centrifuge Loader"


# ---- CRC-16-CCITT (XMODEM variant) ---------------------------------------


def crc16_ccitt(data: bytes) -> int:
    """CRC-16-CCITT with polynomial 0x1021, init 0x0000, no reflection,
    big-endian output. Bit-for-bit equivalent to the C++
    `CRCCalculator::CRCCalculator(message, length)` in
    `DLLs/DeviceEnumerator/CRCCalculator.cpp`."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_query() -> bytes:
    """Marshal the 7-byte broadcast query (5 message bytes + 2 CRC)."""
    body = bytes([V11_PACKET_HEADER, V11_CMD_BROADCAST_QUERY, 0, 0, 0])
    crc = crc16_ccitt(body)
    return body + bytes([(crc >> 8) & 0xFF, crc & 0xFF])


def parse_response(data: bytes, source_ip: str) -> DiscoveredDevice | None:
    """Parse one UDP response packet. Returns None if the packet is
    malformed, NAK, or fails CRC. We're permissive — discovery should
    never raise; bad packets are just ignored."""
    if len(data) < 7:  # header + cmd + len + port + at least 1 type byte + crc
        return None
    if data[0] != V11_PACKET_HEADER:
        return None
    cmd = data[1]
    if cmd == V11_CMD_NAK or cmd != V11_CMD_BROADCAST_RESP:
        return None
    length = data[2]  # length = 2 (port bytes) + len(type_string)
    if length < 2 or len(data) < length + 5:  # +5 = header+cmd+len+CRC(2)
        return None
    port = (data[3] << 8) | data[4]
    type_bytes = data[5:3 + length]  # skip header(1)+cmd(1)+len(1)+port(2) ... = 5; up to 3+length (which is the start of CRC)
    crc_field = (data[3 + length] << 8) | data[4 + length]
    # CRC was computed over everything up to (but not including) the CRC bytes.
    expected = crc16_ccitt(data[:3 + length])
    if crc_field != expected:
        return None
    try:
        type_str = type_bytes.decode("ascii", errors="replace").strip("\x00").strip()
    except Exception:
        type_str = "?"
    return DiscoveredDevice(
        ip=source_ip, port=port,
        type=type_str,
        display_name=TYPE_DISPLAY_MAP.get(type_str, type_str or "Unknown"),
    )


# ---- async discovery ------------------------------------------------------


class _V11DiscoveryProtocol(asyncio.DatagramProtocol):
    """Collect v11 broadcast responses into a queue. We don't filter
    here — `discover_v11_devices` decodes + dedupes after the timeout."""
    def __init__(self) -> None:
        self.received: list[tuple[bytes, tuple[str, int]]] = []

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        self.received.append((data, addr))

    def error_received(self, exc: Exception) -> None:
        # Ignore ICMP-port-unreachable and similar; not all subnets
        # respond cleanly to a broadcast.
        pass


def _local_subnet_broadcasts() -> list[str]:
    """Best-effort enumeration of directed broadcast addresses for each
    routable IPv4 interface on this host, plus 255.255.255.255 as a
    catch-all. Uses only the standard library.

    Why directed broadcasts: macOS doesn't route 255.255.255.255
    packets onto local subnets reliably — it only sends them out the
    default interface. Lab benches are often on a separate Ethernet
    subnet (e.g., 192.168.0.0/24) reached via a non-default interface,
    so we need to send the discovery query to 192.168.0.255 specifically.

    The C++ reference does the same: `GetBroadcastAddress(adapter, …)`
    in DLLs/DeviceEnumerator/DeviceList.cpp builds the broadcast from
    the selected adapter's IP + netmask, not 255.255.255.255."""
    out: list[str] = []
    seen: set[str] = set()

    def add_local_ip(local_ip: str) -> None:
        if local_ip.startswith("127.") or local_ip == "0.0.0.0":
            return
        parts = local_ip.split(".")
        if len(parts) != 4:
            return
        # Assume /24 — true for virtually every lab subnet. (Bigger
        # nets need a netmask, which requires netifaces/psutil; we
        # avoid that dep here and let the operator override manually.)
        bc = ".".join([*parts[:3], "255"])
        if bc not in seen:
            seen.add(bc)
            out.append(bc)

    # Hostname resolution often returns every configured IPv4 address on
    # Windows, including a dedicated bench NIC that is not the default route.
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, family=socket.AF_INET):
            add_local_ip(info[4][0])
    except Exception:
        pass

    # Trick: connect a UDP socket to an arbitrary remote, then look at
    # our local addr. The OS picks the routing-table-preferred interface
    # for each remote, so probing a few representative gateways picks up
    # multi-interface hosts (Ethernet to the bench + WiFi to the office).
    probes = ("8.8.8.8", "192.168.0.1", "192.168.1.1", "10.0.0.1", "172.16.0.1")
    for probe in probes:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.1)
            s.connect((probe, 1))  # no actual packet sent
            local_ip = s.getsockname()[0]
            s.close()
        except Exception:
            continue
        add_local_ip(local_ip)

    # Always include the global broadcast as a last resort.
    if "255.255.255.255" not in seen:
        out.append("255.255.255.255")
    return out


def _hosts_for_broadcasts(broadcasts: list[str]) -> list[str]:
    """Expand directed /24 broadcast addresses into host candidates.

    This is used only as a fallback for Access2 boards that are reachable
    over TCP but do not answer the UDP enumerator. We deliberately ignore
    the all-nets broadcast because scanning all of 255.255.255.0 would be
    nonsense.
    """
    hosts: list[str] = []
    seen: set[str] = set()
    for bc in broadcasts:
        try:
            addr = ipaddress.ip_address(bc)
        except ValueError:
            continue
        if addr.version != 4 or bc == "255.255.255.255":
            continue
        parts = bc.split(".")
        if len(parts) != 4 or parts[-1] != "255":
            continue
        prefix = ".".join(parts[:3])
        for host in range(1, 255):
            ip = f"{prefix}.{host}"
            if ip not in seen:
                seen.add(ip)
                hosts.append(ip)
    return hosts


async def _probe_access2_tcp(
    ip: str,
    *,
    port: int,
    timeout_s: float,
) -> tuple[DiscoveredDevice, bool] | None:
    """Return an Access2 candidate if the TCP control port is open.

    The status probe is best-effort. Some live SNIIP/Access2 endpoints
    accept TCP on 7612 but close when sent the scaffolded status frame,
    so an open port is still useful operator-facing discovery data.
    """
    from vspin_cockpit.drivers.transports_access2_tcp import Access2Link

    link: Access2Link | None = None
    try:
        link = Access2Link(host=ip, port=port, timeout_s=timeout_s)
        await link.open()
        verified = False
        try:
            await link.get_status_short()
            verified = True
        except Exception:
            verified = False

        return (
            DiscoveredDevice(
                ip=ip,
                port=port,
                type="Access2" if verified else "Access2Candidate",
                display_name=TYPE_DISPLAY_MAP["Access2"] if verified else "Access2 candidate (TCP open)",
            ),
            verified,
        )
    except Exception:
        return None
    finally:
        if link is not None:
            try:
                await link.close()
            except Exception:
                pass


async def discover_access2_tcp_devices(
    *,
    timeout_s: float = 0.25,
    broadcast_address: str | None = None,
    port: int = 7612,
    candidate_hosts: list[str] | None = None,
) -> tuple[list[DiscoveredDevice], dict]:
    """Fallback Access2 discovery by probing TCP control ports.

    Some bench controllers are reachable at the Access2 TCP port but do
    not answer the Velocity11 UDP broadcast query. This probes candidate
    hosts with GET_STATUS_SHORT only; it does not home, move, or mutate
    hardware state.
    """
    broadcasts = [broadcast_address] if broadcast_address else _local_subnet_broadcasts()
    candidates = candidate_hosts if candidate_hosts is not None else _hosts_for_broadcasts(broadcasts)
    candidates = list(dict.fromkeys(candidates))
    semaphore = asyncio.Semaphore(64)

    async def probe(ip: str) -> tuple[DiscoveredDevice, bool] | None:
        async with semaphore:
            return await _probe_access2_tcp(ip, port=port, timeout_s=timeout_s)

    results = [dev for dev in await asyncio.gather(*(probe(ip) for ip in candidates)) if dev is not None]
    found = [dev for dev, _verified in results]
    found = sorted(
        found,
        key=lambda d: tuple(int(x) for x in d.ip.split(".")),
    )
    diag = {
        "scan_ranges": broadcasts,
        "probe_count": len(candidates),
        "open_count": len(found),
        "verified_count": sum(1 for _dev, verified in results if verified),
        "port": port,
    }
    return found, diag


async def discover_v11_devices(
    *, timeout_s: float = 2.0,
    bind_address: str = "0.0.0.0",
    broadcast_address: str | None = None,
) -> tuple[list[DiscoveredDevice], dict]:
    """Broadcast a v11 query and collect responses for `timeout_s` seconds.

    Uses `loop.create_datagram_endpoint`, which is the async-loop-agnostic
    UDP API (works under both stdlib asyncio AND uvloop).

    `broadcast_address` is an explicit override; pass it when the
    operator knows the right directed broadcast (e.g.,
    `192.168.0.255`). When left as None, the function auto-derives
    candidates via `_local_subnet_broadcasts()` and sends the query to
    every one.

    Returns `(devices, diag)` where `diag` contains the list of
    broadcast addresses tried and the response count per address — the
    UI surfaces these so the operator can debug subnet/firewall
    problems."""
    loop = asyncio.get_running_loop()

    broadcasts: list[str] = (
        [broadcast_address] if broadcast_address
        else _local_subnet_broadcasts()
    )

    try:
        transport, protocol = await loop.create_datagram_endpoint(
            _V11DiscoveryProtocol,
            local_addr=(bind_address, 0),
            allow_broadcast=True,
        )
    except OSError as exc:
        return [], {"broadcasts_tried": [], "error": f"socket open failed: {exc}"}

    sent_to: list[str] = []
    send_errors: dict[str, str] = {}
    try:
        for bc in broadcasts:
            try:
                transport.sendto(build_query(), (bc, V11_BROADCAST_PORT))
                sent_to.append(bc)
            except OSError as exc:
                send_errors[bc] = str(exc)
        await asyncio.sleep(timeout_s)
    finally:
        transport.close()

    discovered: dict[tuple[str, int], DiscoveredDevice] = {}
    for data, addr in protocol.received:
        dev = parse_response(data, addr[0])
        if dev is None:
            continue
        discovered.setdefault((dev.ip, dev.port), dev)
    devs = sorted(
        discovered.values(),
        key=lambda d: tuple(int(x) for x in d.ip.split(".")),
    )
    diag = {
        "broadcasts_tried": sent_to,
        "send_errors": send_errors,
        "response_count": len(protocol.received),
        "device_count": len(devs),
    }
    return devs, diag
