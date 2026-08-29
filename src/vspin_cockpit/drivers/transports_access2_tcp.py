"""Access2 plate-loader binary protocol over TCP.

The Agilent Access2 (the loader arm paired with the VSpin centrifuge)
exposes a binary command/response channel on TCP. Default port 7612
(SNIIP_DEFAULT_IP_PORT in the C++ source). Each command is framed as a
1-byte command ID, a little-endian 16-bit payload length, then the
fixed-layout argument block. Vendor captures for related Velocity11 TCP
boards use the same header on replies: response ID, length, payload.
Some earlier bring-up notes described replies as `[len][err][data]`, so
the reader below accepts both shapes for bench compatibility.

Constants were reconciled against the Studio 2008 vendor tester,
`Utilities/ActiveX Emulator/XML Command files/VSpinAccess.xml`, and
live read-only probes against an Access2 controller. The exact byte
layout per command is documented inline at the method level.

This is a Tier-B scaffold — the wire format is implemented from the
documented `AppendPacket()` call sites in
`Plugins/CentrifugeLoader/VSpinAccess2Ctrl.cpp`. Bench bring-up (Tier
C) will verify against real hardware and fill in any vendor quirks.
"""

from __future__ import annotations

import asyncio
import struct
from dataclasses import dataclass, field

from vspin_cockpit.core.transports import TransportError

# ---------------------------------------------------------------------------
# Command IDs (CI_*) — Access2Controller.h
# ---------------------------------------------------------------------------

CI_GET_FIRMWARE_VERSION   = 0x00
CI_GET_HARDWARE_VERSION   = 0x16  # "Transmit Board Version" in VSpinAccess XML.
CI_INITIALIZE             = 0x10
CI_CLOSE                  = 0x12
CI_PING                   = 0x14
CI_WRITE_FLASH            = 0x22
CI_READ_FLASH             = 0x24
CI_USE_FLASH              = 0x26
CI_FORMAT_FLASH           = 0x28
CI_RESET_ACCESS2_CB       = 0x30
CI_RESET_VSPIN_CB         = 0x32
CI_RESET_E_STOP           = 0x34
CI_SERVO_SWITCH           = 0x36
CI_HOME                   = 0x40
CI_JOG_AXIS               = 0x42
CI_MOVE_TO_LOCATION       = 0x44
CI_MOVE_TO_POSITION       = 0x46
CI_GET_STATUS             = 0x20
CI_GET_SENSOR_VALUES      = 0x50
CI_GET_STATUS_SHORT       = CI_GET_STATUS
CI_SUBSCRIBE_NMC_INFO     = 0x0A

# ---------------------------------------------------------------------------
# Axis addresses (AA_*) — single byte arg to motion commands.
# ---------------------------------------------------------------------------

AA_GRIPPER = 1
AA_Y       = 2
AA_Z       = 3

# ---------------------------------------------------------------------------
# Teachpoint indices (TI_*) — argument to MOVE_TO_LOCATION.
# ---------------------------------------------------------------------------

TI_PARK    = 0
TI_PICK    = 1  # plate handoff at staging position
TI_BUCKET_1 = 2
TI_BUCKET_2 = 3
TI_HOVER   = 4

# ---------------------------------------------------------------------------
# Motion profiles (PI_*) — affect acceleration/deceleration.
# ---------------------------------------------------------------------------

PI_STATIC        = 0
PI_HOMING        = 1
PI_DYNAMIC_EMPTY = 2
PI_DYNAMIC_FULL  = 3
PI_GRIP_NORMALLY = 4
PI_GRIP_GENTLY   = 5

# ---------------------------------------------------------------------------
# Speed indices (SI_*) — slow/medium/fast.
# ---------------------------------------------------------------------------

SI_SLOW   = 0
SI_MEDIUM = 1
SI_FAST   = 2

# ---------------------------------------------------------------------------
# Status bits — first byte of the status response. The full Access2 status
# payload is access2_status, vspin_status, then gripper/Y/Z status+position.
# ---------------------------------------------------------------------------

A2SB_INITIALIZED              = 0x01
A2SB_HOMED                    = 0x02
A2SB_ESTOP_SET                = 0x04
A2SB_ESTOP_ACTIVE             = 0x08
A2SB_ACCESS2_MOTOR_POWER_FAULT = 0x10
A2SB_OPTICAL_PLATE_SENSOR     = 0x20

# Per-command timeouts (seconds) — match T_* constants in the C++ source.
T_MOTION_COMMAND = 20.0
T_MOTIONLESS     = 5.0
T_HOME           = 60.0
POLL_HOME_S      = 1.0

DEFAULT_PORT = 7612
MAX_RESPONSE_PAYLOAD = 4096
ASYNC_EMPTY_RESPONSE_IDS = {0x0E}


# ---------------------------------------------------------------------------
# Frame helpers
# ---------------------------------------------------------------------------


def build_command(cmd_id: int, data: bytes = b"") -> bytes:
    """Marshal a single command frame: [cmd_id][len_le_u16][data]."""
    if not (0 <= cmd_id <= 0xFF):
        raise ValueError(f"command id 0x{cmd_id:X} out of byte range")
    if len(data) > 0xFFFF:
        raise ValueError("Access2 payload exceeds 64 KB")
    return bytes([cmd_id]) + struct.pack("<H", len(data)) + data


def parse_response_frame(buf: bytes) -> tuple[int, bytes]:
    """Unpack a vendor command-prefixed response frame.

    Format: `[response_id][len_le_u16][payload...]`.
    """
    if len(buf) < 3:
        raise TransportError(f"Access2 response frame too short: {buf!r}")
    response_id = buf[0]
    payload_len = struct.unpack("<H", buf[1:3])[0]
    payload = bytes(buf[3:])
    if len(payload) != payload_len:
        raise TransportError(
            f"Access2 response length mismatch: header={payload_len} actual={len(payload)}"
        )
    return response_id, payload


def parse_response(buf: bytes) -> tuple[int, bytes]:
    """Unpack [err_byte][response_bytes]. Raises TransportError if the
    error byte is non-zero. Caller is responsible for further decoding."""
    if not buf:
        raise TransportError("empty Access2 response")
    err = buf[0]
    data = bytes(buf[1:])
    if err != 0:
        raise TransportError(f"Access2 error 0x{err:02X} (response data={data!r})")
    return err, data


def _status_flags(access2_status: int, vspin_status: int = 0) -> dict:
    return {
        "access2_raw": access2_status,
        "initialized":       bool(access2_status & A2SB_INITIALIZED),
        "homed":             bool(access2_status & A2SB_HOMED),
        "estop_set":         bool(access2_status & A2SB_ESTOP_SET),
        "estop_active":      bool(access2_status & A2SB_ESTOP_ACTIVE),
        "motor_power_fault": bool(access2_status & A2SB_ACCESS2_MOTOR_POWER_FAULT),
        "optical_plate_sensor": bool(access2_status & A2SB_OPTICAL_PLATE_SENSOR),
        "vspin_raw":         vspin_status,
    }


def decode_status_short(data: bytes) -> dict:
    """Unpack the 4-byte GET_STATUS_SHORT payload.

    C++ `Access2Response::ParseLongResponse()` exposes the four status
    bytes after the command result. For callers that pass the complete
    response payload, tolerate and skip a leading success byte. Some
    Access2 firmware exposes only the full GET_STATUS frame; in that
    case the first two bytes after the success byte are the same
    Access2/VSpin status bytes and the remaining axis data is ignored.
    """
    if len(data) >= 5 and data[0] == 0:
        data = data[1:]
    if len(data) < 4:
        raise TransportError(f"GET_STATUS_SHORT payload too short: {data!r}")
    return _status_flags(data[0], data[1])


# ---------------------------------------------------------------------------
# Async TCP link
# ---------------------------------------------------------------------------


@dataclass
class Access2Link:
    """Async TCP client speaking the Access2 binary protocol.

    The loader board negotiates DHCP at boot with a static fallback;
    operators usually pin a static IP. For mDNS-discoverable units the
    `host` may be `<serial>.local` — DNS resolution falls through the
    standard asyncio path.
    """
    host: str
    port: int = DEFAULT_PORT
    timeout_s: float = T_MOTION_COMMAND
    _reader: asyncio.StreamReader | None = field(default=None, init=False)
    _writer: asyncio.StreamWriter | None = field(default=None, init=False)
    _lock: asyncio.Lock = field(default_factory=asyncio.Lock, init=False)

    @property
    def is_open(self) -> bool:
        return self._writer is not None and not self._writer.is_closing()

    async def open(self) -> None:
        if self.is_open:
            return
        if self._writer is not None or self._reader is not None:
            await self.close()
        try:
            self._reader, self._writer = await asyncio.wait_for(
                asyncio.open_connection(self.host, self.port),
                timeout=min(10.0, max(0.1, self.timeout_s)),
            )
        except Exception as exc:
            raise TransportError(
                f"could not connect to Access2 at {self.host}:{self.port}: {exc}"
            ) from exc

    async def close(self) -> None:
        if self._writer is not None:
            try:
                self._writer.close()
                await self._writer.wait_closed()
            except Exception:
                pass
        self._reader = None
        self._writer = None

    async def request(self, frame: bytes, *, timeout_s: float | None = None) -> tuple[int, bytes]:
        """Send a command frame and parse an error-coded response.

        Returns `(err_code, data_after_err)`. The underlying reader accepts
        both the vendor `[response_id][len][payload]` response shape and
        the earlier scaffold's legacy `[len][payload]` response shape.
        Returns (err_code, data). Serialized via internal lock."""
        _, payload = await self.request_payload(
            frame,
            timeout_s=timeout_s,
            expected_response_id=frame[0] if frame else None,
        )
        return parse_response(payload)

    async def request_payload(
        self,
        frame: bytes,
        *,
        timeout_s: float | None = None,
        expected_response_id: int | None = None,
    ) -> tuple[int | None, bytes]:
        """Send a command frame and return raw `(response_id, payload)`.

        `response_id` is `None` for legacy responses that omit the command
        byte. Use this for version/status helpers whose payload is not just
        a single success byte.
        """
        if not self.is_open:
            raise TransportError("Access2 link is not open")
        timeout = timeout_s if timeout_s is not None else self.timeout_s
        async with self._lock:
            try:
                self._writer.write(frame)  # type: ignore[union-attr]
                await self._writer.drain()  # type: ignore[union-attr]
                while True:
                    hdr = await asyncio.wait_for(
                        self._reader.readexactly(3),  # type: ignore[union-attr]
                        timeout=timeout,
                    )
                    vendor_len = struct.unpack("<H", hdr[1:3])[0]
                    if hdr[0] in ASYNC_EMPTY_RESPONSE_IDS and vendor_len == 0:
                        continue
                    break
                response_id: int | None
                legacy_len = struct.unpack("<H", hdr[:2])[0]
                expected_successors = set()
                if expected_response_id is not None:
                    expected_successors = {
                        expected_response_id & 0xFF,
                        (expected_response_id + 1) & 0xFF,
                        0x02,  # vendor error packet
                        0x66,  # procedure-done packet used by sibling V11 boards
                    }
                if (
                    expected_response_id is not None
                    and hdr[0] in expected_successors
                    and vendor_len <= MAX_RESPONSE_PAYLOAD
                ):
                    response_id = hdr[0]
                    payload_len = vendor_len
                    payload = await asyncio.wait_for(
                        self._reader.readexactly(payload_len),  # type: ignore[union-attr]
                        timeout=timeout,
                    )
                else:
                    if (
                        expected_response_id is None
                        and hdr[1] != 0
                        and vendor_len <= MAX_RESPONSE_PAYLOAD
                    ):
                        response_id = hdr[0]
                        payload = await asyncio.wait_for(
                            self._reader.readexactly(vendor_len),  # type: ignore[union-attr]
                            timeout=timeout,
                        )
                    elif 0 < legacy_len <= MAX_RESPONSE_PAYLOAD:
                        response_id = None
                        rest = await asyncio.wait_for(
                            self._reader.readexactly(legacy_len - 1),  # type: ignore[union-attr]
                            timeout=timeout,
                        )
                        payload = bytes([hdr[2]]) + rest
                    else:
                        response_id = hdr[0]
                        payload_len = vendor_len
                        payload = await asyncio.wait_for(
                            self._reader.readexactly(payload_len),  # type: ignore[union-attr]
                            timeout=timeout,
                        )
                await self._drain_unsolicited()
            except TimeoutError as exc:
                await self.close()
                raise TransportError(
                    f"Access2 read timed out after {timeout}s"
                ) from exc
            except (asyncio.IncompleteReadError, ConnectionError, OSError) as exc:
                await self.close()
                raise TransportError(f"Access2 connection lost: {exc}") from exc
        return response_id, payload

    async def _drain_unsolicited(self) -> None:
        """Discard short async frames that can follow diagnostic replies.

        Live Access2 firmware has been observed appending a zero-length
        0x0E frame after version reads. Leaving it queued corrupts the
        next request/response pair on a persistent socket.
        """
        if self._reader is None:
            return
        for _ in range(4):
            try:
                hdr = await asyncio.wait_for(self._reader.readexactly(3), timeout=0.02)
            except TimeoutError:
                return
            except (asyncio.IncompleteReadError, ConnectionError, OSError):
                return
            payload_len = struct.unpack("<H", hdr[1:3])[0]
            if payload_len > MAX_RESPONSE_PAYLOAD:
                return
            if payload_len:
                try:
                    await asyncio.wait_for(self._reader.readexactly(payload_len), timeout=0.02)
                except Exception:
                    return

    # ------------------------------------------------------------------
    # Per-command method wrappers (mirror the C++ Access2Controller API).
    # ------------------------------------------------------------------

    async def home(
        self, *,
        timeout_s: float = T_HOME,
        poll_read_timeout_s: float = T_MOTIONLESS,
    ) -> dict:
        ack = True
        warning: str | None = None
        try:
            await self.request(build_command(CI_HOME), timeout_s=timeout_s)
        except TransportError as exc:
            if "connection lost" not in str(exc):
                raise
            ack = False
            warning = str(exc)
            await self.close()
        status = await self.wait_until_homed(
            timeout_s=timeout_s,
            poll_read_timeout_s=poll_read_timeout_s,
        )
        status["home_acknowledged"] = ack
        if warning is not None:
            status["home_warning"] = warning
        return status

    async def wait_until_homed(
        self, *,
        timeout_s: float = T_HOME,
        poll_read_timeout_s: float = T_MOTIONLESS,
    ) -> dict:
        """Poll status until the firmware reports A2SB_HOMED.

        The vendor state machine treats Home as a controller-level
        operation. On real Access2 firmware, the low-level CI_HOME reply
        can arrive before regular status reads are responsive again, so
        a single 5-second status read is too brittle. Reconnect after a
        timed-out poll to discard late frames from the abandoned socket.
        """
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout_s
        last_status: dict | None = None
        last_error: Exception | None = None
        while True:
            remaining = deadline - loop.time()
            if remaining <= 0:
                detail = f"; last status={last_status}" if last_status else ""
                if last_error is not None:
                    detail += f"; last error={last_error}"
                raise TransportError(f"Access2 did not report homed within {timeout_s}s{detail}")
            try:
                if not self.is_open:
                    await self.open()
                status = await self.get_status_short(
                    timeout_s=min(poll_read_timeout_s, max(0.2, remaining))
                )
                last_status = status
                if bool(status.get("homed")):
                    return status
            except TransportError as exc:
                last_error = exc
            await asyncio.sleep(min(POLL_HOME_S, max(0.0, deadline - loop.time())))

    async def init(self) -> int:
        err, _ = await self.request(build_command(CI_INITIALIZE),
                                    timeout_s=T_HOME)
        return err

    async def controller_close(self) -> int:
        err, _ = await self.request(build_command(CI_CLOSE),
                                    timeout_s=T_MOTIONLESS)
        return err

    async def get_status_short(self, *, timeout_s: float | None = None) -> dict:
        status = await self.get_status(timeout_s=timeout_s)
        short = _status_flags(
            int(status.get("access2_raw", 0)),
            int(status.get("vspin_raw", 0)),
        )
        short["axis_status"] = {
            "y": status.get("y_status"),
            "z": status.get("z_status"),
            "gripper": status.get("gripper_status"),
        }
        short["axis_positions"] = {
            "y_mm": status.get("y_mm"),
            "z_mm": status.get("z_mm"),
            "gripper_mm": status.get("gripper_mm"),
        }
        return short

    async def get_status(self, *, timeout_s: float | None = None) -> dict:
        _, data = await self.request(build_command(CI_GET_STATUS),
                                     timeout_s=timeout_s or T_MOTIONLESS)
        if len(data) >= 17:
            return {
                "access2_raw": data[0],
                "vspin_raw": data[1],
                "gripper_status": data[2],
                "gripper_mm": struct.unpack("<f", data[3:7])[0],
                "y_status": data[7],
                "y_mm": struct.unpack("<f", data[8:12])[0],
                "z_status": data[12],
                "z_mm": struct.unpack("<f", data[13:17])[0],
                "raw": data.hex(),
            }
        short = decode_status_short(data)
        short["raw"] = data.hex()
        return short

    async def get_positions(self) -> dict:
        status = await self.get_status()
        return {
            "gripper_mm": status.get("gripper_mm"),
            "y_mm": status.get("y_mm"),
            "z_mm": status.get("z_mm"),
            "raw": status.get("raw"),
        }

    async def get_sensor_values(self) -> dict:
        _, data = await self.request(build_command(CI_GET_SENSOR_VALUES),
                                     timeout_s=T_MOTIONLESS)
        result: dict[str, int | str] = {"raw": data.hex()}
        if len(data) >= 4:
            result["sensor_values"] = struct.unpack("<L", data[:4])[0]
        return result

    async def ping(self, data: bytes = b"\x5A") -> bytes:
        _, echoed = await self.request(build_command(CI_PING, data),
                                       timeout_s=T_MOTIONLESS)
        return echoed

    async def get_firmware_version(self) -> str:
        _, payload = await self.request_payload(
            build_command(CI_GET_FIRMWARE_VERSION),
            timeout_s=T_MOTIONLESS,
            expected_response_id=CI_GET_FIRMWARE_VERSION,
        )
        _, data = parse_response(payload)
        return data.rstrip(b"\x00").decode("ascii", errors="replace")

    async def get_hardware_version(self) -> int | str:
        _, payload = await self.request_payload(
            build_command(CI_GET_HARDWARE_VERSION),
            timeout_s=T_MOTIONLESS,
            expected_response_id=CI_GET_HARDWARE_VERSION,
        )
        _, data = parse_response(payload)
        if len(data) >= 2:
            return struct.unpack("<h", data[:2])[0]
        return data.hex()

    async def move_to_location(
        self, *, location: int, z_offset_mm: float, plate_height_mm: float,
        profile: int = PI_DYNAMIC_EMPTY, speed: int = SI_SLOW,
    ) -> int:
        """Move the gripper centre to a teachpoint (TI_*) with z offset
        and plate-height clearance. Returns the new error byte (0 = ok)."""
        body = (
            bytes([location])
            + struct.pack("<f", z_offset_mm)
            + struct.pack("<f", plate_height_mm)
            + bytes([profile, speed])
        )
        err, _ = await self.request(build_command(CI_MOVE_TO_LOCATION, body),
                                    timeout_s=T_MOTION_COMMAND)
        return err

    async def move_to_position(
        self, *, axis: int, position_mm: float,
        profile: int = PI_DYNAMIC_EMPTY, speed: int = SI_SLOW,
    ) -> int:
        body = (
            bytes([axis])
            + struct.pack("<f", position_mm)
            + bytes([profile, speed])
        )
        err, _ = await self.request(build_command(CI_MOVE_TO_POSITION, body),
                                    timeout_s=T_MOTION_COMMAND)
        return err

    async def jog_axis(
        self, *, axis: int, displacement_mm: float,
        profile: int = PI_DYNAMIC_EMPTY, speed: int = SI_SLOW,
    ) -> int:
        body = (
            bytes([axis])
            + struct.pack("<f", displacement_mm)
            + bytes([profile, speed])
        )
        err, _ = await self.request(build_command(CI_JOG_AXIS, body),
                                    timeout_s=T_MOTION_COMMAND)
        return err

    async def servo_switch(self, *, axis: int, on: bool | None = None,
                           switch_code: int | None = None) -> int:
        if switch_code is None:
            switch_code = 1 if on else 0
        body = bytes([axis, switch_code & 0xFF])
        err, _ = await self.request(build_command(CI_SERVO_SWITCH, body),
                                    timeout_s=T_MOTIONLESS)
        return err

    async def reset_estop(self) -> int:
        err, _ = await self.request(build_command(CI_RESET_E_STOP),
                                    timeout_s=T_MOTIONLESS)
        return err

    async def reset_access2_breaker(self) -> int:
        err, _ = await self.request(build_command(CI_RESET_ACCESS2_CB),
                                    timeout_s=T_MOTIONLESS)
        return err

    async def reset_vspin_breaker(self) -> int:
        err, _ = await self.request(build_command(CI_RESET_VSPIN_CB),
                                    timeout_s=T_MOTIONLESS)
        return err

    async def read_flash(self, *, address: int, length: int) -> bytes:
        body = struct.pack("<HH", address & 0xFFFF, length & 0xFFFF)
        _, data = await self.request(build_command(CI_READ_FLASH, body),
                                     timeout_s=T_MOTIONLESS)
        return data

    async def write_flash(self, *, address: int, data: bytes, apply: bool = False) -> int:
        err = 0
        for offset, value in enumerate(data):
            body = struct.pack("<H", (address + offset) & 0xFFFF) + bytes([value])
            err, _ = await self.request(build_command(CI_WRITE_FLASH, body),
                                        timeout_s=T_MOTIONLESS)
            if err:
                return err
        if apply:
            err = await self.use_flash()
        return err

    async def use_flash(self) -> int:
        err, _ = await self.request(build_command(CI_USE_FLASH),
                                    timeout_s=T_MOTIONLESS)
        return err

    async def format_flash(self, *, block: int) -> int:
        err, _ = await self.request(build_command(CI_FORMAT_FLASH, bytes([block & 0xFF])),
                                    timeout_s=T_MOTIONLESS)
        return err

    async def subscribe_nmc_information(self, *, switch_code: int) -> int:
        body = (
            bytes([switch_code & 0xFF, 0])
            + struct.pack("<L", 0)
            + struct.pack("<H", DEFAULT_PORT)
            + struct.pack("<H", 0)
            + bytes([0])
        )
        err, _ = await self.request(build_command(CI_SUBSCRIBE_NMC_INFO, body),
                                    timeout_s=T_MOTIONLESS)
        return err

    async def raw_command(self, *, cmd_id: int, data: bytes = b"",
                          timeout_s: float | None = None) -> dict:
        response_id, payload = await self.request_payload(
            build_command(cmd_id, data),
            timeout_s=timeout_s,
            expected_response_id=cmd_id & 0xFF,
        )
        return {
            "response_id": response_id,
            "payload_hex": payload.hex(),
            "payload_len": len(payload),
        }
