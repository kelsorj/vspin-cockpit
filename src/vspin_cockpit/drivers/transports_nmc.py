"""NMC (Network Module Controller) binary protocol — Velocity11 PIC-SERVO
+ PIC-IO bus, used by the Agilent VSpin centrifuge and other Velocity11
instruments. Based on JR Kerr's NMC2.x specification.

Wire format (each command frame):
    +------+----+-----+--------+----------+
    | 0xAA | AD | CMD | DATA ... | CHECKSUM |
    +------+----+-----+--------+----------+
where:
  * 0xAA   - fixed sync header
  * AD     - module address (1-32, 0 while assigning addresses, or 0xFF group)
  * CMD    - high nibble = data length N, low nibble = command code
  * DATA   - N bytes of command-specific arguments
  * CHK    - low byte of (AD + CMD + DATA bytes)

Response frame:
    +-----------+----------+---+
    | STATUS    | DATA ... | CHK |
    +-----------+----------+---+
The STATUS byte's "Define Status" mask (set by DEFINE_STATUS at init time)
determines how many DATA bytes follow. The response checksum is the low
byte of (STATUS + DATA).

Bus connection: RS-485 multi-drop. Vendor init probes 19200, 115200,
57600, and 9600 baud, then switches the VSpin network to 57600 baud.

Implementation notes:
  * `NmcLink` extends asyncio framing over the SerialLink — it does NOT
    use line-terminated `read_line`; instead it reads a fixed number of
    bytes per response (governed by the active status mask).
  * Module addresses must be assigned via the SET_ADDR command at boot
    after a hardware-reset broadcast (address 0 → module address-N).
  * The VSpin uses two NMC modules: a PIC-SERVO (address 1) for the
    rotor and a PIC-IO (address 2) for door/lock sensors.

This is a Tier-B implementation — it scaffolds the wire protocol with
constants ported from the C++ plugin source at
`/Users/kelsorj/GitHub/Great-Conversion/Plugins/Centrifuge/VSpin v3/
SynchronizedPic.h`. The framing math is correct per the public NMC2
spec; bench bring-up will verify the byte-for-byte behaviour against
real hardware.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Any

from vspin_cockpit.core.transports import TransportError

# ---------------------------------------------------------------------------
# Wire constants
# ---------------------------------------------------------------------------

SYNC_BYTE = 0xAA
SERVOMODTYPE = 0
IOMODTYPE = 2
_BAUD_CODES = {19200: 63, 57600: 20, 115200: 10}
_INIT_BAUDRATES = (19200, 115200, 57600, 9600)

# NMC command codes are bare 4-bit values (0..15). Vendor CPicServo
# encodes CMD as high nibble = data length and low nibble = command.
# Some codes are reused across PIC-SERVO and PIC-IO modules; each
# module's docs interpret them differently.
CMD_RESET_POS         = 0x0
CMD_SET_IO_DIR        = 0x0   # PIC-IO: 2-byte direction mask
CMD_SET_ADDR          = 0x1
CMD_DEFINE_STATUS     = 0x2
CMD_READ_STATUS       = 0x3
CMD_LOAD_TRAJECTORY   = 0x4   # PIC-SERVO move
CMD_START_MOTION      = 0x5
CMD_SET_OUTPUT        = 0x6   # PIC-IO: 2-byte output mask
CMD_SET_GAIN          = 0x6
CMD_STOP_MOTOR        = 0x7
CMD_IO_CTRL           = 0x8   # PIC-SERVO limit-output control
CMD_SET_HOMING        = 0x9   # PIC-SERVO homing modes
CMD_SET_BAUD          = 0xA
CMD_CLEAR_BITS        = 0xB
CMD_NOOP              = 0xE
CMD_HARD_RESET        = 0xF

# PIC-SERVO LOAD_TRAJECTORY mode bits.
LOAD_POS              = 0x01
LOAD_VEL              = 0x02
LOAD_ACC              = 0x04
LOAD_PWM              = 0x08
ENABLE_SERVO          = 0x10
VEL_MODE              = 0x20
START_NOW             = 0x80

# PIC-SERVO STOP_MOTOR mode bits.
AMP_ENABLE            = 0x01
MOTOR_OFF             = 0x02
STOP_ABRUPT           = 0x04
STOP_SMOOTH           = 0x08
STOP_HERE             = 0x10

# Status mask bits used by DEFINE_STATUS / READ_STATUS responses.
SEND_POS              = 0x01
SEND_AD               = 0x02
SEND_VEL              = 0x04
SEND_AUX              = 0x08
SEND_HOME             = 0x10
SEND_ID               = 0x20
SEND_PERROR           = 0x40
SEND_OUT              = SEND_PERROR  # backwards-compatible alias
SEND_NPOINTS          = 0x80

# PIC-IO status mask bits.
SEND_INPUTS           = 0x01  # 2-byte input word
SEND_AD1              = 0x02
SEND_AD2              = 0x04
SEND_AD3              = 0x08
SEND_TIMER            = 0x10
SEND_SYNC_IN          = 0x40
SEND_SYNC_TMR         = 0x80

# Status byte bits.
STAT_MOVE_DONE        = 0x01
STAT_CKSUM_ERR        = 0x02
STAT_OVERCURRENT      = 0x04
STAT_POW_ON           = 0x08
STAT_POS_ERR          = 0x10
STAT_LIMIT1           = 0x20
STAT_LIMIT2           = 0x40
STAT_HOME_IN_PROG     = 0x80

# Homing modes (PIC-SERVO `SET_HOMING` data byte).
HOME_ON_LIMIT1        = 0x01
HOME_ON_LIMIT2        = 0x02
HOME_ON_HOME          = 0x04
HOME_ON_INDEX         = 0x08
HOME_STOP_ABRUPT      = 0x10
HOME_STOP_SMOOTH      = 0x20
ON_INDEX              = HOME_ON_INDEX  # alias matching the C++ symbol name

# Default module addresses for the VSpin (assigned at boot via SET_ADDR).
AN_ADDR_PIC_SERVO     = 0x01
AN_ADDR_PIC_IO        = 0x02

# PIC-IO bit indices for VSpin door/bucket sensors and actuators.
IOI_IN_AMP_FAULT      = 0
IOI_IN_SPINNING       = 1
IOI_IN_IMBALANCE      = 2
IOI_IN_BUCKET_UNLOCKED = 3
IOI_IN_BUCKET_LOCKED  = 4
IOI_IN_DOOR_OPEN      = 6
IOI_IN_DOOR_LOCKED    = 7
IOI_IN_AMP_ENABLED    = 11

IOI_OUT_VERSION_TOGGLE = 5
IOI_OUT_BUCKET_LOCK_CYL = 8  # engage / release bucket lock
IOI_OUT_DOOR_CYL      = 9   # open / close door cylinder
IOI_OUT_DOOR_LOCK_CYL = 10  # engage / release door lock


# ---------------------------------------------------------------------------
# Framing
# ---------------------------------------------------------------------------


def build_command(address: int, command_code: int, data: bytes = b"") -> bytes:
    """Marshal an NMC command frame. `command_code` is the 4-bit command
    number (e.g., CMD_NOOP = 0xE means command 14). The framer shifts
    data length into the high nibble of the CMD byte and ORs in the
    command code in the low nibble."""
    if not (0 <= address <= 32 or address == 0xFF):
        raise ValueError(f"address {address} out of NMC range 0..32 or 0xFF")
    if not (0 <= len(data) <= 15):
        raise ValueError(f"data length {len(data)} exceeds NMC nibble (0..15)")
    if not (0 <= command_code <= 0xF):
        raise ValueError(f"command_code 0x{command_code:X} must fit in a 4-bit nibble")
    cmd_byte = ((len(data) & 0x0F) << 4) | (command_code & 0x0F)
    body = bytes([address, cmd_byte]) + data
    checksum = sum(body) & 0xFF
    return bytes([SYNC_BYTE]) + body + bytes([checksum])


def parse_response(buf: bytes, expected_data_len: int) -> tuple[int, bytes]:
    """Unpack an NMC response frame. Returns (status_byte, data_bytes).
    Raises TransportError on checksum mismatch or short frame.

    `expected_data_len` is computed from the DEFINE_STATUS mask the
    operator most recently sent; the response is fixed-length under
    that mask, so callers must track it. (See NmcLink.define_status.)"""
    needed = 1 + expected_data_len + 1  # status + data + checksum
    if len(buf) < needed:
        raise TransportError(f"NMC response too short: got {len(buf)} bytes, need {needed}")
    status = buf[0]
    data = bytes(buf[1:1 + expected_data_len])
    cksum = buf[1 + expected_data_len]
    expected = (status + sum(data)) & 0xFF
    if cksum != expected:
        raise TransportError(
            f"NMC checksum failure: got 0x{cksum:02X}, expected 0x{expected:02X}"
        )
    if status & STAT_CKSUM_ERR:
        raise TransportError(f"NMC command rejected with checksum status 0x{status:02X}")
    return status, data


# ---------------------------------------------------------------------------
# High-level link
# ---------------------------------------------------------------------------


@dataclass
class NmcLink:
    """Async wrapper around a serial port speaking NMC frames.

    Drivers create one NmcLink per RS-485 bus and address it to different
    modules per call. Each module has its own active status mask, so
    `request()` tracks response lengths by address.
    """
    port: str
    baudrate: int = 57600
    timeout_s: float = 5.0
    # Fallback payload length before DEFINE_STATUS has been sent for an
    # address. Startup/reset replies carry no status data.
    status_data_len: int = 0
    _reader: asyncio.StreamReader | None = field(default=None, init=False)
    _writer: asyncio.StreamWriter | None = field(default=None, init=False)
    _lock: asyncio.Lock = field(default_factory=asyncio.Lock, init=False)
    _status_data_len_by_addr: dict[int, int] = field(default_factory=dict, init=False)
    _status_mask_by_addr: dict[int, int] = field(default_factory=dict, init=False)
    _io_outbits_by_addr: dict[int, int] = field(default_factory=dict, init=False)
    _io_bitdirs_by_addr: dict[int, int] = field(default_factory=dict, init=False)

    @property
    def is_open(self) -> bool:
        return self._writer is not None and not self._writer.is_closing()

    def _clear_cached_module_state(self) -> None:
        self._status_data_len_by_addr.clear()
        self._status_mask_by_addr.clear()
        self._io_outbits_by_addr.clear()
        self._io_bitdirs_by_addr.clear()

    def _clear_input_buffer(self) -> None:
        if self._writer is None:
            return
        serial = None
        get_extra_info = getattr(self._writer, "get_extra_info", None)
        if get_extra_info is not None:
            serial = get_extra_info("serial")
        if serial is not None and hasattr(serial, "reset_input_buffer"):
            try:
                serial.reset_input_buffer()
            except Exception:
                pass

    async def open(self) -> None:
        if self.is_open:
            return
        try:
            import serial_asyncio  # type: ignore[import-untyped]
        except ImportError as exc:  # pragma: no cover - real hardware only
            raise TransportError(
                "pyserial-asyncio not installed; pip install 'vspin-cockpit[hardware]'"
            ) from exc
        try:
            self._reader, self._writer = await serial_asyncio.open_serial_connection(
                url=self.port, baudrate=self.baudrate,
                bytesize=8, parity="N", stopbits=1,
                xonxoff=False, rtscts=False, dsrdtr=False,
            )
            serial = self._writer.get_extra_info("serial")
            if serial is not None:
                serial.dtr = False
                serial.rts = False
        except Exception as exc:
            raise TransportError(f"could not open {self.port}: {exc}") from exc

    async def close(self) -> None:
        if self._writer is not None:
            try:
                self._writer.close()
                await self._writer.wait_closed()
            except Exception:
                pass
        self._reader = None
        self._writer = None

    async def reopen(self, baudrate: int) -> None:
        await self.close()
        self.baudrate = int(baudrate)
        await self.open()

    async def _write_no_response(self, frame: bytes) -> None:
        if not self.is_open:
            raise TransportError("NMC link is not open")
        async with self._lock:
            self._clear_input_buffer()
            self._writer.write(frame)  # type: ignore[union-attr]
            await self._writer.drain()  # type: ignore[union-attr]

    def _response_data_len(self, addr: int) -> int:
        return self._status_data_len_by_addr.get(addr, self.status_data_len)

    async def request(
        self, frame: bytes, *, expected_data_len: int | None = None,
    ) -> tuple[int, bytes]:
        """Send a framed command, read one response, return (status, data).
        Serialized via internal lock so concurrent callers don't interleave
        on the RS-485 bus."""
        if not self.is_open:
            raise TransportError("NMC link is not open")
        if len(frame) < 4 or frame[0] != SYNC_BYTE:
            raise TransportError("NMC request frame is malformed")
        addr = frame[1]
        cmd = frame[2]
        data_len = self._response_data_len(addr) if expected_data_len is None else expected_data_len
        async with self._lock:
            self._clear_input_buffer()
            self._writer.write(frame)  # type: ignore[union-attr]
            await self._writer.drain()  # type: ignore[union-attr]
            try:
                raw = await asyncio.wait_for(
                    self._reader.readexactly(1 + data_len + 1),  # type: ignore[union-attr]
                    timeout=self.timeout_s,
                )
            except asyncio.TimeoutError as exc:
                raise TransportError(
                    f"NMC read timed out after {self.timeout_s}s "
                    f"(addr={addr}, cmd=0x{cmd:02X}, data_len={data_len})"
                ) from exc
        return parse_response(raw, data_len)

    async def hard_reset(self) -> None:
        """Vendor NmcHardReset: flush, prod every address with NOP, then
        broadcast HARD_RESET. Hardware returns to 19200 baud after reset."""
        self._clear_cached_module_state()
        await self._write_no_response(b"\x00" * 20)
        for addr in range(33):
            await self._write_no_response(build_command(addr, CMD_NOOP) + (b"\x00" * 8))
        await self._write_no_response(build_command(0xFF, CMD_HARD_RESET))
        await asyncio.sleep(0.1)

    async def set_addr_from_default(self, addr: int, groupaddr: int = 0xFF) -> int:
        status, _ = await self.request(
            build_command(0, CMD_SET_ADDR, bytes([addr & 0xFF, groupaddr & 0xFF])),
            expected_data_len=0,
        )
        return status

    async def read_module_id(self, addr: int) -> tuple[int, int]:
        _, data = await self.request(
            build_command(addr, CMD_READ_STATUS, bytes([SEND_ID])),
            expected_data_len=2,
        )
        if len(data) != 2:
            raise TransportError("NMC module ID response was malformed")
        return data[0], data[1]

    async def change_baud(self, baudrate: int) -> None:
        baudrate = int(baudrate)
        try:
            code = _BAUD_CODES[baudrate]
        except KeyError as exc:
            raise TransportError(f"NMC baud rate {baudrate} is not supported") from exc
        await self._write_no_response(build_command(0xFF, CMD_SET_BAUD, bytes([code])))
        await asyncio.sleep(0.1)
        await self.reopen(baudrate)
        await asyncio.sleep(0.1)
        self._clear_input_buffer()

    async def init_network(self, *, target_baudrate: int | None = None) -> dict[int, tuple[int, int]]:
        """Port of vendor CPicServo::NmcInit.

        It tries the baud rates the vendor DLL probes, hard-resets the
        modules, assigns sequential addresses, reads module type/version,
        then switches the network to the requested final baud rate.
        """
        target = int(target_baudrate or self.baudrate)
        previous_timeout = self.timeout_s
        last_error: Exception | None = None
        for initial_baud in _INIT_BAUDRATES:
            modules: dict[int, tuple[int, int]] = {}
            try:
                await self.reopen(initial_baud)
                await self.hard_reset()
                await self.reopen(19200)
                await self.hard_reset()
                # serial_asyncio may have already buffered status replies
                # from reset-time NOPs; reopen to discard that reader state
                # before the first address-assignment response.
                await self.reopen(19200)
                self.timeout_s = min(previous_timeout, 0.75)
                for addr in range(1, 33):
                    try:
                        await self.set_addr_from_default(addr)
                    except TransportError:
                        break
                    modules[addr] = await self.read_module_id(addr)
                self.timeout_s = previous_timeout
                if modules:
                    await self.change_baud(target)
                    return modules
            except Exception as exc:
                last_error = exc
            finally:
                self.timeout_s = previous_timeout
        if last_error is not None:
            raise TransportError(f"NMC init failed: {last_error}") from last_error
        raise TransportError("NMC init found no modules")

    # ------------------------------------------------------------------
    # PIC-SERVO + PIC-IO method-style wrappers (mirrors the C++ class
    # interface so the VSpinDriver can keep the same call sites used in
    # the original plugin).
    # ------------------------------------------------------------------

    async def nmc_no_op(self, addr: int) -> int:
        status, _ = await self.request(build_command(addr, CMD_NOOP, b""))
        return status

    async def nmc_get_stat(self, addr: int) -> int:
        status, _ = await self.request(build_command(addr, CMD_NOOP, b""))
        return status

    async def define_status(self, addr: int, mask: int) -> int:
        # Vendor CPicServo updates the module status mask before reading
        # this command's reply, so the DEFINE_STATUS response already has
        # the newly requested payload shape.
        data_len = _mask_to_data_len(mask, addr=addr)
        status, _ = await self.request(
            build_command(addr, CMD_DEFINE_STATUS, bytes([mask])),
            expected_data_len=data_len,
        )
        self._status_data_len_by_addr[addr] = data_len
        self._status_mask_by_addr[addr] = mask
        return status

    async def servo_set_homing(self, addr: int, mode: int) -> int:
        status, _ = await self.request(build_command(addr, CMD_SET_HOMING,
                                                     bytes([mode])))
        return status

    async def servo_get_position(self, addr: int) -> int:
        # NMC encodes position as a signed 32-bit little-endian field.
        # We rely on the active DEFINE_STATUS mask including SEND_POS so
        # the first 4 data bytes of the response are the position.
        status, data = await self.request(build_command(addr, CMD_NOOP, b""))
        if len(data) < 4:
            raise TransportError(
                "servo_get_position requires DEFINE_STATUS mask with SEND_POS"
            )
        return int.from_bytes(data[:4], "little", signed=True)

    async def servo_get_home(self, addr: int) -> int:
        # Returns the position captured on the last homing index pulse.
        # Requires SEND_HOME in the active status mask.
        _, data = await self.request(build_command(addr, CMD_NOOP, b""))
        mask = self._status_mask_by_addr.get(addr, 0)
        offset = _mask_field_offset(mask, SEND_HOME, addr=addr)
        if offset is None or len(data) < offset + 4:
            raise TransportError(
                "servo_get_home requires DEFINE_STATUS mask with SEND_HOME"
            )
        return int.from_bytes(data[offset:offset + 4], "little", signed=True)

    async def servo_get_velocity(self, addr: int) -> int:
        """Return the signed 16-bit velocity field from the active status
        payload. Requires DEFINE_STATUS to include SEND_VEL."""
        _, data = await self.request(build_command(addr, CMD_NOOP, b""))
        mask = self._status_mask_by_addr.get(addr, 0)
        offset = _mask_field_offset(mask, SEND_VEL, addr=addr)
        if offset is None or len(data) < offset + 2:
            raise TransportError(
                "servo_get_velocity requires DEFINE_STATUS mask with SEND_VEL"
            )
        return int.from_bytes(data[offset:offset + 2], "little", signed=True)

    async def servo_get_aux(self, addr: int) -> int:
        """Return the one-byte auxiliary status field. Requires SEND_AUX."""
        _, data = await self.request(build_command(addr, CMD_NOOP, b""))
        mask = self._status_mask_by_addr.get(addr, 0)
        offset = _mask_field_offset(mask, SEND_AUX, addr=addr)
        if offset is None or len(data) < offset + 1:
            raise TransportError(
                "servo_get_aux requires DEFINE_STATUS mask with SEND_AUX"
            )
        return data[offset]

    async def servo_set_gain(
        self,
        addr: int,
        *,
        kp: int,
        kd: int,
        ki: int,
        int_limit: int,
        out_limit: int,
        cur_limit: int,
        position_error_limit: int,
        servo_rate: int,
        deadband: int,
    ) -> int:
        """PIC-SERVO SET_GAIN. The vendor packet is:
        kp, kd, ki, int_limit, out_limit, cur_limit,
        position_error_limit, servo_rate, deadband."""
        body = (
            int(kp).to_bytes(2, "little", signed=True)
            + int(kd).to_bytes(2, "little", signed=True)
            + int(ki).to_bytes(2, "little", signed=True)
            + int(int_limit).to_bytes(2, "little", signed=True)
            + bytes([int(out_limit) & 0xFF, int(cur_limit) & 0xFF])
            + int(position_error_limit).to_bytes(2, "little", signed=True)
            + bytes([int(servo_rate) & 0xFF, int(deadband) & 0xFF])
        )
        status, _ = await self.request(build_command(addr, CMD_SET_GAIN, body))
        return status

    async def servo_load_trajectory(
        self,
        addr: int,
        *,
        mode: int,
        position: int = 0,
        velocity: int = 0,
        accel: int = 0,
        pwm: int = 0,
    ) -> int:
        """PIC-SERVO LOAD_TRAJECTORY. Fields are present only when their
        corresponding LOAD_* bit is set in `mode`, matching ServoLoadTraj."""
        body = bytearray([mode & 0xFF])
        if mode & LOAD_POS:
            body.extend(int(position).to_bytes(4, "little", signed=True))
        if mode & LOAD_VEL:
            body.extend(int(velocity).to_bytes(4, "little", signed=False))
        if mode & LOAD_ACC:
            body.extend(int(accel).to_bytes(4, "little", signed=False))
        if mode & LOAD_PWM:
            body.append(int(pwm) & 0xFF)
        status, _ = await self.request(
            build_command(addr, CMD_LOAD_TRAJECTORY, bytes(body))
        )
        return status

    async def servo_stop_motor(self, addr: int, mode: int) -> int:
        status, _ = await self.request(build_command(
            addr, CMD_STOP_MOTOR, bytes([mode & 0xFF]),
        ))
        return status

    async def servo_clear_bits(self, addr: int) -> int:
        status, _ = await self.request(build_command(addr, CMD_CLEAR_BITS, b""))
        return status

    async def servo_reset_pos(self, addr: int) -> int:
        status, _ = await self.request(build_command(addr, CMD_RESET_POS, b""))
        return status

    async def servo_go_to_position(
        self, addr: int, position: int, velocity: int = 100_000, accel: int = 10_000,
    ) -> int:
        """LOAD_TRAJECTORY (pos+vel+acc, servo enabled, start now)."""
        return await self.servo_load_trajectory(
            addr,
            mode=LOAD_POS | LOAD_VEL | LOAD_ACC | ENABLE_SERVO | START_NOW,
            position=position,
            velocity=velocity,
            accel=accel,
        )

    async def io_set_out_bit(self, addr: int, bit: int) -> int:
        """PIC-IO: set output bit `bit` to 1."""
        outbits = self._io_outbits_by_addr.get(addr, 0) | (1 << bit)
        self._io_outbits_by_addr[addr] = outbits
        status, _ = await self.request(build_command(
            addr, CMD_SET_OUTPUT, outbits.to_bytes(2, "little", signed=False),
        ))
        return status

    async def io_clr_out_bit(self, addr: int, bit: int) -> int:
        """PIC-IO: clear output bit `bit` (set to 0)."""
        outbits = self._io_outbits_by_addr.get(addr, 0) & ~(1 << bit)
        self._io_outbits_by_addr[addr] = outbits
        status, _ = await self.request(build_command(
            addr, CMD_SET_OUTPUT, outbits.to_bytes(2, "little", signed=False),
        ))
        return status

    async def io_bit_dir_in(self, addr: int, bit: int) -> int:
        """PIC-IO: set bit direction to input."""
        bitdirs = self._io_bitdirs_by_addr.get(addr, 0x0FFF) | (1 << bit)
        self._io_bitdirs_by_addr[addr] = bitdirs
        status, _ = await self.request(build_command(
            addr, CMD_SET_IO_DIR, bitdirs.to_bytes(2, "little", signed=False),
        ))
        return status

    async def io_bit_dir_out(self, addr: int, bit: int) -> int:
        """PIC-IO: set bit direction to output."""
        bitdirs = self._io_bitdirs_by_addr.get(addr, 0x0FFF) & ~(1 << bit)
        self._io_bitdirs_by_addr[addr] = bitdirs
        status, _ = await self.request(build_command(
            addr, CMD_SET_IO_DIR, bitdirs.to_bytes(2, "little", signed=False),
        ))
        return status

    async def io_in_bit_val(self, addr: int, bit: int) -> bool:
        """PIC-IO: read input bit `bit`. Requires DEFINE_STATUS includes
        SEND_INPUTS so the response carries the input word."""
        _, data = await self.request(build_command(addr, CMD_NOOP, b""))
        mask = self._status_mask_by_addr.get(addr, 0)
        offset = _mask_field_offset(mask, SEND_INPUTS, addr=addr)
        if offset is None or len(data) < offset + 2:
            raise TransportError(
                "io_in_bit_val requires DEFINE_STATUS mask with SEND_INPUTS"
            )
        inbits = int.from_bytes(data[offset:offset + 2], "little", signed=False)
        return bool((inbits >> bit) & 0x1)


_SERVO_STATUS_FIELDS = (
    (SEND_POS, 4),
    (SEND_AD, 1),
    (SEND_VEL, 2),
    (SEND_AUX, 1),
    (SEND_HOME, 4),
    (SEND_ID, 2),
    (SEND_PERROR, 2),
    (SEND_NPOINTS, 1),
)

_IO_STATUS_FIELDS = (
    (SEND_INPUTS, 2),
    (SEND_AD1, 1),
    (SEND_AD2, 1),
    (SEND_AD3, 1),
    (SEND_TIMER, 4),
    (SEND_ID, 2),
    (SEND_SYNC_IN, 2),
    (SEND_SYNC_TMR, 4),
)


def _status_fields(addr: int | None) -> tuple[tuple[int, int], ...]:
    return _IO_STATUS_FIELDS if addr == AN_ADDR_PIC_IO else _SERVO_STATUS_FIELDS


def _mask_to_data_len(mask: int, *, addr: int | None = None) -> int:
    """Total data-byte length for a given DEFINE_STATUS mask."""
    return sum(n for bit, n in _status_fields(addr) if mask & bit)


def _mask_field_offset(mask: int, bit: int, *, addr: int | None = None) -> int | None:
    offset = 0
    for field_bit, n in _status_fields(addr):
        if field_bit == bit:
            return offset if mask & bit else None
        if mask & field_bit:
            offset += n
    return None
