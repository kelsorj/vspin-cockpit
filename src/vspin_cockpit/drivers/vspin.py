"""Agilent VSpin centrifuge + Access2 plate-loader drivers.

This module hosts FOUR driver classes that compose into two real-world
units:

  * `VSpinBase` / `SimVSpin` / `VSpinDriver` — the centrifuge itself. The
    real wire protocol is Velocity11 PIC Servo NMC binary over RS-232 at
    115200 8N1 (see `drivers/transports/nmc.py` for the framer). The
    diagnostic dialog screenshots at `diags/vspin_controls.png` and
    `diags/vspin_profile.png` are the full surface this driver exposes.

  * `Access2Base` / `SimAccess2` / `Access2Driver` — the 3-axis loader
    arm (Y / Z / gripper) that picks plates from a staging position and
    drops them into either bucket of the paired centrifuge. The real
    wire protocol is binary packet framing on TCP port 7612 (see
    `drivers/transports/access2_tcp.py`). Diagnostic screenshots at
    `diags/vspin_autoloader_{control,move,settings,profile}.png`.

The `[pair]` block on each catalog descriptor causes the system builder
to auto-add the partner device and link them bidirectionally via
`Device.peer_device_id`. The loader driver looks up its peer at
command-dispatch time via `Scheduler.get_peer(device)` to forward
`spin` commands to the centrifuge during `complete_cycle`.

The simulators emit `kinematic_state` events keyed by URDF joint name
(`dof_rotor`, `dof_y_axis`, `dof_z_axis`, `dof_left_finger`,
`dof_right_finger`) so the 3D URDF viewer animates the rotor and the
loader arm in real time during simulation.

Sources:
  - Original Velocity11 / Agilent C++ plugin (VSpinAccess2Ctrl.cpp/h,
    VSpinCtrl.h, VSpinAccess2StateMachine.cpp/h). All CI_* / IOI_* /
    A2SB_* / TI_* / PI_* / SI_* constants ported from there.
  - Velocity11 PIC Servo NMC documentation (see transports_nmc.py).
"""

from __future__ import annotations

import asyncio
import math
import time
from copy import deepcopy
from typing import Any

from vspin_cockpit.core.driver import Command, CommandResult, Driver, DriverState

# ---------------------------------------------------------------------------
# VSpin centrifuge — base + sim + real
# ---------------------------------------------------------------------------


_VSPIN_COUNTS_PER_REV = 8000
_VSPIN_MAX_ACCEL = 916.19328
_VSPIN_VELOCITY_CONSTANT = 4473.925
_VSPIN_DEFAULT_MAX_VELOCITY_RPM = 3000.0
_VSPIN_SPIN_OVERSHOOT_MS = 5000
_VSPIN_HOMING_GAIN_OUT_LIMIT = 50
_VSPIN_HOMING_GAIN_POSITION_ERROR_LIMIT = 1000
_VSPIN_HOMING_VEL_RPM = 7.5
_VSPIN_HOMING_ACCEL_PERCENT = 30.0
_VSPIN_TIMEOUT_HOMING_S = 15.0
_VSPIN_DOOR_VEL_RPM = 120.0
_VSPIN_DOOR_ACCEL_PERCENT = 30.0
_VSPIN_BUCKET_TOLERANCE_TICKS = 5
_VSPIN_BUCKET_PRESENT_RETRIES = 5
_VSPIN_TIMEOUT_SERVO_TO_DOOR_S = 15.0
_VSPIN_DEFAULT_POSITION_GAINS = {
    "kp": 200,
    "kd": 1200,
    "ki": 150,
    "int_limit": 15,
    "out_limit": 75,
    "cur_limit": 0,
    "position_error_limit": 4000,
    "servo_rate": 5,
    "deadband": 0,
}
_VSPIN_DEFAULT_VELOCITY_GAINS = {
    "kp": 5,
    "kd": 100,
    "ki": 0,
    "int_limit": 0,
    "out_limit": 253,
    "cur_limit": 0,
    "position_error_limit": 16000,
    "servo_rate": 1,
    "deadband": 0,
}


class VSpinBase(Driver):
    """Agilent VSpin centrifuge — bare unit (no loader)."""

    role = "centrifuge"
    DEFAULT_RAMP_S = 6.0  # accel and decel each take ~6s
    BUCKET_TEACH_DEFAULTS = {1: 1353, 2: 5349}  # from vspin_profile.png

    # Mirrors the Agilent Centrifuge Diagnostics dialog — every button,
    # checkbox, and field on the Controls + Profiles tabs.
    supported_commands = {
        # ---- spin (the main routine) -----------------------------------
        "spin": {
            "group": "Control",
            "description": "Spin at RCF (or RPM) for the given duration with ramp-up/down.",
            "params": {
                "rcf": {"type": "float", "default": 251.55,
                        "description": "Relative centrifugal force (×g). Use rpm instead by setting it explicitly."},
                "rpm": {"type": "float", "default": 1500.0,
                        "description": "Spin speed in RPM. Ignored when rcf is provided."},
                "duration_s": {"type": "float", "default": 10.0, "required": True,
                               "description": "Time at speed (or total time — see time_mode)."},
                "accel_pct": {"type": "int", "default": 80,
                              "description": "Acceleration as % of max (1–100)."},
                "decel_pct": {"type": "int", "default": 80,
                              "description": "Deceleration / braking as % of max (1–100)."},
                "time_mode": {"type": "choice", "default": "time_at_speed",
                              "choices": ["time_at_speed", "total_time"],
                              "description": "Whether duration_s counts only steady-state, or includes ramps."},
                "rotor_radius_mm": {"type": "float", "default": 100.0,
                                    "description": "Used to derive RPM from RCF and vice-versa."},
            },
            "dangerous": True,
        },
        "stop_spin": {
            "group": "Control",
            "description": "Abort an active spin cycle and command the rotor to stop.",
            "params": {"smooth": {"type": "bool", "default": True}},
            "dangerous": True,
        },
        # ---- door + bucket -------------------------------------------
        "open_door":     {"group": "Control", "description": "Open the centrifuge lid.",
                          "params": {}, "dangerous": True},
        "close_door":    {"group": "Control", "description": "Close the centrifuge lid.",
                          "params": {}, "dangerous": True},
        "lock_door":     {"group": "Control", "description": "Engage the door-lock cylinder.",
                          "params": {}},
        "unlock_door":   {"group": "Control", "description": "Disengage the door-lock cylinder.",
                          "params": {}},
        "lock_bucket":   {"group": "Control", "description": "Engage the bucket-lock cylinder.",
                          "params": {}},
        "unlock_bucket": {"group": "Control", "description": "Disengage the bucket-lock cylinder.",
                          "params": {}},
        "go_to_bucket": {
            "group": "Control",
            "description": "Rotate the rotor so the named bucket is in the loading position.",
            "params": {"n": {"type": "choice", "default": 1, "choices": [1, 2], "required": True}},
            "dangerous": True,
        },
        # ---- profile / calibration -----------------------------------
        "home": {"group": "Control", "description": "Home the rotor (find index pulse).",
                 "params": {}, "dangerous": True},
        "teach_bucket": {
            "group": "Profiles",
            "description": "Calibrate the encoder position for the named bucket.",
            "params": {"n": {"type": "choice", "default": 1, "choices": [1, 2], "required": True}},
            "dangerous": True,
        },
        "calculate_bucket_2": {
            "group": "Profiles",
            "description": "Calculate bucket 2 as 180 degrees from the bucket 1 teachpoint.",
            "params": {},
        },
        "update_profile": {
            "group": "Profiles",
            "description": "Apply the current VSpin profile settings for this driver instance.",
            "params": {},
            "dangerous": True,
        },
        "reinitialize_profile": {
            "group": "Profiles",
            "description": "Reinitialize the VSpin profile and controller status definition.",
            "params": {},
            "dangerous": True,
        },
        # ---- readouts ------------------------------------------------
        "read_status": {
            "group": "Status",
            "description": "Read door / bucket / balanced / spinning / RPM / position flags.",
            "params": {},
            "readonly": True,
        },
    }

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._rotor_angle = 0.0
        self._rpm = 0.0
        self._door_open = False
        self._door_locked = False
        self._bucket_locked = False
        self._current_bucket = 1
        self._homed = False
        self._home_position_ticks = 2502
        self._encoder_pos = 0
        self._position_history: list[tuple[float, int]] = []
        self._servo_velocity_ticks = 0
        self._spin_cancel_requested = False
        params = self.profile.params if self.profile is not None else {}
        vspin_profile = params.get("vspin_profile") if isinstance(params, dict) else None
        if not isinstance(vspin_profile, dict):
            vspin_profile = {}
        self._teachpoints: dict[int, int] = _bucket_teachpoints(
            vspin_profile.get("bucket_teachpoints"),
            self.BUCKET_TEACH_DEFAULTS,
        )

    def _vspin_profile_dict(self) -> dict[str, Any]:
        params = self.profile.params if self.profile is not None else {}
        profile = params.get("vspin_profile") if isinstance(params, dict) else None
        return profile if isinstance(profile, dict) else {}

    def _vspin_profile_float(self, key: str, default: float) -> float:
        raw = self._vspin_profile_dict().get(key, default)
        try:
            return float(raw)
        except (TypeError, ValueError):
            return float(default)

    def _vspin_gains(self, key: str) -> dict[str, int]:
        defaults = (
            _VSPIN_DEFAULT_POSITION_GAINS
            if key == "position_gains"
            else _VSPIN_DEFAULT_VELOCITY_GAINS
        )
        raw = self._vspin_profile_dict().get(key)
        raw = raw if isinstance(raw, dict) else {}
        gains: dict[str, int] = {}
        for name, default in defaults.items():
            try:
                gains[name] = int(raw.get(name, default))
            except (TypeError, ValueError):
                gains[name] = int(default)
        return gains

    def _sample_now(self) -> float:
        return self.clock.now()

    def _remember_position_sample(self, position: int) -> float:
        now = self._sample_now()
        self._position_history.append((now, int(position)))
        self._position_history = [
            sample for sample in self._position_history
            if now - sample[0] <= 5.0
        ][-8:]
        if len(self._position_history) < 2:
            return 0.0
        t0, p0 = self._position_history[0]
        t1, p1 = self._position_history[-1]
        dt = t1 - t0
        if dt <= 0:
            return 0.0
        rpm = abs(p1 - p0) * 60.0 / (_VSPIN_COUNTS_PER_REV * dt)
        return 0.0 if rpm < 0.5 else rpm

    def _encoder_display_position(self, position: int) -> int:
        home = int(self._home_position_ticks) % _VSPIN_COUNTS_PER_REV
        return home + ((int(position) - home) % _VSPIN_COUNTS_PER_REV)

    def _calculate_bucket_2_teachpoint(self) -> dict[str, Any] | None:
        bucket1 = self._teachpoints.get(1)
        if bucket1 is None:
            return None
        bucket1 = int(bucket1) % _VSPIN_COUNTS_PER_REV
        bucket2 = (bucket1 + (_VSPIN_COUNTS_PER_REV // 2)) % _VSPIN_COUNTS_PER_REV
        self._teachpoints[1] = bucket1
        self._teachpoints[2] = bucket2
        return {
            "bucket": 2,
            "bucket1_teachpoint": bucket1,
            "teachpoint": bucket2,
            "teachpoints": dict(self._teachpoints),
            "counts_per_revolution": _VSPIN_COUNTS_PER_REV,
            "calculated": True,
        }

    async def initialize(self) -> None:
        async with self._transition(DriverState.INITIALIZING):
            self._state = DriverState.IDLE
            await self._emit("state_change", {"to": "IDLE"})

    async def close(self) -> None:
        self._state = DriverState.CLOSED

    def _resolve_peer(self) -> Driver | None:
        resolver = getattr(self, "_peer_resolver_instance", None)
        if resolver is None:
            resolver = getattr(type(self), "_peer_driver_resolver", None)
        if resolver is None:
            return None
        try:
            return resolver(self)
        except Exception:
            return None

    def set_peer_resolver(self, resolver) -> None:
        self._peer_resolver_instance = resolver


class SimVSpin(VSpinBase):
    """In-process VSpin simulator — animates the rotor joint."""

    transport = "sim"

    async def execute(self, cmd: Command) -> CommandResult:
        dispatch = {
            "spin": self._sim_spin,
            "stop_spin": self._sim_stop_spin,
            "open_door": self._sim_open_door,
            "close_door": self._sim_close_door,
            "lock_door": self._sim_lock_door,
            "unlock_door": self._sim_unlock_door,
            "lock_bucket": self._sim_lock_bucket,
            "unlock_bucket": self._sim_unlock_bucket,
            "go_to_bucket": self._sim_go_to_bucket,
            "home": self._sim_home,
            "teach_bucket": self._sim_teach_bucket,
            "calculate_bucket_2": self._sim_calculate_bucket_2,
            "update_profile": self._sim_update_profile,
            "reinitialize_profile": self._sim_reinitialize_profile,
            "read_status": self._sim_read_status,
        }
        handler = dispatch.get(cmd.kind)
        if handler is None:
            return CommandResult(ok=False, payload={"reason": f"unknown kind {cmd.kind!r}"})
        return await handler(cmd)

    # ---- spin (preserved animation from the original implementation) ----

    async def _sim_spin(self, cmd: Command) -> CommandResult:
        rcf = cmd.params.get("rcf")
        rpm = cmd.params.get("rpm")
        radius = float(cmd.params.get("rotor_radius_mm", 100.0))
        max_velocity_rpm = self._vspin_profile_float(
            "max_velocity",
            _VSPIN_DEFAULT_MAX_VELOCITY_RPM,
        )
        if rcf is not None:
            rpm = _rcf_to_rpm(float(rcf), rotor_radius_mm=radius)
        elif rpm is None:
            rpm = 1500.0
        rpm = _clamp(float(rpm), 1.0, max_velocity_rpm)
        rcf = _rpm_to_rcf(rpm, rotor_radius_mm=radius)
        duration = float(cmd.params.get("duration_s", 10.0))
        omega = float(rpm) * (2 * math.pi / 60.0)
        async with self._transition(DriverState.BUSY):
            self._rpm = float(rpm)
            await self._emit_ramp(omega_target=omega, duration=self.DEFAULT_RAMP_S, accelerating=True)
            steps = max(2, int(duration / 0.5))
            step_dt = duration / steps
            for _ in range(steps):
                await self.clock.sleep(step_dt)
                self._rotor_angle = (self._rotor_angle + omega * step_dt) % (2 * math.pi)
                await self._emit("kinematic_state", _kine_payload(self.clock.now(),
                                  {"dof_rotor": self._rotor_angle, "rotor": self._rotor_angle},
                                  omega))
            await self._emit_ramp(omega_target=0.0, duration=self.DEFAULT_RAMP_S, accelerating=False)
            self._rpm = 0.0
        return CommandResult(ok=True, payload={"rcf": float(rcf), "duration_s": duration, "rpm": float(rpm)})

    async def _sim_stop_spin(self, cmd: Command) -> CommandResult:
        self._rpm = 0.0
        await self._emit("kinematic_state", _kine_payload(
            self.clock.now(),
            {"dof_rotor": self._rotor_angle, "rotor": self._rotor_angle},
            0.0,
        ))
        return CommandResult(
            ok=True,
            payload={"stopped": True, "smooth": bool(cmd.params.get("smooth", True))},
        )

    async def _emit_ramp(self, *, omega_target: float, duration: float, accelerating: bool) -> None:
        steps = max(2, int(duration * 5))
        step_dt = duration / steps
        if accelerating:
            start_omega, end_omega = 0.0, omega_target
        else:
            start_omega, end_omega = omega_target, 0.0
        for i in range(1, steps + 1):
            alpha = i / steps
            omega_i = start_omega + (end_omega - start_omega) * alpha
            await self.clock.sleep(step_dt)
            self._rotor_angle = (self._rotor_angle + omega_i * step_dt) % (2 * math.pi)
            await self._emit("kinematic_state", _kine_payload(self.clock.now(),
                              {"dof_rotor": self._rotor_angle, "rotor": self._rotor_angle},
                              omega_i))

    # ---- door / bucket / status -------------------------------------

    async def _sim_open_door(self, _cmd: Command) -> CommandResult:
        await self.clock.sleep(2.0)
        self._door_open = True
        return CommandResult(ok=True, payload={"door_open": True})

    async def _sim_close_door(self, _cmd: Command) -> CommandResult:
        await self.clock.sleep(2.0)
        self._door_open = False
        return CommandResult(ok=True, payload={"door_open": False})

    async def _sim_lock_door(self, _cmd: Command) -> CommandResult:
        await self.clock.sleep(0.3)
        self._door_locked = True
        return CommandResult(ok=True, payload={"door_locked": True})

    async def _sim_unlock_door(self, _cmd: Command) -> CommandResult:
        await self.clock.sleep(0.3)
        self._door_locked = False
        return CommandResult(ok=True, payload={"door_locked": False})

    async def _sim_lock_bucket(self, _cmd: Command) -> CommandResult:
        await self.clock.sleep(0.3)
        self._bucket_locked = True
        return CommandResult(ok=True, payload={"bucket_locked": True})

    async def _sim_unlock_bucket(self, _cmd: Command) -> CommandResult:
        await self.clock.sleep(0.3)
        self._bucket_locked = False
        return CommandResult(ok=True, payload={"bucket_locked": False})

    async def _sim_go_to_bucket(self, cmd: Command) -> CommandResult:
        n = int(cmd.params.get("n", 1))
        if n not in (1, 2):
            return CommandResult(ok=False, payload={"reason": f"bad bucket {n}"})
        self._door_open = False
        self._door_locked = True
        self._bucket_locked = False
        # 180° between buckets. Rotate via small steps so the viewer animates.
        target_angle = 0.0 if n == 1 else math.pi
        async with self._transition(DriverState.BUSY):
            steps = 12
            start = self._rotor_angle
            for i in range(1, steps + 1):
                await self.clock.sleep(0.25)
                self._rotor_angle = (start + (target_angle - start) * i / steps) % (2 * math.pi)
                await self._emit("kinematic_state", _kine_payload(
                    self.clock.now(),
                    {"dof_rotor": self._rotor_angle, "rotor": self._rotor_angle},
                    0.0,
                ))
        self._current_bucket = n
        self._encoder_pos = self._home_position_ticks + self._teachpoints.get(n, 0)
        self._bucket_locked = True
        self._door_locked = False
        self._door_open = True
        return CommandResult(ok=True, payload={
            "current_bucket": n,
            "encoder_pos": self._encoder_pos,
            "door_open": True,
            "door_locked": False,
            "bucket_locked": True,
        })

    async def _sim_home(self, _cmd: Command) -> CommandResult:
        async with self._transition(DriverState.BUSY):
            await self.clock.sleep(3.0)
            self._rotor_angle = 0.0
            self._homed = True
            self._encoder_pos = 0
            await self._emit("kinematic_state", _kine_payload(
                self.clock.now(),
                {"dof_rotor": 0.0, "rotor": 0.0},
                0.0,
            ))
        return CommandResult(ok=True, payload={"homed": True})

    async def _sim_teach_bucket(self, cmd: Command) -> CommandResult:
        n = int(cmd.params.get("n", 1))
        # In real hardware this captures the current encoder ticks. We
        # synthesize a plausible value mid-range.
        tp = self._teachpoints.get(n, 1500 + n * 1000)
        self._teachpoints[n] = tp
        return CommandResult(ok=True, payload={"bucket": n, "teachpoint": tp})

    async def _sim_calculate_bucket_2(self, _cmd: Command) -> CommandResult:
        payload = self._calculate_bucket_2_teachpoint()
        if payload is None:
            return CommandResult(ok=False, payload={"reason": "no teachpoint for bucket 1"})
        return CommandResult(ok=True, payload=payload)

    async def _sim_update_profile(self, _cmd: Command) -> CommandResult:
        return CommandResult(ok=True, payload={
            "profile": "Centrifuge",
            "teachpoints": dict(self._teachpoints),
            "runtime_only": True,
        })

    async def _sim_reinitialize_profile(self, _cmd: Command) -> CommandResult:
        self._homed = False
        self._rpm = 0.0
        return CommandResult(ok=True, payload={"profile": "Centrifuge", "reinitialized": True})

    async def _sim_read_status(self, _cmd: Command) -> CommandResult:
        return CommandResult(ok=True, payload={
            "door_open": self._door_open,
            "door_closed": not self._door_open,
            "door_locked": self._door_locked,
            "bucket_locked": self._bucket_locked,
            "bucket_unlocked": not self._bucket_locked,
            "balanced": True,
            "spinning": self._rpm > 0.0,
            "in_motion": self._rpm > 0.0,
            "amp_enabled": True,
            "homing": False,
            "rpm": self._rpm,
            "home_position_ticks": self._home_position_ticks,
            "encoder_pos": self._encoder_pos,
            "encoder_display_pos": self._encoder_display_position(self._encoder_pos),
            "current_bucket": self._current_bucket,
            "rotor_angle_deg": math.degrees(self._rotor_angle),
            "homed": self._homed,
            "teachpoints": dict(self._teachpoints),
        })

    async def spin(self, rcf: float, duration_s: float) -> None:
        """Convenience wrapper used by older protocols."""
        await self.execute(Command(kind="spin",
                                   params={"rcf": rcf, "duration_s": duration_s}))


class VSpinDriver(VSpinBase):
    """Real-hardware VSpin driver — Velocity11 PIC-Servo NMC over RS-232.

    Each method dispatches to the corresponding NMC primitive in
    drivers/transports_nmc.py. The patterns are lifted verbatim from
    the C++ plugin source at Plugins/Centrifuge/VSpin v3/ — e.g.,
    `home()` issues `servo_set_homing(AN_ADDR_PIC_SERVO,
    ON_INDEX | HOME_STOP_SMOOTH)` then polls `nmc_get_stat` for
    `HOME_IN_PROG` to clear.

    Untested on real hardware as of writing — bench bring-up (Tier C)
    will verify and patch any vendor quirks.
    """

    transport = "serial"

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._link = None  # type: ignore[assignment]

    def _hardware_now(self) -> float:
        return time.monotonic()

    async def _hardware_sleep(self, seconds: float) -> None:
        await asyncio.sleep(max(0.0, float(seconds)))

    def _sample_now(self) -> float:
        return self._hardware_now()

    async def initialize(self) -> None:
        from vspin_cockpit.drivers.transports_nmc import (
            AN_ADDR_PIC_IO, AN_ADDR_PIC_SERVO, IOMODTYPE, NmcLink,
            SERVOMODTYPE,
        )
        port = self.profile.params.get("port") or self.profile.params.get("connection", {}).get("port")
        if not port:
            raise RuntimeError("VSpinDriver requires a serial port in DeviceProfile.params")
        baudrate = int(self.profile.params.get("baudrate", 57600))
        self._link = NmcLink(
            port=str(port),
            baudrate=baudrate,
        )
        async with self._transition(DriverState.INITIALIZING):
            modules = await self._link.init_network(target_baudrate=baudrate)
            if len(modules) != 2:
                raise RuntimeError(f"VSpin expected 2 NMC modules, found {len(modules)}")
            if modules.get(AN_ADDR_PIC_SERVO, (None, None))[0] != SERVOMODTYPE:
                raise RuntimeError("VSpin PIC-SERVO was not found at address 1")
            if modules.get(AN_ADDR_PIC_IO, (None, None))[0] != IOMODTYPE:
                raise RuntimeError("VSpin PIC-IO was not found at address 2")
            await self._setup_vendor_network()
            self._state = DriverState.IDLE
            await self._emit("state_change", {"to": "IDLE"})

    async def close(self) -> None:
        if self._link is not None:
            await self._link.close()
        self._state = DriverState.CLOSED

    async def _setup_vendor_network(self) -> None:
        """Mirror VSpinController::SetUpNetwork from the vendor plugin."""
        from vspin_cockpit.drivers.transports_nmc import (
            AN_ADDR_PIC_IO, AN_ADDR_PIC_SERVO, IOI_IN_AMP_ENABLED,
            IOI_IN_AMP_FAULT, IOI_IN_BUCKET_LOCKED, IOI_IN_BUCKET_UNLOCKED,
            IOI_IN_DOOR_LOCKED, IOI_IN_DOOR_OPEN, IOI_IN_IMBALANCE,
            IOI_IN_SPINNING, IOI_OUT_BUCKET_LOCK_CYL, IOI_OUT_DOOR_CYL,
            IOI_OUT_DOOR_LOCK_CYL, IOI_OUT_VERSION_TOGGLE, SEND_AD,
            SEND_AD1, SEND_AUX, SEND_HOME, SEND_INPUTS, SEND_POS, SEND_VEL,
        )
        link = self._link
        if link is None:
            raise RuntimeError("VSpin link not initialized")
        await link.define_status(
            AN_ADDR_PIC_SERVO,
            SEND_POS | SEND_AD | SEND_VEL | SEND_AUX | SEND_HOME,
        )
        for bit in (
            IOI_IN_AMP_FAULT,
            IOI_IN_SPINNING,
            IOI_IN_IMBALANCE,
            IOI_IN_BUCKET_UNLOCKED,
            IOI_IN_BUCKET_LOCKED,
            IOI_IN_DOOR_OPEN,
            IOI_IN_DOOR_LOCKED,
            IOI_IN_AMP_ENABLED,
        ):
            await link.io_bit_dir_in(AN_ADDR_PIC_IO, bit)
        for bit in (
            IOI_OUT_VERSION_TOGGLE,
            IOI_OUT_BUCKET_LOCK_CYL,
            IOI_OUT_DOOR_CYL,
            IOI_OUT_DOOR_LOCK_CYL,
        ):
            await link.io_bit_dir_out(AN_ADDR_PIC_IO, bit)
        for bit in (
            IOI_OUT_VERSION_TOGGLE,
            IOI_OUT_BUCKET_LOCK_CYL,
            IOI_OUT_DOOR_CYL,
            IOI_OUT_DOOR_LOCK_CYL,
        ):
            await link.io_clr_out_bit(AN_ADDR_PIC_IO, bit)
        await link.define_status(AN_ADDR_PIC_IO, SEND_INPUTS | SEND_AD1)

    async def _set_servo_gains(
        self,
        link,
        gain_key: str,
        *,
        out_limit: int | None = None,
        position_error_limit: int | None = None,
    ) -> int:
        from vspin_cockpit.drivers.transports_nmc import AN_ADDR_PIC_SERVO

        gains = self._vspin_gains(gain_key)
        if out_limit is not None:
            gains["out_limit"] = int(out_limit)
        if position_error_limit is not None:
            gains["position_error_limit"] = int(position_error_limit)
        return await link.servo_set_gain(AN_ADDR_PIC_SERVO, **gains)

    async def _enable_amp_and_reset_servo_status(self, link) -> None:
        from vspin_cockpit.drivers.transports_nmc import (
            AMP_ENABLE, AN_ADDR_PIC_SERVO, MOTOR_OFF, STOP_ABRUPT,
        )

        await self._hardware_sleep(0.1)
        await link.servo_stop_motor(AN_ADDR_PIC_SERVO, MOTOR_OFF)
        await self._set_servo_gains(link, "position_gains")
        await link.servo_stop_motor(AN_ADDR_PIC_SERVO, STOP_ABRUPT)
        await link.servo_stop_motor(AN_ADDR_PIC_SERVO, AMP_ENABLE)
        await self._hardware_sleep(0.1)
        await link.servo_clear_bits(AN_ADDR_PIC_SERVO)

    async def _wait_for_io_bit(
        self,
        link,
        bit: int,
        value: bool,
        *,
        timeout_s: float = 5.0,
    ) -> None:
        from vspin_cockpit.core.transports import TransportError
        from vspin_cockpit.drivers.transports_nmc import AN_ADDR_PIC_IO

        deadline = self._hardware_now() + timeout_s
        last = None
        while self._hardware_now() <= deadline:
            last = await link.io_in_bit_val(AN_ADDR_PIC_IO, bit)
            if bool(last) == bool(value):
                return
            await self._hardware_sleep(0.05)
        raise TransportError(f"timed out waiting for IO bit {bit} to become {value}")

    async def _vspin_io_state(self, link, bit: int) -> bool:
        """Return the logical VSpin sensor state, matching the vendor driver."""
        from vspin_cockpit.drivers.transports_nmc import (
            AN_ADDR_PIC_IO, IOI_IN_AMP_ENABLED, IOI_IN_BUCKET_LOCKED,
            IOI_IN_BUCKET_UNLOCKED, IOI_IN_DOOR_LOCKED,
        )

        raw = bool(await link.io_in_bit_val(AN_ADDR_PIC_IO, bit))
        if bit in {
            IOI_IN_BUCKET_UNLOCKED,
            IOI_IN_BUCKET_LOCKED,
            IOI_IN_DOOR_LOCKED,
            IOI_IN_AMP_ENABLED,
        }:
            return not raw
        return raw

    async def _wait_for_vspin_io_state(
        self,
        link,
        bit: int,
        value: bool,
        *,
        timeout_s: float = 5.0,
    ) -> None:
        from vspin_cockpit.core.transports import TransportError
        from vspin_cockpit.drivers.transports_nmc import AN_ADDR_PIC_IO

        names = {
            0: "amp_fault",
            1: "spinning",
            2: "imbalance",
            3: "bucket_unlocked",
            4: "bucket_locked",
            6: "door_open",
            7: "door_locked",
            11: "amp_enabled",
        }
        deadline = self._hardware_now() + timeout_s
        last_raw = None
        last_logical = None
        while self._hardware_now() <= deadline:
            last_raw = bool(await link.io_in_bit_val(AN_ADDR_PIC_IO, bit))
            last_logical = await self._vspin_io_state(link, bit)
            if bool(last_logical) == bool(value):
                return
            await self._hardware_sleep(0.05)
        name = names.get(bit, f"bit_{bit}")
        raise TransportError(
            f"timed out waiting for {name} to become {value} "
            f"(bit={bit}, raw={last_raw}, logical={last_logical})"
        )

    async def _prepare_spin_hardware(self, link) -> None:
        from vspin_cockpit.drivers.transports_nmc import (
            AN_ADDR_PIC_IO, IOI_IN_BUCKET_UNLOCKED, IOI_IN_DOOR_LOCKED,
            IOI_IN_DOOR_OPEN, IOI_OUT_BUCKET_LOCK_CYL, IOI_OUT_DOOR_CYL,
            IOI_OUT_DOOR_LOCK_CYL,
        )

        await link.io_clr_out_bit(AN_ADDR_PIC_IO, IOI_OUT_DOOR_CYL)
        await self._wait_for_vspin_io_state(link, IOI_IN_DOOR_OPEN, False)
        await self._hardware_sleep(0.75)
        await link.io_clr_out_bit(AN_ADDR_PIC_IO, IOI_OUT_DOOR_LOCK_CYL)
        await self._wait_for_vspin_io_state(link, IOI_IN_DOOR_LOCKED, True)
        await link.io_clr_out_bit(AN_ADDR_PIC_IO, IOI_OUT_BUCKET_LOCK_CYL)
        await self._wait_for_vspin_io_state(link, IOI_IN_BUCKET_UNLOCKED, True)
        await self._enable_amp_and_reset_servo_status(link)
        self._door_open = False
        self._door_locked = True
        self._bucket_locked = False

    async def _open_vspin_door_after_motion(self, link) -> None:
        from vspin_cockpit.drivers.transports_nmc import (
            AN_ADDR_PIC_IO, AN_ADDR_PIC_SERVO, IOI_IN_BUCKET_LOCKED,
            IOI_IN_BUCKET_UNLOCKED, IOI_IN_DOOR_LOCKED, IOI_IN_DOOR_OPEN,
            IOI_OUT_BUCKET_LOCK_CYL, IOI_OUT_DOOR_CYL,
            IOI_OUT_DOOR_LOCK_CYL, MOTOR_OFF,
        )

        await self._hardware_sleep(0.1)
        await link.servo_stop_motor(AN_ADDR_PIC_SERVO, MOTOR_OFF)
        await self._hardware_sleep(0.1)
        if await self._vspin_io_state(link, IOI_IN_BUCKET_UNLOCKED):
            await link.io_set_out_bit(AN_ADDR_PIC_IO, IOI_OUT_BUCKET_LOCK_CYL)
            await self._wait_for_vspin_io_state(link, IOI_IN_BUCKET_LOCKED, True)
        if await self._vspin_io_state(link, IOI_IN_DOOR_LOCKED):
            await link.io_set_out_bit(AN_ADDR_PIC_IO, IOI_OUT_DOOR_LOCK_CYL)
            await self._wait_for_vspin_io_state(link, IOI_IN_DOOR_LOCKED, False)
        if not await self._vspin_io_state(link, IOI_IN_DOOR_OPEN):
            await link.io_set_out_bit(AN_ADDR_PIC_IO, IOI_OUT_DOOR_CYL)
            await self._wait_for_vspin_io_state(link, IOI_IN_DOOR_OPEN, True)
        self._door_open = True
        self._door_locked = False
        self._bucket_locked = True

    def _trajectory_velocity_accel(
        self,
        *,
        vel_percent: float,
        accel_percent: float,
        gain_key: str = "velocity_gains",
    ) -> tuple[int, int]:
        max_velocity_rpm = self._vspin_profile_float(
            "max_velocity",
            _VSPIN_DEFAULT_MAX_VELOCITY_RPM,
        )
        gain_servo_rate = self._vspin_gains(gain_key)["servo_rate"]
        velocity = int(
            _VSPIN_VELOCITY_CONSTANT
            * max_velocity_rpm
            * gain_servo_rate
            * vel_percent
            / 100.0
        )
        acceleration = int(
            _VSPIN_MAX_ACCEL
            * gain_servo_rate
            * gain_servo_rate
            * accel_percent
            / 100.0
        )
        return velocity, acceleration

    def _spin_trajectory_values(
        self,
        *,
        current_position: int,
        vel_percent: float,
        accel_percent: float,
        cruise_time_ms: int,
    ) -> tuple[int, int, int]:
        max_velocity_rpm = self._vspin_profile_float(
            "max_velocity",
            _VSPIN_DEFAULT_MAX_VELOCITY_RPM,
        )
        desired_position = int(current_position)
        desired_position += int(
            _VSPIN_COUNTS_PER_REV
            * (max_velocity_rpm * vel_percent)
            * (cruise_time_ms + _VSPIN_SPIN_OVERSHOOT_MS)
            / (60.0 * 1000.0 * 100.0)
        )
        desired_position += 2 * int(
            (
                max_velocity_rpm
                * max_velocity_rpm
                * vel_percent
                * vel_percent
                * _VSPIN_COUNTS_PER_REV
            )
            / (48000.0 * accel_percent * 100.0)
        )
        velocity, acceleration = self._trajectory_velocity_accel(
            vel_percent=vel_percent,
            accel_percent=accel_percent,
        )
        return desired_position, velocity, acceleration

    async def _command_spin_deceleration(self, link, decel_percent: float) -> None:
        from vspin_cockpit.drivers.transports_nmc import (
            AN_ADDR_PIC_SERVO, ENABLE_SERVO, LOAD_ACC, LOAD_VEL, START_NOW,
            VEL_MODE,
        )

        _, acceleration = self._trajectory_velocity_accel(
            vel_percent=0.0,
            accel_percent=max(1.0, decel_percent),
        )
        await self._set_servo_gains(link, "velocity_gains")
        await link.servo_load_trajectory(
            AN_ADDR_PIC_SERVO,
            mode=ENABLE_SERVO | VEL_MODE | LOAD_VEL | LOAD_ACC | START_NOW,
            velocity=0,
            accel=acceleration,
        )

    async def _home_rotor_vendor_sequence(self, link) -> int:
        from vspin_cockpit.core.transports import TransportError
        from vspin_cockpit.drivers.transports_nmc import (
            AN_ADDR_PIC_IO, AN_ADDR_PIC_SERVO, ENABLE_SERVO,
            HOME_STOP_SMOOTH, IOI_IN_BUCKET_UNLOCKED, IOI_IN_DOOR_LOCKED,
            IOI_IN_DOOR_OPEN, IOI_IN_IMBALANCE, IOI_OUT_BUCKET_LOCK_CYL,
            IOI_OUT_DOOR_CYL, IOI_OUT_DOOR_LOCK_CYL, LOAD_ACC, LOAD_VEL,
            ON_INDEX, START_NOW, STAT_HOME_IN_PROG, VEL_MODE,
        )

        await link.io_clr_out_bit(AN_ADDR_PIC_IO, IOI_OUT_DOOR_CYL)
        await self._wait_for_vspin_io_state(link, IOI_IN_DOOR_OPEN, False)
        await self._hardware_sleep(0.75)
        await link.io_clr_out_bit(AN_ADDR_PIC_IO, IOI_OUT_DOOR_LOCK_CYL)
        await self._wait_for_vspin_io_state(link, IOI_IN_DOOR_LOCKED, True)
        await link.io_clr_out_bit(AN_ADDR_PIC_IO, IOI_OUT_BUCKET_LOCK_CYL)
        await self._wait_for_vspin_io_state(link, IOI_IN_BUCKET_UNLOCKED, True)

        await self._enable_amp_and_reset_servo_status(link)
        await link.servo_reset_pos(AN_ADDR_PIC_SERVO)
        await self._set_servo_gains(
            link,
            "velocity_gains",
            out_limit=_VSPIN_HOMING_GAIN_OUT_LIMIT,
            position_error_limit=_VSPIN_HOMING_GAIN_POSITION_ERROR_LIMIT,
        )
        max_velocity_rpm = self._vspin_profile_float(
            "max_velocity",
            _VSPIN_DEFAULT_MAX_VELOCITY_RPM,
        )
        homing_velocity_percent = (100.0 * _VSPIN_HOMING_VEL_RPM) / max_velocity_rpm
        velocity, acceleration = self._trajectory_velocity_accel(
            vel_percent=homing_velocity_percent,
            accel_percent=_VSPIN_HOMING_ACCEL_PERCENT,
        )
        await link.servo_load_trajectory(
            AN_ADDR_PIC_SERVO,
            mode=ENABLE_SERVO | VEL_MODE | LOAD_VEL | LOAD_ACC | START_NOW,
            velocity=velocity,
            accel=acceleration,
        )
        await link.servo_set_homing(AN_ADDR_PIC_SERVO, HOME_STOP_SMOOTH | ON_INDEX)

        deadline = self._hardware_now() + _VSPIN_TIMEOUT_HOMING_S
        last_stat = 0
        while self._hardware_now() <= deadline:
            last_stat = await link.nmc_get_stat(AN_ADDR_PIC_SERVO)
            if not (last_stat & STAT_HOME_IN_PROG):
                home_pos = await link.servo_get_home(AN_ADDR_PIC_SERVO)
                home_pos %= _VSPIN_COUNTS_PER_REV
                self._home_position_ticks = home_pos
                self._encoder_pos = await link.servo_get_position(AN_ADDR_PIC_SERVO)
                self._homed = True
                self._position_history.clear()
                self._remember_position_sample(self._encoder_pos)
                return home_pos
            await self._hardware_sleep(0.1)

        bucket_unlocked = await self._vspin_io_state(link, IOI_IN_BUCKET_UNLOCKED)
        door_locked = await self._vspin_io_state(link, IOI_IN_DOOR_LOCKED)
        balanced = not await link.io_in_bit_val(AN_ADDR_PIC_IO, IOI_IN_IMBALANCE)
        raise TransportError(
            "VSpin home timed out after "
            f"{_VSPIN_TIMEOUT_HOMING_S:.1f}s "
            f"(stat=0x{last_stat:02X}, bucket_unlocked={bucket_unlocked}, "
            f"door_locked={door_locked}, balanced={balanced})"
        )

    def _bucket_target_position(
        self,
        current_position: int,
        bucket: int,
        *,
        retry: bool = False,
    ) -> int | None:
        teachpoint = self._teachpoints.get(bucket)
        if teachpoint is None:
            return None
        current_position = int(current_position)
        target_mod = (int(self._home_position_ticks) + int(teachpoint)) % _VSPIN_COUNTS_PER_REV
        current_mod = current_position % _VSPIN_COUNTS_PER_REV
        delta = (target_mod - current_mod) % _VSPIN_COUNTS_PER_REV
        if delta > (_VSPIN_COUNTS_PER_REV / 2):
            delta -= _VSPIN_COUNTS_PER_REV
        target = current_position + int(delta)
        # The vendor driver retries short failed bucket moves by commanding the
        # same teachpoint one full revolution later, which unsticks controllers
        # that report MOVE_DONE before settling at the target.
        if retry and abs(target - current_position) < (_VSPIN_COUNTS_PER_REV / 2):
            target += _VSPIN_COUNTS_PER_REV
        return target

    def _bucket_teachpoint_from_position(self, position: int) -> int:
        return (int(position) - int(self._home_position_ticks)) % _VSPIN_COUNTS_PER_REV

    async def _move_to_bucket_position(
        self,
        link,
        bucket: int,
        current_position: int | None = None,
        *,
        retry: bool = False,
    ) -> dict[str, Any] | None:
        from vspin_cockpit.drivers.transports_nmc import (
            AN_ADDR_PIC_SERVO, ENABLE_SERVO, LOAD_ACC, LOAD_POS, LOAD_VEL,
            START_NOW,
        )

        if current_position is None:
            current_position = await link.servo_get_position(AN_ADDR_PIC_SERVO)
        target = self._bucket_target_position(current_position, bucket, retry=retry)
        if target is None:
            return None
        max_velocity_rpm = self._vspin_profile_float(
            "max_velocity",
            _VSPIN_DEFAULT_MAX_VELOCITY_RPM,
        )
        velocity, acceleration = self._trajectory_velocity_accel(
            vel_percent=(100.0 * _VSPIN_DOOR_VEL_RPM) / max_velocity_rpm,
            accel_percent=_VSPIN_DOOR_ACCEL_PERCENT,
            gain_key="position_gains",
        )
        await self._set_servo_gains(link, "position_gains")
        await link.servo_load_trajectory(
            AN_ADDR_PIC_SERVO,
            mode=ENABLE_SERVO | LOAD_POS | LOAD_VEL | LOAD_ACC | START_NOW,
            position=target,
            velocity=velocity,
            accel=acceleration,
        )
        self._current_bucket = bucket
        self._encoder_pos = target
        return {
            "current_bucket": bucket,
            "encoder_pos": current_position,
            "encoder_target": target,
            "counts_per_revolution": _VSPIN_COUNTS_PER_REV,
        }

    async def _present_bucket_position(self, link, bucket: int) -> dict[str, Any] | None:
        from vspin_cockpit.core.transports import TransportError
        from vspin_cockpit.drivers.transports_nmc import AN_ADDR_PIC_SERVO

        last_presentation: dict[str, Any] | None = None
        last_achieved = self._encoder_pos
        for attempt in range(_VSPIN_BUCKET_PRESENT_RETRIES + 1):
            current_position = await link.servo_get_position(AN_ADDR_PIC_SERVO)
            presentation = await self._move_to_bucket_position(
                link,
                bucket,
                current_position=current_position,
                retry=attempt > 0,
            )
            if presentation is None:
                return None
            last_presentation = presentation
            await self._wait_for_vspin_move_started(link)
            last_achieved = await self._wait_for_vspin_move_done(
                link,
                timeout_s=_VSPIN_TIMEOUT_SERVO_TO_DOOR_S,
            )
            await self._hardware_sleep(1.0)
            presentation["encoder_achieved"] = last_achieved
            if abs(last_achieved - int(presentation["encoder_target"])) <= _VSPIN_BUCKET_TOLERANCE_TICKS:
                return presentation

        target = None if last_presentation is None else last_presentation.get("encoder_target")
        raise TransportError(
            "VSpin bucket presentation failed alignment after "
            f"{_VSPIN_BUCKET_PRESENT_RETRIES + 1} attempts "
            f"(position={last_achieved}, target={target}, "
            f"tolerance={_VSPIN_BUCKET_TOLERANCE_TICKS})"
        )

    async def _wait_for_vspin_move_started(self, link, timeout_s: float = 0.5) -> bool:
        from vspin_cockpit.drivers.transports_nmc import (
            AN_ADDR_PIC_SERVO, STAT_MOVE_DONE,
        )

        deadline = self._hardware_now() + timeout_s
        while self._hardware_now() <= deadline:
            stat = await link.nmc_get_stat(AN_ADDR_PIC_SERVO)
            if not (stat & STAT_MOVE_DONE):
                return True
            await self._hardware_sleep(0.05)
        return False

    async def _wait_for_vspin_move_done(
        self,
        link,
        timeout_s: float = 15.0,
        *,
        target_position: int | None = None,
    ) -> int:
        from vspin_cockpit.core.transports import TransportError
        from vspin_cockpit.drivers.transports_nmc import (
            AN_ADDR_PIC_SERVO, STAT_MOVE_DONE,
        )

        deadline = self._hardware_now() + timeout_s
        last_stat = 0
        last_position = self._encoder_pos
        while self._hardware_now() <= deadline:
            last_stat = await link.nmc_get_stat(AN_ADDR_PIC_SERVO)
            last_position = await link.servo_get_position(AN_ADDR_PIC_SERVO)
            if last_stat & STAT_MOVE_DONE:
                self._encoder_pos = last_position
                self._remember_position_sample(self._encoder_pos)
                return self._encoder_pos
            await self._hardware_sleep(0.05)
        raise TransportError(
            f"VSpin bucket presentation timed out after {timeout_s:.1f}s "
            f"(stat=0x{last_stat:02X}, position={last_position}, "
            f"target={target_position})"
        )

    async def execute(self, cmd: Command) -> CommandResult:
        from vspin_cockpit.drivers.transports_nmc import (
            AMP_ENABLE, AN_ADDR_PIC_IO, AN_ADDR_PIC_SERVO, ENABLE_SERVO,
            HOME_STOP_SMOOTH, IOI_IN_AMP_ENABLED, IOI_IN_BUCKET_LOCKED,
            IOI_IN_BUCKET_UNLOCKED, IOI_IN_DOOR_LOCKED, IOI_IN_DOOR_OPEN,
            IOI_IN_IMBALANCE, IOI_IN_SPINNING, IOI_OUT_BUCKET_LOCK_CYL,
            IOI_OUT_DOOR_CYL, IOI_OUT_DOOR_LOCK_CYL, LOAD_ACC, LOAD_POS,
            LOAD_VEL, ON_INDEX, START_NOW, STAT_HOME_IN_PROG, STAT_MOVE_DONE,
            STOP_SMOOTH,
            TransportError,
        )
        link = self._link
        if link is None:
            return CommandResult(ok=False, payload={"reason": "VSpin link not initialized"})
        readonly = bool(type(self).list_commands().get(cmd.kind, {}).get("readonly", False))
        previous_state = self._state
        mark_busy = not readonly and cmd.kind != "stop_spin"
        if mark_busy:
            self._state = DriverState.BUSY
        try:
            if cmd.kind == "home":
                home_pos = await self._home_rotor_vendor_sequence(link)
                presentation = await self._present_bucket_position(link, 1)
                if presentation is None:
                    return CommandResult(ok=False, payload={"reason": "no teachpoint for bucket 1"})
                achieved_position = presentation["encoder_achieved"]
                presentation["encoder_achieved"] = achieved_position
                return CommandResult(ok=True, payload={
                    **presentation,
                    "homed": True,
                    "home_position_ticks": home_pos,
                    "encoder_pos": achieved_position,
                    "bucket_presented": 1,
                })
            if cmd.kind == "open_door":
                await link.io_set_out_bit(AN_ADDR_PIC_IO, IOI_OUT_DOOR_CYL)
                self._door_open = True
                return CommandResult(ok=True)
            if cmd.kind == "close_door":
                await link.io_clr_out_bit(AN_ADDR_PIC_IO, IOI_OUT_DOOR_CYL)
                self._door_open = False
                return CommandResult(ok=True)
            if cmd.kind == "lock_door":
                await link.io_clr_out_bit(AN_ADDR_PIC_IO, IOI_OUT_DOOR_LOCK_CYL)
                self._door_locked = True
                return CommandResult(ok=True)
            if cmd.kind == "unlock_door":
                await link.io_set_out_bit(AN_ADDR_PIC_IO, IOI_OUT_DOOR_LOCK_CYL)
                self._door_locked = False
                return CommandResult(ok=True)
            if cmd.kind == "lock_bucket":
                await link.io_set_out_bit(AN_ADDR_PIC_IO, IOI_OUT_BUCKET_LOCK_CYL)
                self._bucket_locked = True
                return CommandResult(ok=True)
            if cmd.kind == "unlock_bucket":
                await link.io_clr_out_bit(AN_ADDR_PIC_IO, IOI_OUT_BUCKET_LOCK_CYL)
                self._bucket_locked = False
                return CommandResult(ok=True)
            if cmd.kind == "go_to_bucket":
                n = int(cmd.params.get("n", 1))
                await self._prepare_spin_hardware(link)
                presentation = await self._present_bucket_position(link, n)
                if presentation is None:
                    return CommandResult(ok=False, payload={"reason": f"no teachpoint for bucket {n}"})
                achieved_position = presentation["encoder_achieved"]
                await self._open_vspin_door_after_motion(link)
                presentation["encoder_achieved"] = achieved_position
                return CommandResult(ok=True, payload={
                    **presentation,
                    "encoder_pos": achieved_position,
                    "door_open": True,
                    "door_locked": False,
                    "bucket_locked": True,
                })
            if cmd.kind == "teach_bucket":
                n = int(cmd.params.get("n", 1))
                pos = await link.servo_get_position(AN_ADDR_PIC_SERVO)
                teachpoint = self._bucket_teachpoint_from_position(pos)
                self._teachpoints[n] = teachpoint
                return CommandResult(ok=True, payload={
                    "bucket": n,
                    "encoder_pos": pos,
                    "home_position_ticks": self._home_position_ticks,
                    "teachpoint": teachpoint,
                })
            if cmd.kind == "calculate_bucket_2":
                payload = self._calculate_bucket_2_teachpoint()
                if payload is None:
                    return CommandResult(ok=False, payload={"reason": "no teachpoint for bucket 1"})
                return CommandResult(ok=True, payload=payload)
            if cmd.kind == "update_profile":
                return CommandResult(ok=True, payload={
                    "profile": "Centrifuge",
                    "teachpoints": dict(self._teachpoints),
                    "runtime_only": True,
                })
            if cmd.kind == "reinitialize_profile":
                await self._setup_vendor_network()
                return CommandResult(ok=True, payload={"profile": "Centrifuge", "reinitialized": True})
            if cmd.kind == "read_status":
                stat = await link.nmc_get_stat(AN_ADDR_PIC_SERVO)
                pos = await link.servo_get_position(AN_ADDR_PIC_SERVO)
                try:
                    home_pos = await link.servo_get_home(AN_ADDR_PIC_SERVO)
                except Exception:
                    home_pos = self._home_position_ticks
                home_pos = int(home_pos) % _VSPIN_COUNTS_PER_REV
                try:
                    servo_velocity = await link.servo_get_velocity(AN_ADDR_PIC_SERVO)
                except Exception:
                    servo_velocity = 0
                door_open = await self._vspin_io_state(link, IOI_IN_DOOR_OPEN)
                door_locked = await self._vspin_io_state(link, IOI_IN_DOOR_LOCKED)
                bucket_locked = await self._vspin_io_state(link, IOI_IN_BUCKET_LOCKED)
                bucket_unlocked = await self._vspin_io_state(link, IOI_IN_BUCKET_UNLOCKED)
                balanced = not await link.io_in_bit_val(AN_ADDR_PIC_IO, IOI_IN_IMBALANCE)
                spinning = await link.io_in_bit_val(AN_ADDR_PIC_IO, IOI_IN_SPINNING)
                amp_enabled = await self._vspin_io_state(link, IOI_IN_AMP_ENABLED)
                rpm = self._remember_position_sample(pos)
                if not spinning and bool(stat & STAT_MOVE_DONE) and abs(servo_velocity) == 0:
                    rpm = 0.0
                self._door_open = door_open
                self._door_locked = door_locked
                self._bucket_locked = bucket_locked
                self._encoder_pos = pos
                self._home_position_ticks = home_pos
                self._servo_velocity_ticks = servo_velocity
                self._rpm = rpm
                display_pos = self._encoder_display_position(pos)
                return CommandResult(ok=True, payload={
                    "stat_byte": stat, "encoder_pos": pos,
                    "encoder_display_pos": display_pos,
                    "servo_velocity_ticks": servo_velocity,
                    "home_position_ticks": home_pos,
                    "homed": self._homed, "door_open": door_open,
                    "door_closed": not door_open,
                    "door_locked": door_locked, "bucket_locked": bucket_locked,
                    "bucket_unlocked": bucket_unlocked,
                    "balanced": balanced,
                    "spinning": spinning,
                    "in_motion": not bool(stat & STAT_MOVE_DONE),
                    "amp_enabled": amp_enabled,
                    "homing": bool(stat & STAT_HOME_IN_PROG),
                    "rpm": rpm,
                    "current_bucket": self._current_bucket,
                    "teachpoints": dict(self._teachpoints),
                })
            if cmd.kind == "spin":
                rcf = cmd.params.get("rcf")
                rpm = cmd.params.get("rpm")
                radius = float(cmd.params.get("rotor_radius_mm", 100.0))
                max_velocity_rpm = self._vspin_profile_float(
                    "max_velocity",
                    _VSPIN_DEFAULT_MAX_VELOCITY_RPM,
                )
                if rcf is not None:
                    target_rpm = _rcf_to_rpm(float(rcf), rotor_radius_mm=radius)
                elif rpm is not None:
                    target_rpm = float(rpm)
                else:
                    target_rpm = 1500.0
                target_rpm = _clamp(target_rpm, 1.0, max_velocity_rpm)
                rcf = _rpm_to_rcf(target_rpm, rotor_radius_mm=radius)
                vel_percent = _clamp((target_rpm * 100.0) / max_velocity_rpm, 1.0, 100.0)
                accel_percent = _clamp(float(cmd.params.get("accel_pct", 80)), 1.0, 100.0)
                decel_percent = _clamp(float(cmd.params.get("decel_pct", 80)), 1.0, 100.0)
                duration_s = float(cmd.params.get("duration_s", 10.0))
                cruise_time_ms = int(max(0.0, duration_s) * 1000)
                predicted_accel_ms = int((2.5 * max_velocity_rpm * vel_percent) / accel_percent)
                predicted_decel_ms = int((2.5 * max_velocity_rpm * vel_percent) / decel_percent)
                if str(cmd.params.get("time_mode", "time_at_speed")) == "total_time":
                    cruise_time_ms -= predicted_accel_ms + predicted_decel_ms
                    if cruise_time_ms < 0:
                        cruise_time_ms = 0
                        vel_percent = (
                            duration_s
                            * accel_percent
                            * decel_percent
                            / (0.0025 * max_velocity_rpm * (accel_percent + decel_percent))
                        )
                        vel_percent = _clamp(vel_percent, 1.0, 100.0)
                        target_rpm = max_velocity_rpm * vel_percent / 100.0
                        predicted_accel_ms = int((2.5 * max_velocity_rpm * vel_percent) / accel_percent)
                        predicted_decel_ms = int((2.5 * max_velocity_rpm * vel_percent) / decel_percent)
                self._spin_cancel_requested = False
                await self._prepare_spin_hardware(link)
                current_position = await link.servo_get_position(AN_ADDR_PIC_SERVO)
                self._position_history.clear()
                self._remember_position_sample(current_position)
                target_position, velocity, acceleration = self._spin_trajectory_values(
                    current_position=current_position,
                    vel_percent=vel_percent,
                    accel_percent=accel_percent,
                    cruise_time_ms=cruise_time_ms,
                )
                await self._set_servo_gains(link, "velocity_gains")
                await link.servo_load_trajectory(
                    AN_ADDR_PIC_SERVO,
                    mode=ENABLE_SERVO | LOAD_POS | LOAD_VEL | LOAD_ACC | START_NOW,
                    position=target_position,
                    velocity=velocity,
                    accel=acceleration,
                )
                self._rpm = target_rpm
                cruise_deadline = self._hardware_now() + ((predicted_accel_ms + cruise_time_ms) / 1000.0)
                while self._hardware_now() < cruise_deadline and not self._spin_cancel_requested:
                    await self._hardware_sleep(min(0.2, cruise_deadline - self._hardware_now()))
                await self._command_spin_deceleration(link, decel_percent)
                decel_timeout_s = max(
                    5.0,
                    (predicted_decel_ms / 1000.0) + 5.0,
                )
                decel_deadline = self._hardware_now() + decel_timeout_s
                while self._hardware_now() < decel_deadline:
                    stat = await link.nmc_get_stat(AN_ADDR_PIC_SERVO)
                    try:
                        servo_velocity = await link.servo_get_velocity(AN_ADDR_PIC_SERVO)
                    except Exception:
                        servo_velocity = 0
                    if (stat & STAT_MOVE_DONE) and abs(servo_velocity) == 0:
                        break
                    await self._hardware_sleep(0.1)
                self._rpm = 0.0
                return CommandResult(ok=True, payload={
                    "duration_s": duration_s,
                    "rcf": float(rcf),
                    "rpm": float(target_rpm),
                    "velocity_percent": vel_percent,
                    "accel_percent": accel_percent,
                    "decel_percent": decel_percent,
                    "encoder_start": current_position,
                    "encoder_target": target_position,
                    "cancelled": self._spin_cancel_requested,
                })
            if cmd.kind == "stop_spin":
                self._spin_cancel_requested = True
                decel_percent = _clamp(float(cmd.params.get("decel_pct", 80)), 1.0, 100.0)
                await self._command_spin_deceleration(link, decel_percent)
                if bool(cmd.params.get("smooth", True)):
                    await link.servo_stop_motor(AN_ADDR_PIC_SERVO, AMP_ENABLE | STOP_SMOOTH)
                else:
                    from vspin_cockpit.drivers.transports_nmc import STOP_ABRUPT
                    await link.servo_stop_motor(AN_ADDR_PIC_SERVO, AMP_ENABLE | STOP_ABRUPT)
                self._rpm = 0.0
                return CommandResult(ok=True, payload={
                    "stopped": True,
                    "smooth": bool(cmd.params.get("smooth", True)),
                    "decel_percent": decel_percent,
                })
            return CommandResult(ok=False, payload={"reason": f"unknown kind {cmd.kind!r}"})
        except TransportError as exc:
            return CommandResult(ok=False, payload={"transport_error": str(exc)})
        finally:
            if mark_busy and self._state == DriverState.BUSY:
                self._state = previous_state


# ---------------------------------------------------------------------------
# Access2 plate loader — base + sim + real
# ---------------------------------------------------------------------------

# Axis ranges from the URDF (URDF/vspin_access_2_urdf/urdf/vspin_and_loader_urdf.urdf):
#   dof_y_axis      prismatic   0.091–0.120 m  → 91–120 mm
#   dof_z_axis      prismatic   0.010–0.180 m  → 10–180 mm
#   dof_left_finger prismatic   0.014–0.032 m  → 14–32 mm
#   dof_right_finger prismatic  0.016–0.030 m  → 16–30 mm
_Y_MIN_MM, _Y_MAX_MM = 91.0, 120.0
_Z_MIN_MM, _Z_MAX_MM = 10.0, 180.0
_GRIPPER_MIN_MM, _GRIPPER_MAX_MM = 14.0, 32.0

# Teachpoints: where the gripper centre should be for each operation.
# Tuples are (y_mm, z_mm) with gripper_mm computed from holding state.
_TEACHPOINTS: dict[str, tuple[float, float]] = {
    "park":    (91.0, 180.0),  # safe top-rear
    "stage":   (95.0,  50.0),  # plate handoff position
    "hover":   (115.0, 60.0),  # immediately above bucket
    "bucket1": (115.0, 20.0),  # bucket 1 drop position
    "bucket2": (115.0, 20.0),  # bucket 2 drop position (same XY; rotor rotates)
}

# Speed indices map to seconds-per-mm so longer moves take longer.
_SPEED_S_PER_MM = {0: 0.040, 1: 0.020, 2: 0.010}  # slow / medium / fast
_ACCESS2_PROFILE_CHOICES = [0, 1, 2, 3, 4, 5]
_ACCESS2_SPEED_CHOICES = [0, 1, 2]
_ACCESS2_AXIS_CHOICES = ["y", "z", "gripper"]

_ACCESS2_VENDOR_MOTION_DEFAULTS = {
    "load_bucket": 1,
    "unload_bucket": 1,
    "present_bucket": 1,
    "gripper_z_offset_mm": 8.0,
    "plate_height_mm": 15.0,
    "speed": 0,
    "jog_y_mm": 10.0,
    "jog_z_mm": 5.0,
    "jog_gripper_mm": 1.0,
    "absolute_y_mm": 0.0,
    "absolute_z_mm": 0.0,
    "absolute_gripper_mm": 0.0,
    "y_mode": "holding_plate",
    "z_mode": "holding_plate",
    "gripper_mode": "grip_normally",
    "teachpoint_mode": "holding_plate",
    "gripper_teachpoint_mode": "grip_normally",
    "spin_velocity_percent": 10.0,
    "spin_acceleration_percent": 50.0,
    "spin_deceleration_percent": 50.0,
    "spin_timer_mode": "total_time",
    "spin_time_s": 5.0,
}

_ACCESS2_VENDOR_FLASH_DEFAULTS = {
    "ip_settings": {
        "use_dhcp": True,
        "use_dhcp_fallback": True,
        "use_static_ip": False,
        "fallback_ip": "192.168.0.66",
        "fallback_subnet": "255.255.255.0",
    },
    "homing_offsets_mm": {
        "y": -2.0,
        "z": 20.0,
        "gripper": -2.0,
    },
    "gripper_head_teachpoints_mm": {
        "park": {"y": 0.0, "z": -20.0},
    },
    "gripper_teachpoints_mm": {
        "open_position": 0.0,
        "open_threshold": 0.5,
        "close_threshold": 1.5,
        "close_position": 4.5,
    },
    "speed_regulators_percent": {
        "slow": {"y": 7.500000477, "z": 10.0, "gripper": 100.0},
    },
    "conversion_factors_counts_per_mm": {
        "y": 314.9599915,
        "z": 629.9199829,
        "gripper": 1128.81897,
    },
    "motor_settings": {
        "gripper": {
            "grip_normally": {
                "kp": 400,
                "kd": 800,
                "ki": 0,
                "int_limit": 0,
                "out_limit": 40,
                "cur_limit": 0,
                "servo_rate": 1,
                "deadband_compensation": 0,
                "error_limit": 0.1058330014,
                "velocity": 15.99110031,
                "acceleration": 98.56497192,
                "revision": 0,
            },
        },
    },
}

_GRIPPER_SETTINGS = {
    "open_position": 0.0,
    "open_threshold": 0.5,
    "close_threshold": 1.5,
    "close_position": 4.5,
}


class Access2Base(Driver):
    """Agilent Access2 plate-loader arm (3 axes: Y, Z, gripper)."""

    role = "centrifuge_loader"

    supported_commands = {
        # ---- Control tab ----
        "home": {"group": "Control", "description": "Home all 3 axes (Y, Z, gripper).",
                 "params": {}, "dangerous": True},
        "park": {"group": "Control", "description": "Move to the park teachpoint (safe top-rear).",
                 "params": {}, "dangerous": True},
        "load_plate": {
            "group": "Control",
            "description": "Pick a plate from the stage and place it in bucket N of the paired centrifuge.",
            "params": {
                "bucket": {"type": "choice", "default": 1, "choices": [1, 2], "required": True},
                "gripper_offset_mm": {"type": "float", "default": 8.0,
                                      "description": "Z offset from teachpoint to gripper centre."},
                "plate_height_mm": {"type": "float", "default": 15.0,
                                    "description": "Plate height (drives Z clearance)."},
                "speed": {"type": "choice", "default": 0, "choices": [0, 1, 2],
                          "description": "0=slow, 1=medium, 2=fast."},
                "ignore_optical_sensor": {"type": "bool", "default": False},
                "grip_gently": {"type": "bool", "default": False},
                "assume_tallest_plate": {"type": "bool", "default": False},
            },
            "dangerous": True,
        },
        "unload_plate": {
            "group": "Control",
            "description": "Pick a plate from bucket N and return it to the stage.",
            "params": {
                "bucket": {"type": "choice", "default": 1, "choices": [1, 2], "required": True},
                "gripper_offset_mm": {"type": "float", "default": 8.0},
                "plate_height_mm": {"type": "float", "default": 15.0},
                "speed": {"type": "choice", "default": 0, "choices": [0, 1, 2]},
                "ignore_optical_sensor": {"type": "bool", "default": False},
                "grip_gently": {"type": "bool", "default": False},
                "assume_tallest_plate": {"type": "bool", "default": False},
            },
            "dangerous": True,
        },
        "complete_cycle": {
            "group": "Control",
            "description": "Full load → spin → unload cycle. Dispatches `spin` to the paired centrifuge.",
            "params": {
                "bucket": {"type": "choice", "default": 1, "choices": [1, 2], "required": True},
                "rcf": {"type": "float", "default": 1000.0, "required": True},
                "duration_s": {"type": "float", "default": 30.0, "required": True},
                "gripper_offset_mm": {"type": "float", "default": 8.0},
                "plate_height_mm": {"type": "float", "default": 15.0},
                "speed": {"type": "choice", "default": 0, "choices": [0, 1, 2]},
            },
            "dangerous": True,
        },
        # ---- Move tab — absolute / relative axis moves ----
        "move_axis_absolute": {
            "group": "Move",
            "description": "Drive one axis to an absolute position.",
            "params": {
                "axis": {"type": "choice", "choices": ["y", "z", "gripper"], "required": True},
                "position_mm": {"type": "float", "required": True},
                "speed": {"type": "choice", "default": 0, "choices": [0, 1, 2]},
            },
            "dangerous": True,
        },
        "move_axis_relative": {
            "group": "Move",
            "description": "Jog one axis by a delta.",
            "params": {
                "axis": {"type": "choice", "choices": ["y", "z", "gripper"], "required": True},
                "delta_mm": {"type": "float", "required": True},
                "speed": {"type": "choice", "default": 0, "choices": [0, 1, 2]},
            },
            "dangerous": True,
        },
        "goto_teachpoint": {
            "group": "Move",
            "description": "Move the gripper centre to a named teachpoint.",
            "params": {
                "name": {"type": "choice", "default": "park",
                         "choices": list(_TEACHPOINTS.keys()),
                         "required": True},
                "speed": {"type": "choice", "default": 0, "choices": [0, 1, 2]},
            },
            "dangerous": True,
        },
        "open_gripper":  {"group": "Move", "description": "Open the gripper fully.",
                          "params": {}},
        "close_gripper": {"group": "Move", "description": "Close the gripper (around a plate).",
                          "params": {}},
        # ---- Settings tab — recovery ----
        "jog_axis": {
            "group": "Move",
            "description": "Vendor low-level jog command for one Access2 axis.",
            "params": {
                "axis": {"type": "choice", "choices": _ACCESS2_AXIS_CHOICES, "required": True},
                "delta_mm": {"type": "float", "default": 1.0, "required": True},
                "profile": {"type": "choice", "default": 3, "choices": _ACCESS2_PROFILE_CHOICES},
                "speed": {"type": "choice", "default": 0, "choices": _ACCESS2_SPEED_CHOICES},
            },
            "dangerous": True,
        },
        "servo_switch": {
            "group": "Move",
            "description": "Send the vendor servo-switch code to a single axis.",
            "params": {
                "axis": {"type": "choice", "choices": _ACCESS2_AXIS_CHOICES, "required": True},
                "switch_code": {"type": "choice", "default": 1, "choices": [0, 1, 3, 5, 9]},
            },
            "dangerous": True,
        },
        "stop_motor": {
            "group": "Move",
            "description": "Stop or hold one Access2 axis using the vendor stop-motor options.",
            "params": {
                "axis": {"type": "choice", "choices": _ACCESS2_AXIS_CHOICES, "required": True},
                "mode": {"type": "choice", "default": "stop_smooth",
                         "choices": ["amp_disable", "motor_off", "stop_abrupt", "stop_smooth", "hold_position"]},
            },
            "dangerous": True,
        },
        "go_gripper_setting": {
            "group": "Move/Teach",
            "description": "Move gripper to a vendor open/close target or threshold.",
            "params": {
                "setting": {"type": "choice", "default": "open_position",
                            "choices": list(_GRIPPER_SETTINGS.keys()), "required": True},
                "position_mm": {"type": "float", "default": None},
                "speed": {"type": "choice", "default": 0, "choices": _ACCESS2_SPEED_CHOICES},
            },
            "dangerous": True,
        },
        "teach_teachpoint": {
            "group": "Move/Teach",
            "description": "Capture current Y/Z position as a named Access2 teachpoint.",
            "params": {
                "name": {"type": "choice", "default": "park", "choices": list(_TEACHPOINTS.keys()), "required": True},
                "gripper_offset_mm": {"type": "float", "default": 8.0},
                "commit_flash": {"type": "bool", "default": False},
            },
            "dangerous": True,
        },
        "teach_gripper_setting": {
            "group": "Move/Teach",
            "description": "Capture current gripper position as an open/close target or threshold.",
            "params": {
                "setting": {"type": "choice", "default": "open_position",
                            "choices": list(_GRIPPER_SETTINGS.keys()), "required": True},
                "commit_flash": {"type": "bool", "default": False},
            },
            "dangerous": True,
        },
        "open_door": {"group": "Centrifuge", "description": "Open the paired VSpin door.",
                      "params": {"bucket": {"type": "choice", "default": 1, "choices": [1, 2]}},
                      "dangerous": True},
        "close_door": {"group": "Centrifuge", "description": "Close the paired VSpin door.",
                       "params": {}, "dangerous": True},
        "stop_spin_cycle": {"group": "Centrifuge", "description": "Abort a paired VSpin spin cycle.",
                            "params": {"bucket": {"type": "choice", "default": 1, "choices": [1, 2]}},
                            "dangerous": True},
        "initialize_controller": {
            "group": "Settings",
            "description": "Run the vendor Access2 controller Initialize command.",
            "params": {},
            "dangerous": True,
        },
        "close_controller": {
            "group": "Settings",
            "description": "Run the vendor Access2 controller Close command without tearing down this driver instance.",
            "params": {},
            "dangerous": True,
        },
        "get_firmware_version": {
            "group": "Settings",
            "description": "Read the Access2 firmware version.",
            "params": {},
            "readonly": True,
        },
        "get_hardware_version": {
            "group": "Settings",
            "description": "Read the Access2 board/hardware version.",
            "params": {},
            "readonly": True,
        },
        "read_full_status": {
            "group": "Settings",
            "description": "Read full Access2 status including per-axis status and positions.",
            "params": {},
            "readonly": True,
        },
        "read_positions": {
            "group": "Settings",
            "description": "Read gripper/Y/Z positions from the controller.",
            "params": {},
            "readonly": True,
        },
        "reset_estop": {
            "group": "Settings",
            "description": "Clear the e-stop latch (after the operator presses Reset on the loader).",
            "params": {},
        },
        "reset_circuit_breaker": {
            "group": "Settings",
            "description": "Reset the named circuit breaker (loader or centrifuge).",
            "params": {
                "target": {"type": "choice", "default": "access2",
                           "choices": ["access2", "vspin"], "required": True},
            },
        },
        "use_flash": {
            "group": "Flash",
            "description": "Tell the controller to reload flash-resident settings.",
            "params": {},
            "dangerous": True,
        },
        "format_flash": {
            "group": "Flash",
            "description": "Format or restore a flash block. Use only during service recovery.",
            "params": {"block": {"type": "int", "default": 255, "required": True}},
            "dangerous": True,
        },
        "read_flash": {
            "group": "Flash",
            "description": "Read raw flash bytes by address.",
            "params": {
                "address": {"type": "int", "default": 0, "required": True},
                "length": {"type": "int", "default": 16, "required": True},
            },
            "readonly": True,
        },
        "write_flash": {
            "group": "Flash",
            "description": "Write raw flash bytes by address. Hex data, for vendor-service use only.",
            "params": {
                "address": {"type": "int", "default": 0, "required": True},
                "data_hex": {"type": "str", "default": "", "required": True},
                "apply": {"type": "bool", "default": False},
            },
            "dangerous": True,
        },
        "subscribe_nmc_information": {
            "group": "Diagnostics",
            "description": "Toggle low-level NMC diagnostic telemetry from the controller.",
            "params": {"mode": {"type": "choice", "default": "off", "choices": ["off", "tcpip", "serial"]}},
        },
        "ping": {
            "group": "Diagnostics",
            "description": "Send a vendor ping payload and read the echoed bytes.",
            "params": {"data_hex": {"type": "str", "default": "5a"}},
            "readonly": True,
        },
        "read_sensor_values": {
            "group": "Diagnostics",
            "description": "Read the vendor sensor bitfield.",
            "params": {},
            "readonly": True,
        },
        "validate_position": {
            "group": "Diagnostics",
            "description": "Validate a target axis position without changing driver state.",
            "params": {
                "axis": {"type": "choice", "choices": _ACCESS2_AXIS_CHOICES, "required": True},
                "position_mm": {"type": "float", "required": True},
            },
            "readonly": True,
        },
        "wait": {
            "group": "Diagnostics",
            "description": "Wait for the requested number of seconds.",
            "params": {"seconds": {"type": "float", "default": 1.0, "required": True}},
        },
        "raw_command": {
            "group": "Diagnostics",
            "description": "Send a raw vendor command frame by command ID and hex payload.",
            "params": {
                "cmd_id": {"type": "int", "required": True},
                "data_hex": {"type": "str", "default": ""},
                "timeout_s": {"type": "float", "default": 5.0},
            },
            "dangerous": True,
        },
        # ---- readouts ----
        "read_status": {
            "group": "Status",
            "description": "Read connected / homed / e-stop / servos / axis positions.",
            "params": {},
            "readonly": True,
        },
    }

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._homed = False
        self._estop_tripped = False
        self._servos_enabled = True
        self._optical_plate_sensor = False
        self._y_mm = _Y_MAX_MM       # park position (rear)
        self._z_mm = _Z_MAX_MM       # park position (top)
        self._gripper_mm = _GRIPPER_MAX_MM  # open
        params = self.profile.params if self.profile is not None else {}
        if not isinstance(params, dict):
            params = {}
        self._access2_motion: dict[str, Any] = _merge_dict(
            _ACCESS2_VENDOR_MOTION_DEFAULTS,
            params.get("access2_motion"),
        )
        self._access2_flash: dict[str, Any] = _merge_dict(
            _ACCESS2_VENDOR_FLASH_DEFAULTS,
            params.get("access2_flash"),
        )
        self._teachpoints: dict[str, tuple[float, float]] = dict(_TEACHPOINTS)
        self._gripper_settings: dict[str, float] = _float_mapping(
            self._access2_flash.get("gripper_teachpoints_mm"),
            _GRIPPER_SETTINGS,
        )
        self._firmware_version = "sim-Access2-1.0"
        self._hardware_version = "sim-board"
        self._flash: dict[int, int] = {}

    async def initialize(self) -> None:
        async with self._transition(DriverState.INITIALIZING):
            self._state = DriverState.IDLE
            await self._emit("state_change", {"to": "IDLE"})

    async def close(self) -> None:
        self._state = DriverState.CLOSED

    def _motion_float(self, key: str, fallback: float) -> float:
        return _float_value(self._access2_motion.get(key), fallback)

    def _motion_int(self, key: str, fallback: int) -> int:
        return _int_value(self._access2_motion.get(key), fallback)

    def _cmd_motion_float(self, cmd: Command, param: str, key: str, fallback: float) -> float:
        value = cmd.params.get(param)
        if value in (None, ""):
            return self._motion_float(key, fallback)
        return _float_value(value, fallback)

    def _cmd_motion_int(self, cmd: Command, param: str, key: str, fallback: int) -> int:
        value = cmd.params.get(param)
        if value in (None, ""):
            return self._motion_int(key, fallback)
        return _int_value(value, fallback)

    def _gripper_setting(self, key: str) -> float:
        return _float_value(self._gripper_settings.get(key), _GRIPPER_SETTINGS[key])


class SimAccess2(Access2Base):
    """In-process Access2 simulator — animates the loader arm joints.

    Emits `kinematic_state` events at ~10 Hz during motion. Axis state
    is clamped to the URDF joint limits. `complete_cycle` finds the
    paired centrifuge via scheduler.get_peer() and forwards `spin` to
    its driver, falling back to a synthetic 'spinning' wait if no peer
    is registered (e.g., in standalone unit tests).
    """

    transport = "sim"
    # Resolved at register-time by the scheduler so the sim can forward
    # spin commands to the paired centrifuge driver.
    _peer_driver_resolver: Any = None

    async def execute(self, cmd: Command) -> CommandResult:
        dispatch = {
            "home": self._sim_home,
            "park": self._sim_park,
            "load_plate": self._sim_load_plate,
            "unload_plate": self._sim_unload_plate,
            "complete_cycle": self._sim_complete_cycle,
            "move_axis_absolute": self._sim_move_axis_abs,
            "move_axis_relative": self._sim_move_axis_rel,
            "goto_teachpoint": self._sim_goto_teachpoint,
            "open_gripper": self._sim_open_gripper,
            "close_gripper": self._sim_close_gripper,
            "jog_axis": self._sim_jog_axis,
            "servo_switch": self._sim_servo_switch,
            "stop_motor": self._sim_stop_motor,
            "go_gripper_setting": self._sim_go_gripper_setting,
            "teach_teachpoint": self._sim_teach_teachpoint,
            "teach_gripper_setting": self._sim_teach_gripper_setting,
            "open_door": self._sim_open_door,
            "close_door": self._sim_close_door,
            "stop_spin_cycle": self._sim_stop_spin_cycle,
            "initialize_controller": self._sim_initialize_controller,
            "close_controller": self._sim_close_controller,
            "get_firmware_version": self._sim_get_firmware_version,
            "get_hardware_version": self._sim_get_hardware_version,
            "read_full_status": self._sim_read_full_status,
            "read_positions": self._sim_read_positions,
            "reset_estop": self._sim_reset_estop,
            "reset_circuit_breaker": self._sim_reset_cb,
            "use_flash": self._sim_use_flash,
            "format_flash": self._sim_format_flash,
            "read_flash": self._sim_read_flash,
            "write_flash": self._sim_write_flash,
            "subscribe_nmc_information": self._sim_subscribe_nmc_information,
            "ping": self._sim_ping,
            "read_sensor_values": self._sim_read_sensor_values,
            "validate_position": self._sim_validate_position,
            "wait": self._sim_wait,
            "raw_command": self._sim_raw_command,
            "read_status": self._sim_read_status,
        }
        handler = dispatch.get(cmd.kind)
        if handler is None:
            return CommandResult(ok=False, payload={"reason": f"unknown kind {cmd.kind!r}"})
        return await handler(cmd)

    # ---- motion primitives ------------------------------------------

    async def _move_to(self, *, y: float | None = None, z: float | None = None,
                       gripper: float | None = None, speed: int = 0) -> None:
        """Interpolate to (y, z, gripper) over a duration scaled by speed and
        emit kinematic_state events at 10 Hz so the viewer animates."""
        sper_mm = _SPEED_S_PER_MM.get(int(speed), _SPEED_S_PER_MM[0])
        target_y = _clamp(y if y is not None else self._y_mm, _Y_MIN_MM, _Y_MAX_MM)
        target_z = _clamp(z if z is not None else self._z_mm, _Z_MIN_MM, _Z_MAX_MM)
        target_g = _clamp(gripper if gripper is not None else self._gripper_mm,
                          _GRIPPER_MIN_MM, _GRIPPER_MAX_MM)
        dy, dz, dg = target_y - self._y_mm, target_z - self._z_mm, target_g - self._gripper_mm
        # Duration = total travel × seconds-per-mm, with a 200ms floor so
        # even no-op moves emit one frame.
        travel = max(abs(dy), abs(dz), abs(dg))
        duration = max(0.2, travel * sper_mm)
        steps = max(2, int(duration * 10))
        step_dt = duration / steps
        start_y, start_z, start_g = self._y_mm, self._z_mm, self._gripper_mm
        async with self._transition(DriverState.BUSY):
            for i in range(1, steps + 1):
                alpha = i / steps
                self._y_mm = start_y + dy * alpha
                self._z_mm = start_z + dz * alpha
                self._gripper_mm = start_g + dg * alpha
                await self.clock.sleep(step_dt)
                await self._emit("kinematic_state", _kine_payload(
                    self.clock.now(),
                    _arm_joints(self._y_mm, self._z_mm, self._gripper_mm),
                    0.0,
                ))
        self._y_mm, self._z_mm, self._gripper_mm = target_y, target_z, target_g

    # ---- handlers ---------------------------------------------------

    async def _sim_home(self, _cmd: Command) -> CommandResult:
        await self._move_to(y=_Y_MAX_MM, z=_Z_MAX_MM, gripper=_GRIPPER_MAX_MM, speed=0)
        self._homed = True
        return CommandResult(ok=True, payload={"homed": True})

    async def _sim_park(self, _cmd: Command) -> CommandResult:
        y, z = _TEACHPOINTS["park"]
        await self._move_to(y=y, z=z, gripper=_GRIPPER_MAX_MM, speed=0)
        return CommandResult(ok=True, payload={"at": "park"})

    async def _sim_goto_teachpoint(self, cmd: Command) -> CommandResult:
        name = str(cmd.params.get("name", "park"))
        if name not in self._teachpoints:
            return CommandResult(ok=False, payload={"reason": f"unknown teachpoint {name!r}"})
        y, z = self._teachpoints[name]
        await self._move_to(y=y, z=z, speed=int(cmd.params.get("speed", 0)))
        return CommandResult(ok=True, payload={"at": name})

    async def _sim_move_axis_abs(self, cmd: Command) -> CommandResult:
        axis = str(cmd.params.get("axis", "y"))
        pos = float(cmd.params.get("position_mm", 0.0))
        speed = int(cmd.params.get("speed", 0))
        kwargs: dict[str, float] = {"speed": speed}
        if axis == "y":   kwargs["y"] = pos
        elif axis == "z": kwargs["z"] = pos
        elif axis == "gripper": kwargs["gripper"] = pos
        else: return CommandResult(ok=False, payload={"reason": f"bad axis {axis!r}"})
        await self._move_to(**kwargs)
        return CommandResult(ok=True, payload=_arm_state(self._y_mm, self._z_mm, self._gripper_mm))

    async def _sim_move_axis_rel(self, cmd: Command) -> CommandResult:
        axis = str(cmd.params.get("axis", "y"))
        delta = float(cmd.params.get("delta_mm", 0.0))
        if axis == "y":
            cur = self._y_mm
        elif axis == "z":
            cur = self._z_mm
        elif axis == "gripper":
            cur = self._gripper_mm
        else:
            return CommandResult(ok=False, payload={"reason": f"bad axis {axis!r}"})
        cmd_abs = Command(kind="move_axis_absolute",
                          params={"axis": axis, "position_mm": cur + delta,
                                  "speed": int(cmd.params.get("speed", 0))})
        return await self._sim_move_axis_abs(cmd_abs)

    async def _sim_open_gripper(self, _cmd: Command) -> CommandResult:
        await self._move_to(gripper=_GRIPPER_MAX_MM, speed=1)
        return CommandResult(ok=True, payload={"gripper_mm": self._gripper_mm, "state": "open"})

    async def _sim_close_gripper(self, _cmd: Command) -> CommandResult:
        await self._move_to(gripper=_GRIPPER_MIN_MM, speed=1)
        # Plate detection: if we grip and the gripper hit a plate, the
        # sim assumes success. Real hardware would compare position
        # error to a threshold.
        self._optical_plate_sensor = True
        return CommandResult(ok=True, payload={"gripper_mm": self._gripper_mm, "state": "closed"})

    async def _sim_jog_axis(self, cmd: Command) -> CommandResult:
        return await self._sim_move_axis_rel(Command(kind="move_axis_relative", params={
            "axis": cmd.params.get("axis", "y"),
            "delta_mm": cmd.params.get("delta_mm", 0.0),
            "speed": cmd.params.get("speed", 0),
        }))

    async def _sim_servo_switch(self, cmd: Command) -> CommandResult:
        code = int(cmd.params.get("switch_code", 1))
        self._servos_enabled = code not in (0, 3)
        return CommandResult(ok=True, payload={
            "axis": str(cmd.params.get("axis", "y")),
            "switch_code": code,
            "servos_enabled": self._servos_enabled,
        })

    async def _sim_stop_motor(self, cmd: Command) -> CommandResult:
        mode = str(cmd.params.get("mode", "stop_smooth"))
        if mode == "hold_position":
            return await self._sim_jog_axis(Command(kind="jog_axis", params={
                "axis": cmd.params.get("axis", "y"),
                "delta_mm": 0.0,
                "speed": 0,
            }))
        code = {"amp_disable": 0, "motor_off": 3, "stop_abrupt": 5, "stop_smooth": 9}.get(mode, 9)
        return await self._sim_servo_switch(Command(kind="servo_switch", params={
            "axis": cmd.params.get("axis", "y"),
            "switch_code": code,
        }))

    async def _sim_go_gripper_setting(self, cmd: Command) -> CommandResult:
        setting = str(cmd.params.get("setting", "open_position"))
        pos = cmd.params.get("position_mm")
        if pos in (None, ""):
            pos = self._gripper_settings.get(setting)
        if pos is None:
            return CommandResult(ok=False, payload={"reason": f"unknown gripper setting {setting!r}"})
        vendor_pos = float(pos)
        await self._move_to(
            gripper=_sim_gripper_from_vendor(vendor_pos, self._gripper_settings),
            speed=int(cmd.params.get("speed", 0)),
        )
        return CommandResult(ok=True, payload={
            "setting": setting,
            "vendor_gripper_mm": vendor_pos,
            "gripper_mm": self._gripper_mm,
        })

    async def _sim_teach_teachpoint(self, cmd: Command) -> CommandResult:
        name = str(cmd.params.get("name", "park"))
        if name not in self._teachpoints:
            return CommandResult(ok=False, payload={"reason": f"unknown teachpoint {name!r}"})
        z = self._z_mm
        if name != "park":
            z -= float(cmd.params.get("gripper_offset_mm", 0.0))
        self._teachpoints[name] = (self._y_mm, z)
        return CommandResult(ok=True, payload={
            "name": name,
            "y_mm": self._y_mm,
            "z_mm": z,
            "commit_flash": bool(cmd.params.get("commit_flash", False)),
        })

    async def _sim_teach_gripper_setting(self, cmd: Command) -> CommandResult:
        setting = str(cmd.params.get("setting", "open_position"))
        if setting not in self._gripper_settings:
            return CommandResult(ok=False, payload={"reason": f"unknown gripper setting {setting!r}"})
        self._gripper_settings[setting] = _vendor_gripper_from_sim(
            self._gripper_mm,
            self._gripper_settings,
        )
        return CommandResult(ok=True, payload={
            "setting": setting,
            "gripper_mm": self._gripper_settings[setting],
            "commit_flash": bool(cmd.params.get("commit_flash", False)),
        })

    async def _sim_open_door(self, cmd: Command) -> CommandResult:
        peer = self._resolve_peer()
        if peer is not None and hasattr(peer, "execute"):
            return await peer.execute(Command(kind="open_door", params={}))
        return CommandResult(ok=True, payload={"door_open": True, "bucket": int(cmd.params.get("bucket", 1))})

    async def _sim_close_door(self, _cmd: Command) -> CommandResult:
        peer = self._resolve_peer()
        if peer is not None and hasattr(peer, "execute"):
            return await peer.execute(Command(kind="close_door", params={}))
        return CommandResult(ok=True, payload={"door_open": False})

    async def _sim_stop_spin_cycle(self, _cmd: Command) -> CommandResult:
        peer = self._resolve_peer()
        if peer is not None and hasattr(peer, "execute"):
            return await peer.execute(Command(kind="stop_spin", params={}))
        return CommandResult(ok=True, payload={"stopped": True})

    async def _sim_load_plate(self, cmd: Command) -> CommandResult:
        bucket = int(cmd.params.get("bucket", 1))
        speed = int(cmd.params.get("speed", 0))
        # park → stage → grip → hover → bucketN → release → hover → park
        await self._sim_goto_teachpoint(Command(kind="goto_teachpoint",
                                                params={"name": "stage", "speed": speed}))
        await self._sim_close_gripper(Command(kind="close_gripper", params={}))
        await self._sim_goto_teachpoint(Command(kind="goto_teachpoint",
                                                params={"name": "hover", "speed": speed}))
        await self._sim_goto_teachpoint(Command(kind="goto_teachpoint",
                                                params={"name": f"bucket{bucket}", "speed": speed}))
        await self._sim_open_gripper(Command(kind="open_gripper", params={}))
        self._optical_plate_sensor = False
        await self._sim_goto_teachpoint(Command(kind="goto_teachpoint",
                                                params={"name": "hover", "speed": speed}))
        await self._sim_goto_teachpoint(Command(kind="goto_teachpoint",
                                                params={"name": "park", "speed": speed}))
        return CommandResult(ok=True, payload={"bucket": bucket, "loaded": True})

    async def _sim_unload_plate(self, cmd: Command) -> CommandResult:
        bucket = int(cmd.params.get("bucket", 1))
        speed = int(cmd.params.get("speed", 0))
        # park → hover → bucketN → grip → hover → stage → release → park
        await self._sim_goto_teachpoint(Command(kind="goto_teachpoint",
                                                params={"name": "hover", "speed": speed}))
        await self._sim_goto_teachpoint(Command(kind="goto_teachpoint",
                                                params={"name": f"bucket{bucket}", "speed": speed}))
        await self._sim_close_gripper(Command(kind="close_gripper", params={}))
        await self._sim_goto_teachpoint(Command(kind="goto_teachpoint",
                                                params={"name": "hover", "speed": speed}))
        await self._sim_goto_teachpoint(Command(kind="goto_teachpoint",
                                                params={"name": "stage", "speed": speed}))
        await self._sim_open_gripper(Command(kind="open_gripper", params={}))
        await self._sim_goto_teachpoint(Command(kind="goto_teachpoint",
                                                params={"name": "park", "speed": speed}))
        return CommandResult(ok=True, payload={"bucket": bucket, "loaded": False})

    async def _sim_complete_cycle(self, cmd: Command) -> CommandResult:
        bucket = int(cmd.params.get("bucket", 1))
        rcf = float(cmd.params.get("rcf", 1000.0))
        duration_s = float(cmd.params.get("duration_s", 30.0))
        speed = int(cmd.params.get("speed", 0))
        # Load.
        await self._sim_load_plate(Command(kind="load_plate", params={
            "bucket": bucket, "speed": speed,
            "gripper_offset_mm": cmd.params.get("gripper_offset_mm", 8.0),
            "plate_height_mm": cmd.params.get("plate_height_mm", 15.0),
        }))
        # Spin: dispatch to peer centrifuge if available; otherwise just sleep.
        peer = self._resolve_peer()
        if peer is not None and hasattr(peer, "execute"):
            r = await peer.execute(Command(kind="spin",
                                           params={"rcf": rcf, "duration_s": duration_s}))
            if not r.ok:
                return CommandResult(ok=False, payload={"reason": "spin failed",
                                                        "centrifuge": r.payload})
        else:
            # No peer wired — synthesize a wait so the cycle still completes.
            await self.clock.sleep(duration_s)
        # Unload.
        await self._sim_unload_plate(Command(kind="unload_plate", params={
            "bucket": bucket, "speed": speed,
            "gripper_offset_mm": cmd.params.get("gripper_offset_mm", 8.0),
            "plate_height_mm": cmd.params.get("plate_height_mm", 15.0),
        }))
        return CommandResult(ok=True, payload={"bucket": bucket, "rcf": rcf, "duration_s": duration_s})

    async def _sim_reset_estop(self, _cmd: Command) -> CommandResult:
        self._estop_tripped = False
        return CommandResult(ok=True, payload={"estop_tripped": False})

    async def _sim_reset_cb(self, cmd: Command) -> CommandResult:
        target = str(cmd.params.get("target", "access2"))
        return CommandResult(ok=True, payload={"reset": target})

    async def _sim_initialize_controller(self, _cmd: Command) -> CommandResult:
        return CommandResult(ok=True, payload={"initialized": True})

    async def _sim_close_controller(self, _cmd: Command) -> CommandResult:
        return CommandResult(ok=True, payload={"closed_controller": True})

    async def _sim_get_firmware_version(self, _cmd: Command) -> CommandResult:
        return CommandResult(ok=True, payload={"firmware_version": self._firmware_version})

    async def _sim_get_hardware_version(self, _cmd: Command) -> CommandResult:
        return CommandResult(ok=True, payload={"hardware_version": self._hardware_version})

    async def _sim_read_positions(self, _cmd: Command) -> CommandResult:
        return CommandResult(ok=True, payload=_arm_state(self._y_mm, self._z_mm, self._gripper_mm))

    async def _sim_read_full_status(self, _cmd: Command) -> CommandResult:
        status = (await self._sim_read_status(Command(kind="read_status", params={}))).payload
        status["axis_status"] = {"y": 0, "z": 0, "gripper": 0}
        status["teachpoints"] = dict(self._teachpoints)
        status["gripper_settings"] = dict(self._gripper_settings)
        status["access2_motion"] = deepcopy(self._access2_motion)
        status["access2_flash"] = deepcopy(self._access2_flash)
        return CommandResult(ok=True, payload=status)

    async def _sim_use_flash(self, _cmd: Command) -> CommandResult:
        return CommandResult(ok=True, payload={"flash_reloaded": True})

    async def _sim_format_flash(self, cmd: Command) -> CommandResult:
        block = int(cmd.params.get("block", 255))
        self._flash.clear()
        return CommandResult(ok=True, payload={"formatted_block": block})

    async def _sim_read_flash(self, cmd: Command) -> CommandResult:
        address = int(cmd.params.get("address", 0))
        length = int(cmd.params.get("length", 16))
        data = bytes(self._flash.get(address + i, 0) for i in range(max(0, length)))
        return CommandResult(ok=True, payload={
            "address": address,
            "length": length,
            "data_hex": data.hex(),
        })

    async def _sim_write_flash(self, cmd: Command) -> CommandResult:
        address = int(cmd.params.get("address", 0))
        data = _parse_hex_bytes(str(cmd.params.get("data_hex", "")))
        for i, b in enumerate(data):
            self._flash[address + i] = b
        return CommandResult(ok=True, payload={
            "address": address,
            "length": len(data),
            "apply": bool(cmd.params.get("apply", False)),
        })

    async def _sim_subscribe_nmc_information(self, cmd: Command) -> CommandResult:
        return CommandResult(ok=True, payload={"mode": str(cmd.params.get("mode", "off"))})

    async def _sim_ping(self, cmd: Command) -> CommandResult:
        data = _parse_hex_bytes(str(cmd.params.get("data_hex", "5a")))
        return CommandResult(ok=True, payload={"echo_hex": data.hex()})

    async def _sim_read_sensor_values(self, _cmd: Command) -> CommandResult:
        sensor_values = 1 if self._optical_plate_sensor else 0
        return CommandResult(ok=True, payload={
            "sensor_values": sensor_values,
            "optical_plate_sensor": self._optical_plate_sensor,
        })

    async def _sim_validate_position(self, cmd: Command) -> CommandResult:
        axis = str(cmd.params.get("axis", "y"))
        pos = float(cmd.params.get("position_mm", 0.0))
        ranges = {"y": (_Y_MIN_MM, _Y_MAX_MM), "z": (_Z_MIN_MM, _Z_MAX_MM),
                  "gripper": (_GRIPPER_MIN_MM, _GRIPPER_MAX_MM)}
        if axis not in ranges:
            return CommandResult(ok=False, payload={"reason": f"bad axis {axis!r}"})
        lo, hi = ranges[axis]
        return CommandResult(ok=True, payload={"axis": axis, "position_mm": pos, "valid": lo <= pos <= hi})

    async def _sim_wait(self, cmd: Command) -> CommandResult:
        seconds = max(0.0, float(cmd.params.get("seconds", 1.0)))
        await self.clock.sleep(seconds)
        return CommandResult(ok=True, payload={"waited_s": seconds})

    async def _sim_raw_command(self, cmd: Command) -> CommandResult:
        return CommandResult(ok=True, payload={
            "cmd_id": int(cmd.params.get("cmd_id", 0)),
            "data_hex": str(cmd.params.get("data_hex", "")),
            "simulated": True,
        })

    async def _sim_read_status(self, _cmd: Command) -> CommandResult:
        return CommandResult(ok=True, payload={
            "connected": True,
            "homed": self._homed,
            "estop_tripped": self._estop_tripped,
            "servos_enabled": self._servos_enabled,
            "optical_plate_sensor": self._optical_plate_sensor,
            "axis_positions": {
                "y_mm": self._y_mm,
                "z_mm": self._z_mm,
                "gripper_mm": self._gripper_mm,
            },
            "peer_connected": self._resolve_peer() is not None,
        })

    # ---- peer resolution --------------------------------------------

    def _resolve_peer(self) -> Driver | None:
        """Look up the paired centrifuge driver. The scheduler installs a
        resolver callable on this attribute at register time; if absent
        (e.g., bare unit test), returns None."""
        resolver = type(self)._peer_driver_resolver
        if resolver is None:
            # Class-level default is unset; check instance.
            resolver = getattr(self, "_peer_resolver_instance", None)
        if resolver is None:
            return None
        try:
            return resolver(self)
        except Exception:
            return None

    def set_peer_resolver(self, resolver) -> None:
        """Install an instance-level peer resolver. Callable receives this
        driver and returns either the peer Driver or None."""
        self._peer_resolver_instance = resolver


# ---- back-compat shim --------------------------------------------------


class SimVSpinAutoloader(SimAccess2):
    """Legacy alias for SimAccess2 — kept so older protocols/tests that
    instantiate `SimVSpinAutoloader` continue to work. New code should
    use `SimAccess2` directly."""


class Access2Driver(Access2Base):
    """Real-hardware Access2 driver — binary packets over TCP port 7612.

    Each method dispatches to the corresponding CI_* command in
    drivers/transports_access2_tcp.py. The init sequence
    `connect → home → wait for A2SB_HOMED → park → ready` matches the
    C++ Access2Controller initialization. Untested on real hardware;
    bench bring-up (Tier C) is the validation pass."""

    transport = "tcp"

    # Map our axis strings to the AA_* binary axis addresses.
    _AXIS_MAP = {"y": 2, "z": 3, "gripper": 1}
    _SPEED_MAP = {0: 0, 1: 1, 2: 2}  # slow / medium / fast

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._link = None  # type: ignore[assignment]

    async def initialize(self) -> None:
        from vspin_cockpit.drivers.transports_access2_tcp import (
            Access2Link, DEFAULT_PORT,
        )
        host = self.profile.params.get("host") or self.profile.params.get("connection", {}).get("host")
        if not host:
            raise RuntimeError("Access2Driver requires `host` in DeviceProfile.params")
        port = int(self.profile.params.get("port", DEFAULT_PORT))
        self._link = Access2Link(host=str(host), port=port)
        async with self._transition(DriverState.INITIALIZING):
            await self._link.open()
            if bool(self.profile.params.get("probe_on_connect", False)):
                try:
                    status = await self._link.get_status_short()
                    self._homed = bool(status.get("homed"))
                except Exception:
                    pass
            self._state = DriverState.IDLE
            await self._emit("state_change", {"to": "IDLE"})

    async def close(self) -> None:
        if self._link is not None:
            await self._link.close()
        self._state = DriverState.CLOSED

    async def _ensure_link_open(self):
        link = self._link
        if link is None:
            return None
        if not link.is_open:
            await link.open()
        return link

    async def execute(self, cmd: Command) -> CommandResult:
        from vspin_cockpit.drivers.transports_access2_tcp import (
            AA_GRIPPER, PI_DYNAMIC_EMPTY, PI_GRIP_NORMALLY, SI_FAST,
            TI_BUCKET_1, TI_BUCKET_2, TI_HOVER, TI_PARK, TI_PICK,
            TransportError,
        )
        link = self._link
        if link is None:
            return CommandResult(ok=False, payload={"reason": "Access2 link not initialized"})
        readonly = bool(type(self).list_commands().get(cmd.kind, {}).get("readonly", False))
        previous_state = self._state
        mark_busy = not readonly
        if mark_busy:
            self._state = DriverState.BUSY
        try:
            link = await self._ensure_link_open()
            if link is None:
                return CommandResult(ok=False, payload={"reason": "Access2 link not initialized"})
            if cmd.kind == "home":
                home_status = await link.home()
                self._homed = bool(home_status.get("homed"))
                await link.move_to_position(
                    axis=AA_GRIPPER,
                    position_mm=self._gripper_setting("open_position"),
                    profile=PI_GRIP_NORMALLY,
                    speed=SI_FAST,
                )
                await link.move_to_location(
                    location=TI_PARK,
                    z_offset_mm=0.0,
                    plate_height_mm=self._motion_float("plate_height_mm", 15.0),
                    profile=PI_DYNAMIC_EMPTY,
                    speed=SI_FAST,
                )
                s = await link.get_status_short()
                self._homed = bool(s.get("homed"))
                payload = {**s, "at": "park", "gripper": "open"}
                if "home_acknowledged" in home_status:
                    payload["home_acknowledged"] = home_status["home_acknowledged"]
                if "home_warning" in home_status:
                    payload["home_warning"] = home_status["home_warning"]
                return CommandResult(ok=True, payload=payload)
            if cmd.kind == "park":
                await link.move_to_location(location=TI_PARK,
                                            z_offset_mm=0.0,
                                            plate_height_mm=self._motion_float("plate_height_mm", 15.0),
                                            profile=PI_DYNAMIC_EMPTY, speed=SI_FAST)
                return CommandResult(ok=True, payload={"at": "park"})
            if cmd.kind == "goto_teachpoint":
                name = str(cmd.params.get("name", "park"))
                tp_map = {"park": TI_PARK, "stage": TI_PICK,
                          "bucket1": TI_BUCKET_1, "bucket2": TI_BUCKET_2,
                          "hover": TI_HOVER}
                if name not in tp_map:
                    return CommandResult(ok=False, payload={"reason": f"unknown teachpoint {name!r}"})
                await link.move_to_location(
                    location=tp_map[name],
                    z_offset_mm=self._cmd_motion_float(cmd, "gripper_offset_mm", "gripper_z_offset_mm", 8.0),
                    plate_height_mm=self._cmd_motion_float(cmd, "plate_height_mm", "plate_height_mm", 15.0),
                    speed=self._cmd_motion_int(cmd, "speed", "speed", 0),
                )
                return CommandResult(ok=True, payload={"at": name})
            if cmd.kind == "move_axis_absolute":
                axis = self._AXIS_MAP.get(str(cmd.params.get("axis", "y")))
                if axis is None:
                    return CommandResult(ok=False, payload={"reason": "bad axis"})
                await link.move_to_position(
                    axis=axis,
                    position_mm=float(cmd.params.get("position_mm", 0.0)),
                    speed=int(cmd.params.get("speed", 0)),
                )
                return CommandResult(ok=True)
            if cmd.kind == "move_axis_relative":
                axis = self._AXIS_MAP.get(str(cmd.params.get("axis", "y")))
                if axis is None:
                    return CommandResult(ok=False, payload={"reason": "bad axis"})
                await link.jog_axis(
                    axis=axis,
                    displacement_mm=float(cmd.params.get("delta_mm", 0.0)),
                    speed=int(cmd.params.get("speed", 0)),
                )
                return CommandResult(ok=True)
            if cmd.kind == "open_gripper":
                target = self._gripper_setting("open_position")
                await link.move_to_position(
                    axis=AA_GRIPPER,
                    position_mm=target,
                    profile=PI_GRIP_NORMALLY,
                    speed=self._motion_int("speed", 0),
                )
                self._gripper_mm = target
                return CommandResult(ok=True, payload={"state": "open", "gripper_mm": target})
            if cmd.kind == "close_gripper":
                target = self._gripper_setting("close_position")
                await link.move_to_position(
                    axis=AA_GRIPPER,
                    position_mm=target,
                    profile=PI_GRIP_NORMALLY,
                    speed=self._motion_int("speed", 0),
                )
                self._gripper_mm = target
                return CommandResult(ok=True, payload={"state": "closed", "gripper_mm": target})
            if cmd.kind == "jog_axis":
                axis = self._AXIS_MAP.get(str(cmd.params.get("axis", "y")))
                if axis is None:
                    return CommandResult(ok=False, payload={"reason": "bad axis"})
                await link.jog_axis(
                    axis=axis,
                    displacement_mm=float(cmd.params.get("delta_mm", 0.0)),
                    profile=int(cmd.params.get("profile", 3)),
                    speed=int(cmd.params.get("speed", 0)),
                )
                return CommandResult(ok=True)
            if cmd.kind == "servo_switch":
                axis_name = str(cmd.params.get("axis", "y"))
                axis = self._AXIS_MAP.get(axis_name)
                if axis is None:
                    return CommandResult(ok=False, payload={"reason": "bad axis"})
                code = int(cmd.params.get("switch_code", 1))
                await link.servo_switch(axis=axis, switch_code=code)
                return CommandResult(ok=True, payload={"axis": axis_name, "switch_code": code})
            if cmd.kind == "stop_motor":
                axis_name = str(cmd.params.get("axis", "y"))
                axis = self._AXIS_MAP.get(axis_name)
                if axis is None:
                    return CommandResult(ok=False, payload={"reason": "bad axis"})
                mode = str(cmd.params.get("mode", "stop_smooth"))
                if mode == "hold_position":
                    await link.jog_axis(axis=axis, displacement_mm=0.0, profile=0, speed=0)
                    return CommandResult(ok=True, payload={"axis": axis_name, "mode": mode})
                code = {"amp_disable": 0, "motor_off": 3, "stop_abrupt": 5, "stop_smooth": 9}.get(mode, 9)
                await link.servo_switch(axis=axis, switch_code=code)
                return CommandResult(ok=True, payload={"axis": axis_name, "mode": mode, "switch_code": code})
            if cmd.kind == "go_gripper_setting":
                setting = str(cmd.params.get("setting", "open_position"))
                pos = cmd.params.get("position_mm")
                if pos in (None, ""):
                    pos = self._gripper_settings.get(setting)
                if pos is None:
                    return CommandResult(ok=False, payload={"reason": f"unknown gripper setting {setting!r}"})
                await link.move_to_position(axis=AA_GRIPPER, position_mm=float(pos),
                                            profile=PI_GRIP_NORMALLY,
                                            speed=self._cmd_motion_int(cmd, "speed", "speed", 0))
                self._gripper_mm = float(pos)
                return CommandResult(ok=True, payload={"setting": setting, "gripper_mm": self._gripper_mm})
            if cmd.kind == "teach_teachpoint":
                name = str(cmd.params.get("name", "park"))
                if name not in self._teachpoints:
                    return CommandResult(ok=False, payload={"reason": f"unknown teachpoint {name!r}"})
                self._teachpoints[name] = (self._y_mm, self._z_mm)
                return CommandResult(ok=True, payload={
                    "name": name,
                    "runtime_only": not bool(cmd.params.get("commit_flash", False)),
                    "note": "Use write_flash + use_flash to persist vendor teachpoints.",
                })
            if cmd.kind == "teach_gripper_setting":
                setting = str(cmd.params.get("setting", "open_position"))
                if setting not in self._gripper_settings:
                    return CommandResult(ok=False, payload={"reason": f"unknown gripper setting {setting!r}"})
                self._gripper_settings[setting] = self._gripper_mm
                return CommandResult(ok=True, payload={
                    "setting": setting,
                    "runtime_only": not bool(cmd.params.get("commit_flash", False)),
                    "note": "Use write_flash + use_flash to persist vendor gripper thresholds.",
                })
            if cmd.kind == "open_door":
                peer = self._resolve_peer()
                if peer is not None and hasattr(peer, "execute"):
                    return await peer.execute(Command(kind="open_door", params={}))
                return CommandResult(ok=False, payload={"reason": "paired VSpin driver is not connected"})
            if cmd.kind == "close_door":
                peer = self._resolve_peer()
                if peer is not None and hasattr(peer, "execute"):
                    return await peer.execute(Command(kind="close_door", params={}))
                return CommandResult(ok=False, payload={"reason": "paired VSpin driver is not connected"})
            if cmd.kind == "stop_spin_cycle":
                peer = self._resolve_peer()
                if peer is not None and hasattr(peer, "execute"):
                    return await peer.execute(Command(kind="stop_spin", params={}))
                return CommandResult(ok=False, payload={"reason": "paired VSpin driver is not connected"})
            if cmd.kind == "load_plate":
                # stage → grip → hover → bucketN → release → hover → park
                bucket = int(cmd.params.get("bucket", 1))
                tp = TI_BUCKET_1 if bucket == 1 else TI_BUCKET_2
                offs = self._cmd_motion_float(cmd, "gripper_offset_mm", "gripper_z_offset_mm", 8.0)
                ph = self._cmd_motion_float(cmd, "plate_height_mm", "plate_height_mm", 15.0)
                sp = self._cmd_motion_int(cmd, "speed", "speed", 0)
                await link.move_to_location(location=TI_PICK, z_offset_mm=offs,
                                            plate_height_mm=ph, speed=sp)
                await link.move_to_position(axis=AA_GRIPPER,
                                            position_mm=self._gripper_setting("close_position"),
                                            profile=PI_GRIP_NORMALLY, speed=sp)
                await link.move_to_location(location=TI_HOVER, z_offset_mm=offs,
                                            plate_height_mm=ph, speed=sp)
                await link.move_to_location(location=tp, z_offset_mm=offs,
                                            plate_height_mm=ph, speed=sp)
                await link.move_to_position(axis=AA_GRIPPER,
                                            position_mm=self._gripper_setting("open_position"),
                                            profile=PI_GRIP_NORMALLY, speed=sp)
                await link.move_to_location(location=TI_HOVER, z_offset_mm=offs,
                                            plate_height_mm=ph, speed=sp)
                await link.move_to_location(location=TI_PARK, z_offset_mm=0.0,
                                            plate_height_mm=ph, speed=sp)
                return CommandResult(ok=True, payload={"bucket": bucket, "loaded": True})
            if cmd.kind == "unload_plate":
                bucket = int(cmd.params.get("bucket", 1))
                tp = TI_BUCKET_1 if bucket == 1 else TI_BUCKET_2
                offs = self._cmd_motion_float(cmd, "gripper_offset_mm", "gripper_z_offset_mm", 8.0)
                ph = self._cmd_motion_float(cmd, "plate_height_mm", "plate_height_mm", 15.0)
                sp = self._cmd_motion_int(cmd, "speed", "speed", 0)
                await link.move_to_location(location=TI_HOVER, z_offset_mm=offs,
                                            plate_height_mm=ph, speed=sp)
                await link.move_to_location(location=tp, z_offset_mm=offs,
                                            plate_height_mm=ph, speed=sp)
                await link.move_to_position(axis=AA_GRIPPER,
                                            position_mm=self._gripper_setting("close_position"),
                                            profile=PI_GRIP_NORMALLY, speed=sp)
                await link.move_to_location(location=TI_HOVER, z_offset_mm=offs,
                                            plate_height_mm=ph, speed=sp)
                await link.move_to_location(location=TI_PICK, z_offset_mm=offs,
                                            plate_height_mm=ph, speed=sp)
                await link.move_to_position(axis=AA_GRIPPER,
                                            position_mm=self._gripper_setting("open_position"),
                                            profile=PI_GRIP_NORMALLY, speed=sp)
                await link.move_to_location(location=TI_PARK, z_offset_mm=0.0,
                                            plate_height_mm=ph, speed=sp)
                return CommandResult(ok=True, payload={"bucket": bucket, "loaded": False})
            if cmd.kind == "complete_cycle":
                # Load → forward spin to peer → unload. Same orchestration
                # as SimAccess2 but going through the real-hardware paths
                # for load/unload.
                bucket = int(cmd.params.get("bucket", 1))
                load_r = await self.execute(Command(kind="load_plate",
                                                    params=cmd.params))
                if not load_r.ok:
                    return load_r
                peer = self._resolve_peer()
                if peer is not None and hasattr(peer, "execute"):
                    spin_r = await peer.execute(Command(kind="spin", params={
                        "rcf": float(cmd.params.get("rcf", 1000.0)),
                        "duration_s": float(cmd.params.get("duration_s", 30.0)),
                    }))
                    if not spin_r.ok:
                        return CommandResult(ok=False, payload={
                            "reason": "centrifuge spin failed",
                            "centrifuge": spin_r.payload,
                        })
                else:
                    await self.clock.sleep(float(cmd.params.get("duration_s", 30.0)))
                unload_r = await self.execute(Command(kind="unload_plate",
                                                      params=cmd.params))
                if not unload_r.ok:
                    return unload_r
                return CommandResult(ok=True, payload={"bucket": bucket})
            if cmd.kind == "initialize_controller":
                await link.init()
                return CommandResult(ok=True, payload={"initialized": True})
            if cmd.kind == "close_controller":
                await link.controller_close()
                return CommandResult(ok=True, payload={"closed_controller": True})
            if cmd.kind == "get_firmware_version":
                version = await link.get_firmware_version()
                return CommandResult(ok=True, payload={"firmware_version": version})
            if cmd.kind == "get_hardware_version":
                version = await link.get_hardware_version()
                return CommandResult(ok=True, payload={"hardware_version": version})
            if cmd.kind == "read_full_status":
                s = await link.get_status()
                return CommandResult(ok=True, payload=s)
            if cmd.kind == "read_positions":
                s = await link.get_positions()
                return CommandResult(ok=True, payload=s)
            if cmd.kind == "reset_estop":
                await link.reset_estop()
                self._estop_tripped = False
                return CommandResult(ok=True)
            if cmd.kind == "reset_circuit_breaker":
                target = str(cmd.params.get("target", "access2"))
                if target == "vspin":
                    await link.reset_vspin_breaker()
                else:
                    await link.reset_access2_breaker()
                return CommandResult(ok=True, payload={"reset": target})
            if cmd.kind == "use_flash":
                await link.use_flash()
                return CommandResult(ok=True, payload={"flash_reloaded": True})
            if cmd.kind == "format_flash":
                block = int(cmd.params.get("block", 255))
                await link.format_flash(block=block)
                return CommandResult(ok=True, payload={"formatted_block": block})
            if cmd.kind == "read_flash":
                address = int(cmd.params.get("address", 0))
                length = int(cmd.params.get("length", 16))
                data = await link.read_flash(address=address, length=length)
                return CommandResult(ok=True, payload={
                    "address": address,
                    "length": length,
                    "data_hex": data.hex(),
                })
            if cmd.kind == "write_flash":
                address = int(cmd.params.get("address", 0))
                data = _parse_hex_bytes(str(cmd.params.get("data_hex", "")))
                await link.write_flash(address=address, data=data,
                                       apply=bool(cmd.params.get("apply", False)))
                return CommandResult(ok=True, payload={
                    "address": address,
                    "length": len(data),
                    "apply": bool(cmd.params.get("apply", False)),
                })
            if cmd.kind == "subscribe_nmc_information":
                mode = str(cmd.params.get("mode", "off"))
                code = {"off": 0, "serial": 1, "tcpip": 2}.get(mode, 0)
                await link.subscribe_nmc_information(switch_code=code)
                return CommandResult(ok=True, payload={"mode": mode, "switch_code": code})
            if cmd.kind == "ping":
                data = _parse_hex_bytes(str(cmd.params.get("data_hex", "5a")))
                echoed = await link.ping(data)
                return CommandResult(ok=True, payload={"echo_hex": echoed.hex()})
            if cmd.kind == "read_sensor_values":
                sensors = await link.get_sensor_values()
                return CommandResult(ok=True, payload=sensors)
            if cmd.kind == "validate_position":
                axis = str(cmd.params.get("axis", "y"))
                pos = float(cmd.params.get("position_mm", 0.0))
                gripper_min = min(self._gripper_settings.values())
                gripper_max = max(self._gripper_settings.values())
                ranges = {"y": (_Y_MIN_MM, _Y_MAX_MM), "z": (_Z_MIN_MM, _Z_MAX_MM),
                          "gripper": (gripper_min, gripper_max)}
                if axis not in ranges:
                    return CommandResult(ok=False, payload={"reason": f"bad axis {axis!r}"})
                lo, hi = ranges[axis]
                return CommandResult(ok=True, payload={"axis": axis, "position_mm": pos, "valid": lo <= pos <= hi})
            if cmd.kind == "wait":
                seconds = max(0.0, float(cmd.params.get("seconds", 1.0)))
                await self.clock.sleep(seconds)
                return CommandResult(ok=True, payload={"waited_s": seconds})
            if cmd.kind == "raw_command":
                result = await link.raw_command(
                    cmd_id=int(cmd.params.get("cmd_id", 0)),
                    data=_parse_hex_bytes(str(cmd.params.get("data_hex", ""))),
                    timeout_s=float(cmd.params.get("timeout_s", 5.0)),
                )
                return CommandResult(ok=True, payload=result)
            if cmd.kind == "read_status":
                s = await link.get_status_short()
                return CommandResult(ok=True, payload=s)
            return CommandResult(ok=False, payload={"reason": f"unknown kind {cmd.kind!r}"})
        except TransportError as exc:
            return CommandResult(ok=False, payload={"transport_error": str(exc)})
        finally:
            if mark_busy and self._state == DriverState.BUSY:
                self._state = previous_state

    def set_peer_resolver(self, resolver) -> None:
        """Install an instance-level peer resolver (same shape as
        SimAccess2 — used by complete_cycle to dispatch to the
        centrifuge driver)."""
        self._peer_resolver_instance = resolver


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _rcf_to_rpm(rcf: float, *, rotor_radius_mm: float) -> float:
    # RCF = 1.118e-5 * r * N^2 -> N = sqrt(RCF / (1.118e-5 * r))
    r_cm = rotor_radius_mm / 10.0
    return math.sqrt(rcf / (1.118e-5 * r_cm))


def _rpm_to_rcf(rpm: float, *, rotor_radius_mm: float) -> float:
    r_cm = rotor_radius_mm / 10.0
    return 1.118e-5 * r_cm * (rpm ** 2)


def _kine_payload(t: float, cfg: dict[str, float], omega: float) -> dict[str, Any]:
    return {"t": t, "joints": dict(cfg), "omega": omega}


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _arm_joints(y_mm: float, z_mm: float, gripper_mm: float) -> dict[str, float]:
    """URDF prismatic joints take metres, so we divide by 1000. The two
    finger joints mirror each other around the gripper centre."""
    # Gripper finger position: half the gripper opening, mirrored across the
    # finger limit ranges. We map gripper_mm in [14, 32] linearly into the
    # left finger limits [0.014, 0.032] and the right finger limits
    # [0.030, 0.016] (mirrored).
    g_norm = (gripper_mm - _GRIPPER_MIN_MM) / (_GRIPPER_MAX_MM - _GRIPPER_MIN_MM)
    g_norm = max(0.0, min(1.0, g_norm))
    left  = 0.014 + g_norm * (0.032 - 0.014)
    right = 0.030 - g_norm * (0.030 - 0.016)
    return {
        "dof_y_axis": y_mm / 1000.0,
        "dof_z_axis": z_mm / 1000.0,
        "dof_left_finger": left,
        "dof_right_finger": right,
    }


def _arm_state(y_mm: float, z_mm: float, gripper_mm: float) -> dict[str, float]:
    return {"y_mm": y_mm, "z_mm": z_mm, "gripper_mm": gripper_mm}


def _merge_dict(defaults: dict[str, Any], overrides: Any) -> dict[str, Any]:
    result = deepcopy(defaults)
    if not isinstance(overrides, dict):
        return result
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge_dict(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def _float_value(value: Any, fallback: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(fallback)


def _int_value(value: Any, fallback: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(fallback)


def _float_mapping(value: Any, defaults: dict[str, float]) -> dict[str, float]:
    result = dict(defaults)
    if not isinstance(value, dict):
        return result
    for key in defaults:
        if key in value:
            result[key] = _float_value(value[key], defaults[key])
    return result


def _bucket_teachpoints(value: Any, defaults: dict[int, int]) -> dict[int, int]:
    result = dict(defaults)
    if not isinstance(value, dict):
        return result
    for bucket in tuple(result):
        candidates = (bucket, str(bucket), f"bucket{bucket}", f"bucket_{bucket}")
        for key in candidates:
            if key in value:
                result[bucket] = _int_value(value[key], result[bucket])
                break
    return result


def _sim_gripper_from_vendor(pos: float, settings: dict[str, float]) -> float:
    open_pos = _float_value(settings.get("open_position"), _GRIPPER_SETTINGS["open_position"])
    close_pos = _float_value(settings.get("close_position"), _GRIPPER_SETTINGS["close_position"])
    span = close_pos - open_pos
    if abs(span) < 1e-9:
        return _GRIPPER_MAX_MM
    closed_fraction = _clamp((pos - open_pos) / span, 0.0, 1.0)
    return _GRIPPER_MAX_MM + closed_fraction * (_GRIPPER_MIN_MM - _GRIPPER_MAX_MM)


def _vendor_gripper_from_sim(pos: float, settings: dict[str, float]) -> float:
    open_pos = _float_value(settings.get("open_position"), _GRIPPER_SETTINGS["open_position"])
    close_pos = _float_value(settings.get("close_position"), _GRIPPER_SETTINGS["close_position"])
    sim_span = _GRIPPER_MIN_MM - _GRIPPER_MAX_MM
    if abs(sim_span) < 1e-9:
        return open_pos
    closed_fraction = _clamp((pos - _GRIPPER_MAX_MM) / sim_span, 0.0, 1.0)
    return open_pos + closed_fraction * (close_pos - open_pos)


def _parse_hex_bytes(value: str) -> bytes:
    cleaned = "".join(ch for ch in value.strip() if ch not in " \t\r\n:-")
    if not cleaned:
        return b""
    if len(cleaned) % 2:
        cleaned = "0" + cleaned
    return bytes.fromhex(cleaned)
