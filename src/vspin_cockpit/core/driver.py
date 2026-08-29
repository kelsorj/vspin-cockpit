"""Driver ABC + the value types dispatched to it.

Deliberately narrow — kept to just what the Access2 + VSpin drivers
this app ships actually use. Cut:

  * The plate-transfer handshake (``plate_picked_up``,
    ``plate_dropped_off``, ``plate_transfer_aborted``, location
    availability) — nothing here moves plates through a schedule.
  * Role protocols (``RobotDriver``, ``StorageDriver``, ...) — the app
    talks to two known devices, not a role catalog.
  * ``ErrorInfo`` / ``HealthReport`` / ``DriverMetadata`` — the UI
    surfaces state via ``read_status`` payloads.

If you ever want to grow this repo into a general-purpose driver harness,
this file is the right seam to widen first.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from contextlib import asynccontextmanager, suppress
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, ClassVar, Literal

import anyio
from anyio.streams.memory import MemoryObjectReceiveStream, MemoryObjectSendStream

from vspin_cockpit.core.events import DeviceEvent, EventBus
from vspin_cockpit.core.time_source import Clock, WallClock


class DriverState(Enum):
    UNINITIALIZED = auto()
    INITIALIZING = auto()
    IDLE = auto()
    BUSY = auto()
    ERROR = auto()
    RECOVERING = auto()
    CLOSED = auto()


Transport = Literal["serial", "tcp", "http", "sim"]


@dataclass(slots=True)
class DeviceProfile:
    """Per-instance configuration — id/name/role plus a free-form params
    dict the driver reads at init time (host/port/serial-port/etc.)."""

    id: str
    name: str
    role: str
    transport: Transport = "sim"
    params: dict[str, Any] = field(default_factory=dict)
    simulated: bool = False


@dataclass(slots=True)
class Command:
    """A unit of work dispatched to a driver. The driver's ``execute()``
    branches on ``kind``; ``params`` is the argument dict."""

    kind: str
    params: dict[str, Any] = field(default_factory=dict)
    # `location_id` is kept for shape-compatibility with the original
    # command struct in case a driver method still peeks at it. The
    # cockpit never sets it.
    location_id: str | None = None


@dataclass(slots=True)
class CommandResult:
    ok: bool = True
    payload: dict[str, Any] = field(default_factory=dict)


@dataclass(slots=True)
class ErrorContext:
    cmd: Command | None
    message: str
    payload: dict[str, Any] = field(default_factory=dict)


# --- Error hierarchy (three tiers, same names as the original) -------------


class DeviceFaultError(RuntimeError):
    """Device reported an unrecoverable fault (e-stop, sensor fault,
    calibration loss). Raise from ``execute()`` to force the operator to
    intervene."""


class InitializationError(RuntimeError):
    """``initialize()`` could not bring the driver to IDLE — handshake,
    version check, or first contact failed."""


class CommandFailedError(RuntimeError):
    """Synthesized when a driver returns ``CommandResult(ok=False)``.
    Drivers themselves should return the falsy result rather than
    raising this directly — it's here so calling code can catch and
    re-raise uniformly."""

    def __init__(
        self, reason: str, *, payload: dict[str, Any] | None = None,
        suggested_action: str = "retry",
    ) -> None:
        super().__init__(reason)
        self.reason = reason
        self.payload = payload or {}
        self.suggested_action = suggested_action


# --- Driver ABC ------------------------------------------------------------


class Driver(ABC):
    """Async base class for every driver.

    Subclasses declare :attr:`supported_commands` — the UI introspects
    it to figure out what parameter schema each command exposes. The
    shape is::

        supported_commands = {
            "<kind>": {
                "group": "Control",       # UI grouping label
                "description": "...",
                "params": {
                    "<name>": {"type": "int|float|bool|choice", "default": ..., ...},
                },
                "readonly": False,        # optional
                "dangerous": False,       # optional — UI can prompt confirm
            },
            ...
        }
    """

    role: ClassVar[str] = "unknown"
    transport: ClassVar[Transport] = "sim"
    supported_commands: ClassVar[dict[str, dict[str, Any]]] = {}

    @classmethod
    def list_commands(cls) -> dict[str, dict[str, Any]]:
        return dict(cls.supported_commands)

    def __init__(
        self, profile: DeviceProfile, bus: EventBus, clock: Clock | None = None,
    ) -> None:
        self.profile = profile
        self.bus = bus
        self.clock: Clock = clock or WallClock()
        self._state = DriverState.UNINITIALIZED
        self._last_error: str | None = None
        self._last_command: Command | None = None
        # Every driver owns a private receive stream. In this cockpit the
        # app spawns a per-slot pump task that forwards from here onto
        # the shared bus, so SSE subscribers see the frames.
        self._events_send: MemoryObjectSendStream[DeviceEvent]
        self.events: MemoryObjectReceiveStream[DeviceEvent]
        self._events_send, self.events = anyio.create_memory_object_stream[DeviceEvent](64)

    # ---- lifecycle ----

    @abstractmethod
    async def initialize(self) -> None: ...

    @abstractmethod
    async def close(self) -> None: ...

    # ---- commands ----

    @abstractmethod
    async def execute(self, cmd: Command) -> CommandResult: ...

    # ---- error recovery (optional hooks) ----

    async def abort(self, ctx: ErrorContext) -> None:
        self._state = DriverState.ERROR

    async def ignore(self, ctx: ErrorContext) -> None:
        return None

    async def retry(self, ctx: ErrorContext) -> None:
        if ctx.cmd is not None:
            await self.execute(ctx.cmd)

    # ---- state-machine helper ----

    @asynccontextmanager
    async def _transition(self, target: DriverState):
        self._state = target
        try:
            yield
        except Exception:
            self._state = DriverState.ERROR
            raise
        else:
            # If the caller held BUSY and didn't set a more specific state,
            # revert to IDLE. Anything else the caller set is left alone.
            if self._state == target == DriverState.BUSY:
                self._state = DriverState.IDLE

    async def _emit(self, kind: str, payload: dict[str, Any] | None = None) -> None:
        ev = DeviceEvent(
            device_id=self.profile.id,
            kind=kind,
            timestamp=self.clock.now(),
            payload=payload or {},
        )
        # Drop on backpressure — a slow consumer must not stall the driver.
        with suppress(anyio.WouldBlock):
            self._events_send.send_nowait(ev)
