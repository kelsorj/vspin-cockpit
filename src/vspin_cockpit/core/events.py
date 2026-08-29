"""In-process pub/sub bus. Drivers emit :class:`DeviceEvent`s; the app
subscribes via :meth:`EventBus.subscribe` and republishes to the SSE
endpoint.

Deliberately narrow — dropped from the general-purpose form: the
``Literal`` union of event kinds (drivers emit whatever string tag they
like here) and the ``wait_for()`` convenience helper.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import anyio
from anyio.streams.memory import MemoryObjectReceiveStream, MemoryObjectSendStream


@dataclass(frozen=True, slots=True)
class DeviceEvent:
    """One event on the bus. ``kind`` is a free-form tag — the two
    interesting ones for the cockpit are ``"kinematic_state"`` (per-frame
    joint values, emitted at ~10 Hz during motion) and ``"state_change"``
    (whenever a driver transitions between UNINITIALIZED / IDLE /
    BUSY / ERROR)."""

    device_id: str
    kind: str
    timestamp: float
    payload: dict[str, Any] = field(default_factory=dict)


Predicate = Callable[[DeviceEvent], bool]


class _Subscription:
    __slots__ = ("predicate", "send")

    def __init__(self, predicate: Predicate, send: MemoryObjectSendStream[DeviceEvent]) -> None:
        self.predicate = predicate
        self.send = send


class EventBus:
    """Fan-out. Every publish snapshots the sub list under a lock, then
    non-blocking-sends into each matching subscriber. A subscriber that
    can't keep up drops frames instead of stalling the publisher — the
    cockpit's SSE consumer is designed to tolerate drops, and the
    ``read_status`` poll fills any gap."""

    def __init__(self, buffer_size: int = 256) -> None:
        self._subs: list[_Subscription] = []
        self._buffer_size = buffer_size
        self._lock = anyio.Lock()

    async def publish(self, event: DeviceEvent) -> None:
        async with self._lock:
            subs = list(self._subs)
        dead: list[_Subscription] = []
        for sub in subs:
            if not sub.predicate(event):
                continue
            try:
                sub.send.send_nowait(event)
            except anyio.WouldBlock:
                # Slow subscriber — drop rather than stall the publisher.
                pass
            except anyio.ClosedResourceError:
                dead.append(sub)
        if dead:
            async with self._lock:
                for sub in dead:
                    if sub in self._subs:
                        self._subs.remove(sub)

    def subscribe(
        self, predicate: Predicate | None = None, *, buffer_size: int | None = None,
    ) -> "_SubscriptionCM":
        return _SubscriptionCM(
            self, predicate or (lambda _e: True), buffer_size or self._buffer_size,
        )

    async def _add_sub(self, sub: _Subscription) -> None:
        async with self._lock:
            self._subs.append(sub)

    async def _remove_sub(self, sub: _Subscription) -> None:
        async with self._lock:
            if sub in self._subs:
                self._subs.remove(sub)


class _SubscriptionCM:
    """``async with bus.subscribe(...) as stream:`` yields a stream you
    can ``async for`` over. Cleanup on exit removes the sub and closes
    both ends of the memory stream — no leak even on an early break."""

    def __init__(self, bus: EventBus, predicate: Predicate, buffer_size: int) -> None:
        self._bus = bus
        self._predicate = predicate
        self._buffer_size = buffer_size
        self._send: MemoryObjectSendStream[DeviceEvent] | None = None
        self._recv: MemoryObjectReceiveStream[DeviceEvent] | None = None
        self._sub: _Subscription | None = None

    async def __aenter__(self) -> MemoryObjectReceiveStream[DeviceEvent]:
        self._send, self._recv = anyio.create_memory_object_stream[DeviceEvent](self._buffer_size)
        self._sub = _Subscription(self._predicate, self._send)
        await self._bus._add_sub(self._sub)
        return self._recv

    async def __aexit__(self, *exc: Any) -> None:
        if self._sub is not None:
            await self._bus._remove_sub(self._sub)
        if self._send is not None:
            await self._send.aclose()
        if self._recv is not None:
            await self._recv.aclose()
