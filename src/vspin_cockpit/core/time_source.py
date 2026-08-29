"""Clock abstraction — real-time only.

The drivers use ``self.clock.sleep(...)`` instead of ``asyncio.sleep`` so
a simulated clock could fast-forward between events; this standalone
app only ever runs in real time, so we ship just :class:`WallClock`.
Reintroduce ``SimulatedClock`` if you ever want to unit-test a driver's
timing without waiting on wall-time.
"""

from __future__ import annotations

import asyncio
import time
from typing import Protocol, runtime_checkable


@runtime_checkable
class Clock(Protocol):
    def now(self) -> float: ...
    async def sleep(self, seconds: float) -> None: ...


class WallClock:
    """Real-time clock backed by ``time.monotonic`` and ``asyncio.sleep``."""

    def now(self) -> float:
        return time.monotonic()

    async def sleep(self, seconds: float) -> None:
        await asyncio.sleep(seconds)
