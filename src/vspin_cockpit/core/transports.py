"""Transport error hierarchy shared by the hardware driver modules.

The two hardware transports this app ships (``transports_access2_tcp``
for the loader, ``transports_nmc`` for the VSpin serial protocol) frame
their own binary packets over asyncio-managed sockets/serial ports — they
don't need a ``SerialLink`` / ``TcpLink`` wrapper. So this module carries
just the exception types they raise on I/O trouble.
"""

from __future__ import annotations


class TransportError(RuntimeError):
    """Anything that goes wrong at the wire level — connection refused,
    EOF, framing error, timeout."""


class ProtocolError(TransportError):
    """Link is up but the device sent something we could not parse.

    Kept distinct from a bare :class:`TransportError` so callers can
    retry a single bad frame (flush + reissue) without treating the
    whole link as gone.
    """
