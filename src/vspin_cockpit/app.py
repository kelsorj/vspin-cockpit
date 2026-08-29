"""Standalone application: Access2 loader + VSpin centrifuge + URDF twin.

Runs a small FastAPI server bound to loopback that instantiates the
:class:`~vspin_cockpit.drivers.vspin.SimAccess2` /
:class:`~vspin_cockpit.drivers.vspin.SimVSpin` drivers by default (or the
real :class:`~vspin_cockpit.drivers.vspin.Access2Driver` /
:class:`~vspin_cockpit.drivers.vspin.VSpinDriver` with ``--real``) and
serves ONE HTML page combining:

  * A 3D URDF viewer using ``URDF/vspin_access_2_urdf/urdf/
    vspin_and_loader_urdf.urdf`` (both centrifuge body and loader arm as
    one twin). Joints animate live from ``kinematic_state`` events
    streamed via SSE, backed by a 500 ms ``read_status`` poll.
  * Access2 control panel — jog pad, teachpoints, load / unload,
    full-cycle button.
  * VSpin control panel — RCF/RPM entry, spin, door + bucket controls.
  * Shared, colour-coded activity log.

Design
======

In a general-purpose driver harness, a scheduler owns a receive stream
per driver and republishes onto a shared
:class:`~vspin_cockpit.core.events.EventBus`. This standalone app has
no scheduler; instead it starts one pump task per slot that forwards
``driver.events`` onto the bus. That's the smallest slice of
scheduler-shape the SSE endpoint needs to see frames.

Run
===

    python -m vspin_cockpit                              # sim, opens browser
    python -m vspin_cockpit --no-browser
    python -m vspin_cockpit --real \\
        --access2-host 192.168.0.66 --vspin-port /dev/tty.usbserial-A1
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import threading
import time
import webbrowser
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import anyio
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from vspin_cockpit.core.driver import Command, DeviceProfile
from vspin_cockpit.core.events import EventBus
from vspin_cockpit.drivers.vspin import (
    Access2Base,
    Access2Driver,
    SimAccess2,
    SimVSpin,
    VSpinBase,
    VSpinDriver,
)

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_APP_DIR = Path(__file__).resolve().parent
_STATIC_DIR = _APP_DIR / "static"

# The combined VSpin + Access2 URDF ships at <repo>/URDF/vspin_access_2_urdf.
# When installed from a wheel this folder isn't packaged (it's ~4MB of gltf
# meshes and belongs beside the source, not inside the package). We walk
# up from src/vspin_cockpit/ to find the repo root at install time.
# `--urdf-root` overrides this at the CLI.
_REPO_ROOT_DEFAULT = _APP_DIR.parents[1]  # .../vspin-cockpit/
_URDF_DIR_DEFAULT = _REPO_ROOT_DEFAULT / "URDF" / "vspin_access_2_urdf"

log = logging.getLogger("vspin_cockpit")


# ---------------------------------------------------------------------------
# Runtime state
# ---------------------------------------------------------------------------


@dataclass
class DriverSlot:
    """One live driver (loader OR centrifuge) with a mutable instance so
    the ``Connect`` button can swap sim ↔ real without restarting.

    ``pump_task`` is the asyncio Task that forwards this driver's
    private event stream (``driver.events``) into the shared
    :class:`EventBus`. Without it, ``kinematic_state`` frames never
    reach the SSE endpoint.
    """

    name: str
    label: str
    driver: Any | None = None
    simulated: bool = True
    connected: bool = False
    connection: dict[str, Any] = field(default_factory=dict)
    last_error: str | None = None
    pump_task: asyncio.Task | None = None


@dataclass
class AppRuntime:
    bus: EventBus
    access2: DriverSlot
    vspin: DriverSlot
    urdf_root: Path
    default_sim: bool = True
    default_access2_host: str = "192.168.0.66"
    default_access2_port: int = 7612
    default_vspin_port: str = ""
    default_vspin_baud: int = 57600


def _make_bus() -> EventBus:
    """Bus buffer sized larger than default so a slow SSE client doesn't
    starve the sim's ~10 Hz kinematic frames."""
    return EventBus(buffer_size=1024)


# ---------------------------------------------------------------------------
# Driver construction
# ---------------------------------------------------------------------------


def _make_access2(bus: EventBus, *, simulated: bool, host: str, port: int) -> Access2Base:
    profile = DeviceProfile(
        id="access2",
        name="Access2 Loader",
        role="centrifuge_loader",
        transport="sim" if simulated else "tcp",
        params={} if simulated else {"host": host, "port": port},
        simulated=simulated,
    )
    if simulated:
        return SimAccess2(profile=profile, bus=bus)
    return Access2Driver(profile=profile, bus=bus)


def _make_vspin(bus: EventBus, *, simulated: bool, port: str, baud: int) -> VSpinBase:
    profile = DeviceProfile(
        id="vspin",
        name="VSpin Centrifuge",
        role="centrifuge",
        transport="sim" if simulated else "serial",
        params={} if simulated else {"port": port, "baudrate": baud},
        simulated=simulated,
    )
    if simulated:
        return SimVSpin(profile=profile, bus=bus)
    return VSpinDriver(profile=profile, bus=bus)


async def _pump_events(slot_name: str, driver: Any, bus: EventBus) -> None:
    """Forward every event the driver emits on its private receive
    stream onto the shared bus. Runs until the stream is closed."""
    try:
        async for ev in driver.events:
            await bus.publish(ev)
    except (anyio.EndOfStream, anyio.ClosedResourceError):
        pass  # driver.close() closed the send end; normal shutdown
    except Exception as exc:  # pragma: no cover
        log.warning("event pump for %s died: %s: %s",
                    slot_name, type(exc).__name__, exc)


async def _connect_slot(rt: AppRuntime, slot: DriverSlot, factory) -> dict[str, Any]:
    """Close whatever's in the slot, build a new driver, initialize it,
    start the event pump. Idempotent — a reconnect closes cleanly first."""
    await _disconnect_slot(slot)
    driver = factory()
    slot.driver = driver
    try:
        await driver.initialize()
    except Exception as exc:
        slot.connected = False
        slot.last_error = f"{type(exc).__name__}: {exc}"
        # Keep the instance so read_status can still be probed — some
        # drivers report useful info from ERROR state.
        return {"ok": False, "error": slot.last_error}
    slot.connected = True
    slot.last_error = None
    slot.simulated = driver.profile.simulated
    # Peer resolver re-installed after every (re)connect so complete_cycle
    # dispatches spin to whichever VSpin driver is CURRENTLY loaded (the
    # closure captures the runtime, not the driver, so swapping the VSpin
    # slot updates the loader's peer automatically).
    if slot.name == "access2" and isinstance(driver, Access2Base):
        driver.set_peer_resolver(lambda _self: rt.vspin.driver)
    slot.pump_task = asyncio.create_task(
        _pump_events(slot.name, driver, rt.bus),
        name=f"vspin-cockpit-pump-{slot.name}",
    )
    return {"ok": True, "simulated": slot.simulated, "transport": driver.transport}


async def _disconnect_slot(slot: DriverSlot) -> dict[str, Any]:
    driver = slot.driver
    pump = slot.pump_task
    slot.driver = None
    slot.pump_task = None
    slot.connected = False
    if driver is None:
        return {"ok": True, "was_loaded": False}
    try:
        await driver.close()
    except Exception as exc:
        log.warning("close(%s) raised %s: %s", slot.name, type(exc).__name__, exc)
    if pump is not None and not pump.done():
        pump.cancel()
        with contextlib.suppress(asyncio.CancelledError, BaseException):
            await pump
    return {"ok": True, "was_loaded": True}


# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------


def make_app(rt: AppRuntime) -> FastAPI:
    app = FastAPI(title="vspin_cockpit · Access2 + VSpin", version="0.1.0")

    app.mount("/static", StaticFiles(directory=str(_STATIC_DIR)), name="static")
    # URDF mesh files are referenced by the URDF as
    # `package://<any>/meshes/foo.gltf`; the browser-side URDFLoader
    # remaps any package to this mount root so relative paths resolve.
    app.mount("/urdf", StaticFiles(directory=str(rt.urdf_root)), name="urdf")

    @app.get("/", response_class=HTMLResponse)
    async def index() -> HTMLResponse:
        return HTMLResponse((_STATIC_DIR / "index.html").read_text(encoding="utf-8"))

    @app.get("/health")
    async def health() -> dict[str, Any]:
        return {
            "ok": True,
            "access2": _slot_summary(rt.access2),
            "vspin": _slot_summary(rt.vspin),
        }

    @app.get("/devices")
    async def list_devices() -> list[dict[str, Any]]:
        return [_slot_summary(rt.access2), _slot_summary(rt.vspin)]

    @app.get("/devices/{name}/commands")
    async def list_commands(name: str) -> dict[str, Any]:
        slot = _slot_for(rt, name)
        drv = slot.driver
        if drv is None:
            klass = SimAccess2 if slot.name == "access2" else SimVSpin
            return {"device": slot.name, "commands": klass.list_commands()}
        return {"device": slot.name, "commands": drv.__class__.list_commands()}

    @app.post("/devices/{name}/connect")
    async def connect(name: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        slot = _slot_for(rt, name)
        body = body or {}
        simulated = bool(body.get("simulated", slot.simulated))
        if slot.name == "access2":
            host = str(body.get("host") or rt.default_access2_host)
            port = int(body.get("port") or rt.default_access2_port)
            slot.connection = {"host": host, "port": port}
            factory = lambda: _make_access2(rt.bus, simulated=simulated, host=host, port=port)
        else:
            port = str(body.get("port") or rt.default_vspin_port or "")
            baud = int(body.get("baudrate") or rt.default_vspin_baud)
            slot.connection = {"port": port, "baudrate": baud}
            factory = lambda: _make_vspin(rt.bus, simulated=simulated, port=port, baud=baud)
        return await _connect_slot(rt, slot, factory)

    @app.post("/devices/{name}/disconnect")
    async def disconnect(name: str) -> dict[str, Any]:
        return await _disconnect_slot(_slot_for(rt, name))

    @app.post("/devices/{name}/execute")
    async def execute(name: str, body: dict[str, Any]) -> dict[str, Any]:
        slot = _slot_for(rt, name)
        drv = slot.driver
        if drv is None:
            raise HTTPException(status_code=409, detail=f"{name} not connected")
        kind = (body or {}).get("kind")
        if not kind:
            raise HTTPException(status_code=400, detail="missing `kind` in body")
        raw_params = (body or {}).get("params") or {}
        spec = drv.__class__.list_commands().get(str(kind), {})
        coerced = _coerce_params(raw_params, spec.get("params", {}))
        try:
            result = await drv.execute(Command(kind=str(kind), params=coerced))
        except Exception as exc:
            return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        return {"ok": bool(result.ok), "payload": dict(result.payload or {})}

    @app.get("/events")
    async def events_stream(request: Request) -> StreamingResponse:
        """SSE stream — one event per DeviceEvent the drivers publish,
        filtered to ``kinematic_state`` + ``state_change``. Closes cleanly
        on client disconnect."""

        async def gen():
            yield b": vspin-cockpit-stream\n\n"
            async with rt.bus.subscribe(
                lambda e: e.kind in {"kinematic_state", "state_change"},
            ) as stream:
                async for ev in stream:
                    if await request.is_disconnected():
                        break
                    data = {
                        "device_id": ev.device_id,
                        "kind": ev.kind,
                        "timestamp": ev.timestamp,
                        "payload": ev.payload,
                    }
                    yield f"event: {ev.kind}\ndata: {json.dumps(data)}\n\n".encode("utf-8")

        return StreamingResponse(
            gen(),
            media_type="text/event-stream",
            headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
        )

    @app.get("/urdf-info")
    async def urdf_info() -> dict[str, Any]:
        urdf_file = rt.urdf_root / "urdf" / "vspin_and_loader_urdf.urdf"
        if not urdf_file.exists():
            raise HTTPException(status_code=500, detail=f"URDF missing at {urdf_file}")
        return {
            "urdf_url": "/urdf/urdf/vspin_and_loader_urdf.urdf",
            "device_url": "/urdf",
            "joints": [
                "dof_y_axis", "dof_z_axis",
                "dof_left_finger", "dof_right_finger",
                "dof_rotor",
            ],
        }

    @app.get("/api/discover/access2")
    async def discover_access2(
        timeout_s: float = 2.0,
        broadcast: str | None = None,
    ) -> dict[str, Any]:
        """UDP-broadcast on port 7611 (Velocity11 / Agilent discovery
        protocol) and return any Ethernet-equipped Access2 loaders /
        BlueWashers / Bravos that answer. Falls back to a TCP-probe sweep
        of the same subnet if the UDP path returns nothing (some hardware
        has broadcast filtered off).

        Returns discovered devices plus ``diag`` — the broadcast addresses
        tried, per-address send errors, and packet counts — for
        diagnosing "device is on the bench but nothing was found" cases.
        """
        from vspin_cockpit.drivers.discovery_v11 import (
            discover_access2_tcp_devices,
            discover_v11_devices,
        )
        try:
            found, diag = await discover_v11_devices(
                timeout_s=max(0.2, min(10.0, float(timeout_s))),
                broadcast_address=str(broadcast) if broadcast else None,
            )
            if not found:
                tcp_found, tcp_diag = await discover_access2_tcp_devices(
                    timeout_s=0.25,
                    broadcast_address=str(broadcast) if broadcast else None,
                )
                diag["tcp_fallback"] = tcp_diag
                found_by_addr = {(d.ip, d.port): d for d in found}
                for dev in tcp_found:
                    found_by_addr.setdefault((dev.ip, dev.port), dev)
                found = sorted(
                    found_by_addr.values(),
                    key=lambda d: tuple(int(x) for x in d.ip.split(".")),
                )
        except Exception as exc:
            raise HTTPException(
                status_code=500,
                detail=f"discovery failed: {type(exc).__name__}: {exc}",
            ) from exc
        return {
            "devices": [
                {"ip": d.ip, "port": d.port, "type": d.type, "display_name": d.display_name}
                for d in found
            ],
            "timeout_s": timeout_s,
            "diag": diag,
        }

    return app


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _slot_for(rt: AppRuntime, name: str) -> DriverSlot:
    if name == "access2":
        return rt.access2
    if name == "vspin":
        return rt.vspin
    raise HTTPException(status_code=404, detail=f"unknown device {name!r}")


def _slot_summary(slot: DriverSlot) -> dict[str, Any]:
    drv = slot.driver
    return {
        "name": slot.name,
        "label": slot.label,
        "connected": slot.connected,
        "simulated": slot.simulated,
        "connection": dict(slot.connection),
        "transport": drv.transport if drv is not None else None,
        "last_error": slot.last_error,
    }


def _coerce_params(raw: dict[str, Any], schema: dict[str, Any]) -> dict[str, Any]:
    """Convert JSON scalars into the driver's expected Python types per
    the ``supported_commands`` schema."""
    out: dict[str, Any] = {}
    for key, val in (raw or {}).items():
        t = str(schema.get(key, {}).get("type", "str"))
        try:
            if val is None or val == "":
                out[key] = val
            elif t == "int":
                out[key] = int(val)
            elif t == "float":
                out[key] = float(val)
            elif t == "bool":
                if isinstance(val, bool):
                    out[key] = val
                elif isinstance(val, str):
                    out[key] = val.strip().lower() in {"1", "true", "yes", "on"}
                else:
                    out[key] = bool(val)
            else:
                out[key] = val
        except (TypeError, ValueError):
            out[key] = val
    return out


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _build_runtime(args: argparse.Namespace) -> AppRuntime:
    urdf_root = Path(args.urdf_root).resolve()
    if not (urdf_root / "urdf" / "vspin_and_loader_urdf.urdf").exists():
        raise SystemExit(
            f"URDF not found under {urdf_root}. Pass --urdf-root <path> pointing "
            "at the folder containing urdf/vspin_and_loader_urdf.urdf."
        )
    bus = _make_bus()
    return AppRuntime(
        bus=bus,
        urdf_root=urdf_root,
        default_sim=not args.real,
        default_access2_host=args.access2_host,
        default_access2_port=args.access2_port,
        default_vspin_port=args.vspin_port,
        default_vspin_baud=args.vspin_baud,
        access2=DriverSlot(name="access2", label="Access2 Loader"),
        vspin=DriverSlot(name="vspin", label="VSpin Centrifuge"),
    )


async def _initial_connect(rt: AppRuntime) -> None:
    """Pre-connect both drivers on boot so the UI shows live state
    without the operator having to click Connect. A failing connect in
    ``--real`` mode just leaves the slot disconnected and the error on
    the header pill."""
    sim = rt.default_sim
    await _connect_slot(
        rt, rt.vspin,
        lambda: _make_vspin(
            rt.bus, simulated=sim,
            port=rt.default_vspin_port, baud=rt.default_vspin_baud,
        ),
    )
    await _connect_slot(
        rt, rt.access2,
        lambda: _make_access2(
            rt.bus, simulated=sim,
            host=rt.default_access2_host, port=rt.default_access2_port,
        ),
    )


def _open_browser_soon(url: str, *, delay_s: float = 0.8) -> None:
    def _open() -> None:
        time.sleep(delay_s)
        try:
            webbrowser.open_new_tab(url)
        except Exception as exc:  # pragma: no cover
            log.warning("Could not open browser: %s", exc)

    threading.Thread(target=_open, daemon=True, name="vspin-cockpit-open-browser").start()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="python -m vspin_cockpit",
        description="Standalone Access2 + VSpin cockpit with URDF twin.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Bind address.")
    parser.add_argument("--port", type=int, default=8765, help="TCP port to listen on.")
    parser.add_argument("--no-browser", action="store_true",
                        help="Skip auto-opening the default browser.")
    parser.add_argument("--real", action="store_true",
                        help="Connect to real hardware on start (default: simulator).")
    parser.add_argument("--sim", action="store_true",
                        help="Force simulator on start (the default; explicit for scripts).")
    parser.add_argument("--access2-host", default="192.168.0.66",
                        help="Access2 TCP host used when Simulate is unchecked.")
    parser.add_argument("--access2-port", type=int, default=7612,
                        help="Access2 TCP port (default 7612).")
    parser.add_argument("--vspin-port", default="",
                        help="VSpin serial port (e.g. /dev/tty.usbserial-A1 or COM6).")
    parser.add_argument("--vspin-baud", type=int, default=57600,
                        help="VSpin serial baud (default 57600).")
    parser.add_argument("--urdf-root", default=str(_URDF_DIR_DEFAULT),
                        help="Folder containing urdf/vspin_and_loader_urdf.urdf.")
    parser.add_argument("--log-level", default="INFO",
                        help="Root log level (DEBUG/INFO/WARNING/ERROR).")
    args = parser.parse_args(argv)
    if args.sim and args.real:
        parser.error("--sim and --real are mutually exclusive")

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-5s %(name)s: %(message)s",
    )

    try:
        import uvicorn
    except ImportError as exc:
        raise SystemExit(
            "uvicorn is required. Install it with `pip install .` from the "
            "vspin-cockpit repo root (it's a hard dependency)."
        ) from exc

    rt = _build_runtime(args)
    app = make_app(rt)

    @app.on_event("startup")
    async def _on_start() -> None:
        await _initial_connect(rt)
        log.info(
            "vspin_cockpit ready · access2=%s vspin=%s · http://%s:%s/",
            "sim" if rt.access2.simulated else "real",
            "sim" if rt.vspin.simulated else "real",
            args.host, args.port,
        )

    @app.on_event("shutdown")
    async def _on_stop() -> None:
        await _disconnect_slot(rt.access2)
        await _disconnect_slot(rt.vspin)

    url = f"http://{args.host}:{args.port}/"
    if not args.no_browser:
        _open_browser_soon(url)

    uvicorn.run(app, host=args.host, port=args.port, log_level=args.log_level.lower())


if __name__ == "__main__":
    main()
