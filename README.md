# vspin-cockpit

A tiny standalone control app for the **Agilent VSpin centrifuge + Access2
plate-loader** pair, packaged as one Python module. Boots a local FastAPI
server, opens your browser, and gives you one page with:

* A **live 3D URDF twin** of the combined VSpin + loader (drives the Y, Z,
  and gripper axes plus the rotor in real time as the drivers report motion).
* An **Access2 control panel** — Home / Park, jog pad, teachpoints,
  load / unload, and a full-cycle **Load → Spin → Unload** button.
* A **VSpin control panel** — RCF or RPM entry with live conversion, spin
  start/stop, door + bucket controls, teach buckets.
* A **shared, colour-coded activity log** that traces every command from
  both panels top-to-bottom, so a full cycle reads as one story.

Runs on Mac / Linux / Windows. Ships with a **simulator** by default —
no hardware required to try it. Flip the `Simulate` checkbox off (or start
with `--real`) to talk to a real Access2 (TCP :7612) and VSpin (Velocity11
NMC over serial).

![Screenshot placeholder — the app looks like a dark two-column cockpit
with a URDF viewer on the left and stacked device panels on the right.]

## Install

```bash
cd vspin-cockpit
pip install .                    # sim only — no hardware libs pulled in
pip install '.[hardware]'        # + pyserial-asyncio, for the real VSpin
```

## Run

```bash
python -m vspin_cockpit           # sim mode, opens http://127.0.0.1:8765/
vspin-cockpit                     # same, if the console script installed
vspin-cockpit --no-browser        # don't auto-open the browser
```

Real hardware:

```bash
vspin-cockpit --real \
  --access2-host 192.168.0.66 --access2-port 7612 \
  --vspin-port /dev/tty.usbserial-A1 --vspin-baud 57600
```

Windows serial ports look like `COM6`. If you don't know the Access2's
IP, click **Find available device** in the Access2 header pill — it
UDP-broadcasts the Velocity11 discovery protocol and falls back to a
subnet TCP sweep.

Common flags:

| flag | meaning |
|---|---|
| `--sim` / `--real` | Boot in sim or connected to real hardware. `--sim` is the default; both are mutually exclusive. |
| `--access2-host`, `--access2-port` | Access2 TCP endpoint (default `192.168.0.66:7612`). |
| `--vspin-port`, `--vspin-baud` | VSpin serial port + baud (default `57600` 8N1). |
| `--host`, `--port` | Bind address / port for the local server. Loopback by default. |
| `--no-browser` | Don't auto-open the browser (headless / CI). |
| `--urdf-root <path>` | Override the folder containing `urdf/vspin_and_loader_urdf.urdf`. |
| `--log-level DEBUG` | Verbose driver logs. |

The `Simulate` checkbox in each header pill lets you swap sim ↔ real per
device at runtime without restarting the app — flip it, click Connect,
and the app rebuilds just that slot's driver.

## Repo layout

```
vspin-cockpit/
├── pyproject.toml
├── README.md
├── URDF/                                    # ~4 MB of gltf meshes + URDFs
│   ├── vspin_access_2_urdf/
│   │   ├── urdf/vspin_and_loader_urdf.urdf  # the combined twin the viewer loads
│   │   └── meshes/*.gltf
│   └── vspin/urdf/vspin.urdf                # bare-centrifuge URDF (reference)
└── src/vspin_cockpit/
    ├── __init__.py
    ├── __main__.py                          # `python -m vspin_cockpit`
    ├── app.py                               # FastAPI + runtime + CLI
    ├── core/
    │   ├── driver.py                        # Driver ABC, Command, CommandResult, ...
    │   ├── events.py                        # EventBus, DeviceEvent
    │   ├── time_source.py                   # Clock, WallClock
    │   └── transports.py                    # TransportError hierarchy
    ├── drivers/
    │   ├── vspin.py                         # SimVSpin, VSpinDriver, SimAccess2, Access2Driver
    │   ├── transports_nmc.py                # VSpin Velocity11 NMC framer (RS-232)
    │   ├── transports_access2_tcp.py        # Access2 binary packet framer (TCP :7612)
    │   └── discovery_v11.py                 # UDP-broadcast + TCP-probe device discovery
    └── static/
        ├── index.html                       # single-page cockpit layout
        ├── app.css                          # dark theme
        └── app.js                           # three.js/URDFLoader + SSE + command dispatch
```

## How it fits together

* Every button in the UI POSTs `{kind, params}` to `/devices/{slot}/execute`,
  which dispatches to `driver.execute(Command(...))`. Whatever `kind`s the
  driver declares in its `supported_commands` map is dispatchable — the
  buttons wire only the ones you'd actually use at a bench.
* Joint animation comes off `/events` (Server-Sent Events, filtered to
  `kinematic_state`). The two sim drivers emit ~10 Hz frames during motion.
  A 500 ms `read_status` poll fills the KPI strip + status LEDs and acts as
  a fallback so the URDF stays in sync if SSE drops.
* The loader's `complete_cycle` command needs to dispatch a `spin` to the
  paired centrifuge — it does this through a peer resolver that the app
  re-installs on every Access2 (re)connect, so swapping the VSpin slot
  sim↔real updates the loader's peer automatically.

## No hardware attached — what still works

Everything in the UI. Both `SimAccess2` and `SimVSpin` mirror the real
drivers' command surface exactly; they just replace the wire-level I/O
with kinematic bookkeeping + emitted joint frames. You can load / unload
plates (watch the arm move), run full cycles, spin the rotor, jog axes,
teach buckets — all against nothing.

## Real hardware — what to expect

The `VSpinDriver` (RS-232 Velocity11 NMC) and `Access2Driver` (TCP :7612
binary framing) implementations were extracted from a larger orchestrator
where they were being brought up on the bench. Basic flows work; treat
edge cases (fault recovery, non-standard rotor calibrations) as needing
verification against your unit before running unattended. The activity log
surfaces every command + response, which is the fastest way to spot a
protocol drift.
