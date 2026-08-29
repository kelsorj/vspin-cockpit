"""Device drivers.

  * :mod:`vspin_cockpit.drivers.vspin` — ``SimVSpin``, ``VSpinDriver``,
    ``SimAccess2``, ``Access2Driver`` (+ their bases).
  * :mod:`vspin_cockpit.drivers.transports_nmc` — Velocity11 PIC-Servo
    NMC binary framer over RS-232 (used by ``VSpinDriver``).
  * :mod:`vspin_cockpit.drivers.transports_access2_tcp` — Access2 binary
    packet framer over TCP :7612 (used by ``Access2Driver``).
  * :mod:`vspin_cockpit.drivers.discovery_v11` — UDP broadcast discovery
    of Ethernet-equipped Velocity11 loaders / bravos / washers.
"""
