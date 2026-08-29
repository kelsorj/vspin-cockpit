"""vspin_cockpit — standalone control app for the Agilent VSpin
centrifuge + Access2 loader pair, with a live URDF twin and a shared
diagnostic log.

Run with::

    python -m vspin_cockpit                              # sim mode
    python -m vspin_cockpit --real \\
        --access2-host 192.168.0.66 --vspin-port /dev/tty.usbserial-A1
"""

__version__ = "0.1.0"
