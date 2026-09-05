#!/usr/bin/env python3
"""
glove_monitor.py -- live dashboard and CLI bridge for either board.

Build the firmware with CSV telemetry enabled (in common/GloveConfig.h):

    GLOVE_TX_TELEMETRY_CSV 1     for the glove
    GLOVE_RX_TELEMETRY_CSV 1     for the arm

then:

    python3 tools/glove_monitor.py --port /dev/ttyUSB0
    python3 tools/glove_monitor.py --port COM4 --log capture.csv
    python3 tools/glove_monitor.py --port /dev/ttyUSB0 --send s

What you get:

  * a bar per servo channel, so a dead flex sensor, a backwards-mounted one or
    an over-narrow calibration span is obvious at a glance,
  * the radio link statistics the receiver measures (quality, lost frames,
    failsafe events), which is how you tell "bad antenna" from "bad code",
  * anything you type is forwarded to the board's serial CLI, so calibration
    ('c', 'w'), presets ('p 2') and status ('s') all work from here,
  * --log writes every telemetry line to a CSV file with a host timestamp, for
    offline analysis or for attaching to a bug report.

Requires pyserial:  pip install pyserial

Typing into the monitor forwards to the board (Linux/macOS only; on Windows
the monitor is read-only, so use --send to issue commands).
"""

from __future__ import annotations

import argparse
import datetime
import os
import select
import sys
import time

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    sys.exit(
        "glove_monitor.py needs pyserial:\n"
        "    python3 -m pip install pyserial\n"
    )

BAR_WIDTH = 34
CHANNEL_NAMES = ["wrist pitch", "wrist roll", "thumb", "index", "middle",
                 "ring", "little"]
RX_MODES = {0: "BOOT", 1: "LIVE", 2: "PRESET", 3: "PARKED", 4: "RELAXED",
            5: "FAILSAFE", 6: "WAITING"}


def bar(value: float, lo: float = 0.0, hi: float = 180.0) -> str:
    """Render value as a fixed-width bar."""
    span = max(hi - lo, 1e-9)
    frac = (value - lo) / span
    frac = min(max(frac, 0.0), 1.0)
    filled = int(round(frac * BAR_WIDTH))
    return "[" + "#" * filled + "." * (BAR_WIDTH - filled) + "]"


def ints(fields):
    out = []
    for f in fields:
        try:
            out.append(int(f))
        except ValueError:
            out.append(0)
    return out


def render_glove(fields, out) -> None:
    """G,seq,imuOk,calibrating,ch0..ch6,pitch,roll,sent,deferred"""
    if len(fields) < 15:
        return
    v = ints(fields[1:])
    seq, imu_ok, calibrating = v[0], v[1], v[2]
    channels = v[3:10]
    pitch, roll, sent, deferred = v[10], v[11], v[12], v[13]

    out("\x1b[2J\x1b[H")  # clear + home
    out("RoboticArm glove (transmitter)              ")
    out(f"seq {seq & 0xFF:3d}  sent {sent}  deferred {deferred}\n")
    out(f"IMU: {'ok' if imu_ok else 'MISSING'}   "
        f"{'CALIBRATING' if calibrating else 'running'}\n\n")
    for name, value in zip(CHANNEL_NAMES, channels):
        out(f"  {name:<12}{bar(value)} {value:3d}\n")
    out(f"\n  wrist attitude: pitch {pitch:4d} deg   roll {roll:4d} deg\n")
    out("\ntype a command + Enter (h for help on the board)\n")


def render_arm(fields, out) -> None:
    """R,ageMs,quality,good,lost,failsafes,mode,servo0..6,rejected"""
    if len(fields) < 15:
        return
    v = ints(fields[1:])
    age, quality, good, lost, failsafes, mode = v[0], v[1], v[2], v[3], v[4], v[5]
    servos = v[6:13]
    rejected = v[13]

    out("\x1b[2J\x1b[H")
    out("RoboticArm receiver                          ")
    out(f"mode {RX_MODES.get(mode, mode)}\n")
    health = "OK" if age < 500 else "LINK LOST"
    out(f"link {health}  last frame {age} ms ago  quality {quality}%  "
        f"(ok {good}, lost {lost}, failsafes {failsafes}, rejected {rejected})\n\n")
    for name, value in zip(CHANNEL_NAMES, servos):
        out(f"  {name:<12}{bar(value)} {value:3d}\n")
    out("\ntype a command + Enter (h for help on the board)\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--port", required=True, help="serial port of the board")
    ap.add_argument("--baud", type=int, default=115200,
                    help="baud rate (must match GLOVE_SERIAL_BAUD)")
    ap.add_argument("--log", metavar="FILE",
                    help="append every telemetry line to this CSV file")
    ap.add_argument("--raw", action="store_true",
                    help="print every line as-is, no dashboard")
    ap.add_argument("--send", metavar="TEXT", action="append", default=[],
                    help="send this line to the board at startup (repeatable)")
    ap.add_argument("--seconds", type=float, default=0,
                    help="stop after this many seconds (0 = run forever)")
    args = ap.parse_args()

    try:
        link = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as exc:
        sys.exit(f"cannot open {args.port}: {exc}")

    log_file = open(args.log, "a", buffering=1) if args.log else None
    if log_file and os.path.getsize(args.log or "") == 0:
        log_file.write("#host_time,line\n")

    for line in args.send:
        link.write((line + "\n").encode())

    # Typing into the monitor forwards to the board's serial CLI.  That needs
    # select() on a serial object, which only POSIX provides; elsewhere the
    # monitor is read-only and --send is the way to issue commands.
    interactive = (os.name == "posix")

    started = time.time()
    sys.stdout.write("\x1b[?25l")  # hide cursor
    try:
        while True:
            if args.seconds and (time.time() - started) > args.seconds:
                break

            if interactive:
                ready, _, _ = select.select([sys.stdin, link], [], [], 0.05)
                if sys.stdin in ready:
                    typed = sys.stdin.readline()
                    if not typed:
                        break
                    link.write(typed.encode())
                if link not in ready:
                    continue
            raw = link.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", "replace").strip()
            if not text:
                continue

            if log_file:
                stamp = datetime.datetime.now().isoformat(timespec="milliseconds")
                log_file.write(f"{stamp},{text}\n")

            fields = text.split(",")
            if not args.raw and fields[0] == "G":
                render_glove(fields, sys.stdout.write)
            elif not args.raw and fields[0] == "R":
                render_arm(fields, sys.stdout.write)
            else:
                if not args.raw:
                    sys.stdout.write("\x1b[0J")
                sys.stdout.write(text + "\n")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\x1b[?25h\n")  # show cursor
        link.close()
        if log_file:
            log_file.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
