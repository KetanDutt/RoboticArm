#!/usr/bin/env python3
"""
Radio air-time and latency budget calculator for the glove link.

VirtualWire / RadioHead's RH_ASK put the following on the air for a payload of
N bytes (see common/RfLink.h and docs/PROTOCOL.md):

    36 bits  training preamble (alternating 1/0)
    12 bits  start symbol
    12 bits  length byte      } every byte is encoded as two 6-bit symbols,
    12 x N   payload          } i.e. 12 bits on the air per byte
    24 bits  2-byte FCS

    total = 48 + 12 * (N + 3) bits

That number, divided by the data rate, is a hard floor on the control latency:
no firmware cleverness can beat it, only a smaller payload or a faster radio
can.  This script prints the budget so the trade-off is explicit instead of
being discovered with a stopwatch.

    python3 tools/rf_airtime.py
    python3 tools/rf_airtime.py --payload 9 --speed 2000 4000
    python3 tools/rf_airtime.py --budget
"""

from __future__ import annotations

import argparse

FRAME_LEN = 9  # GLOVE_FRAME_LEN


def air_bits(payload_len: int) -> int:
    return 48 + 12 * (payload_len + 3)


def air_ms(payload_len: int, speed_bps: int) -> float:
    return air_bits(payload_len) * 1000.0 / speed_bps


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--payload", type=int, default=FRAME_LEN,
                    help="payload bytes per frame (default: %(default)s)")
    ap.add_argument("--speed", type=int, nargs="+", default=[2000, 4000],
                    help="data rate(s) in bits per second to compare")
    ap.add_argument("--budget", action="store_true",
                    help="also print the end-to-end latency budget")
    args = ap.parse_args()

    print(f"payload: {args.payload} byte(s), "
          f"protocol frame = {FRAME_LEN} bytes")
    print(f"{'speed':>8} {'bits on air':>12} {'air time':>10} {'frames/s':>9}")
    for speed in args.speed:
        bits = air_bits(args.payload)
        ms = air_ms(args.payload, speed)
        print(f"{speed:>6} b {bits:>12} {ms:>8.1f} ms {1000.0/ms:>9.1f}")

    if args.budget:
        speed = args.speed[0]
        tx = air_ms(args.payload, speed)
        print("\nend-to-end budget (one control update, worst case)")
        print(f"  glove samples sensors          ~"
              f"{5:>6} ms   (GLOVE_TX_SAMPLE_MS)")
        print(f"  radio air time                 ~{tx:>6.1f} ms   "
              f"(at {speed} bps)")
        print(f"  arm drains + decodes frame     ~{1:>6} ms")
        print(f"  arm servo refresh              ~{20:>6} ms   "
              f"(GLOVE_RX_UPDATE_MS)")
        print(f"  servo mechanical response      ~{100:>6} ms   "
              f"(typical hobby servo, no load)")
        total = 5 + tx + 1 + 20
        print(f"  --> command latency            ~{total:>6.1f} ms "
              f"({1000.0/total:.1f} Hz control rate) before the servo moves")
        print("\nThe radio dominates: halving the payload or doubling the")
        print("data rate is worth more than any amount of code tuning.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
