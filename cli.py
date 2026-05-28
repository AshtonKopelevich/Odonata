#!/usr/bin/env python3
"""
Odonata CLI — interactive REPL for SJRC F11-GPS drone control.

Usage:
    python cli.py [--host 172.16.10.1] [--timeout 10] [--log-level INFO]

Commands:
    status          Print latest telemetry snapshot
    takeoff         Command drone to take off
    land            Command drone to land
    rth             Return to home
    cancel_rth      Cancel return to home
    stop            Emergency stop
    photo [FILE]    Capture photo; save to FILE (default: photo_<timestamp>.jpg)
    rec start       Start video recording
    rec stop        Stop video recording
    help            List commands
    quit / exit     Disconnect and exit
"""

import argparse
import logging
import os
import sys
import time

# Allow running from repo root without installing
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from process.drone import Drone
from process.protocol.telemetry import decode_voltage

# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------

_GPS_BARS = ["▂", "▄", "▆", "█"]


def _fmt_voltage(raw: int | None) -> str:
    if raw is None:
        return "—"
    v = decode_voltage(raw)
    return f"{v:.1f}V"


def _fmt_gps(signal: int | None) -> str:
    if signal is None:
        return "—"
    bars = "".join(_GPS_BARS[: signal + 1]).ljust(4)
    return f"{bars} ({signal}/3)"


def _fmt_float(val: float | None, unit: str, decimals: int = 1) -> str:
    if val is None:
        return "—"
    return f"{val:.{decimals}f}{unit}"


def print_status(state: dict) -> None:
    model = state["model"] or "unknown"
    print(f"\n  Model     : {model}")
    print(f"  Battery   : drone {_fmt_voltage(state['battery_drone_raw'])}"
          f"  RC {_fmt_voltage(state['battery_rc_raw'])}")
    print(f"  GPS       : {_fmt_gps(state['gps_signal'])}")
    print(f"  Position  : lat {_fmt_float(state['latitude'], '°', 6)}"
          f"  lon {_fmt_float(state['longitude'], '°', 6)}")
    print(f"  Heading   : {_fmt_float(state['heading'], '°', 0)}")
    print(f"  Altitude  : {_fmt_float(state['altitude_m'], 'm')}")
    print(f"  Distance  : {_fmt_float(state['distance_m'], 'm')}")
    print(f"  Speed H   : {_fmt_float(state['speed_horiz_ms'], 'm/s')}")
    print(f"  Speed V   : {_fmt_float(state['speed_vert_ms'], 'm/s')}")
    print()


# ---------------------------------------------------------------------------
# Command dispatch
# ---------------------------------------------------------------------------

def _cmd_status(drone: Drone, _args: list[str]) -> None:
    print_status(drone.get_state())


def _cmd_takeoff(drone: Drone, _args: list[str]) -> None:
    drone.takeoff()
    print("  Takeoff command sent.")


def _cmd_land(drone: Drone, _args: list[str]) -> None:
    drone.land()
    print("  Land command sent.")


def _cmd_rth(drone: Drone, _args: list[str]) -> None:
    drone.rth()
    print("  Returning to home. Type 'cancel_rth' to abort.")


def _cmd_cancel_rth(drone: Drone, _args: list[str]) -> None:
    drone.cancel_rth()
    print("  RTH cancelled.")


def _cmd_stop(drone: Drone, _args: list[str]) -> None:
    confirm = input("  !! Emergency stop — are you sure? [y/N] ").strip().lower()
    if confirm == "y":
        drone.emergency_stop()
        print("  Emergency stop sent.")
    else:
        print("  Cancelled.")


def _cmd_photo(drone: Drone, args: list[str]) -> None:
    if args:
        path = args[0]
    else:
        path = f"photo_{int(time.time())}.jpg"
    print(f"  Capturing photo → {path} ...")
    try:
        jpeg = drone.capture_photo()
        with open(path, "wb") as f:
            f.write(jpeg)
        print(f"  Saved {len(jpeg):,} bytes to {path}")
    except (ConnectionError, ValueError) as exc:
        print(f"  Error: {exc}")


def _cmd_rec(drone: Drone, args: list[str]) -> None:
    sub = args[0].lower() if args else ""
    if sub == "start":
        drone.start_recording()
        print("  Recording started.")
    elif sub == "stop":
        drone.stop_recording()
        print("  Recording stopped.")
    else:
        print("  Usage: rec start | rec stop")


def _cmd_help(_drone: Drone, _args: list[str]) -> None:
    print()
    print("  Commands:")
    for name, (_, desc) in _COMMANDS.items():
        print(f"    {name:<12} {desc}")
    print()


# name -> (handler, description)
_COMMANDS: dict[str, tuple] = {
    "status":     (_cmd_status,     "Print latest telemetry"),
    "takeoff":    (_cmd_takeoff,    "Take off"),
    "land":       (_cmd_land,       "Land"),
    "rth":        (_cmd_rth,        "Return to home"),
    "cancel_rth": (_cmd_cancel_rth, "Cancel return to home"),
    "stop":       (_cmd_stop,       "Emergency stop (confirms first)"),
    "photo":      (_cmd_photo,      "Capture photo [filename]"),
    "rec":        (_cmd_rec,        "rec start | rec stop"),
    "help":       (_cmd_help,       "List commands"),
}

_EXIT_COMMANDS = {"quit", "exit", "q"}


# ---------------------------------------------------------------------------
# REPL
# ---------------------------------------------------------------------------

def repl(drone: Drone) -> None:
    print("\n  Type 'help' for commands, 'quit' to exit.\n")
    while True:
        try:
            line = input("odonata> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        parts = line.split()
        verb, args = parts[0].lower(), parts[1:]

        if verb in _EXIT_COMMANDS:
            break

        handler_entry = _COMMANDS.get(verb)
        if handler_entry is None:
            print(f"  Unknown command '{verb}'. Type 'help' for a list.")
            continue

        handler, _ = handler_entry
        try:
            handler(drone, args)
        except ConnectionError as exc:
            print(f"  Connection error: {exc}")
        except Exception as exc:
            print(f"  Error: {exc}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Odonata — SJRC F11-GPS drone CLI"
    )
    parser.add_argument(
        "--host", default="172.16.10.1",
        help="Drone IP address (default: 172.16.10.1)"
    )
    parser.add_argument(
        "--timeout", type=float, default=10.0,
        help="Connection timeout in seconds (default: 10)"
    )
    parser.add_argument(
        "--log-level", default="WARNING",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Log level (default: WARNING)"
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    print(f"Connecting to drone at {args.host} ...")
    drone = Drone(host=args.host)

    try:
        drone.connect(timeout=args.timeout)
    except TimeoutError:
        print("Error: drone did not respond. Check WiFi connection.")
        sys.exit(1)
    except ConnectionError as exc:
        print(f"Error: {exc}")
        sys.exit(1)

    print(f"Connected. Model: {drone.state.model or 'unknown'}")

    try:
        repl(drone)
    finally:
        print("Disconnecting ...")
        drone.disconnect()
        print("Done.")


if __name__ == "__main__":
    main()