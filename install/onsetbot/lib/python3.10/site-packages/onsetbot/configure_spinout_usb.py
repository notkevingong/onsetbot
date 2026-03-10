#!/usr/bin/env python3
"""Configure ODrive spinout thresholds over USB only.

This script updates only:
- axisX.controller.config.spinout_mechanical_power_threshold
- axisX.controller.config.spinout_electrical_power_threshold

All other settings are left unchanged.
"""

# how to use:
# bash$ python3 configure_spinout_usb.py --axes 0 1 --mech -500 --elec 500 --save
# this

import argparse
import sys
import time


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Set ODrive spinout thresholds over USB")
    parser.add_argument(
        "--serial",
        type=str,
        default="",
        help="ODrive serial number to target (leave empty for first discovered device)",
    )
    parser.add_argument(
        "--axes",
        type=int,
        nargs="+",
        default=[0, 1],
        help="Axis indices to configure (example: --axes 0 1)",
    )
    parser.add_argument(
        "--mech",
        type=float,
        default=-500.0,
        help="spinout_mechanical_power_threshold",
    )
    parser.add_argument(
        "--elec",
        type=float,
        default=500.0,
        help="spinout_electrical_power_threshold",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="USB discovery timeout in seconds",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Persist values using save_configuration()",
    )
    parser.add_argument(
        "--reboot-wait",
        type=float,
        default=3.0,
        help="Wait time after save_configuration()",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    try:
        import odrive
    except Exception as exc:
        print(f"ERROR: Could not import odrive package: {exc}")
        return 1

    try:
        if args.serial.strip():
            odrv = odrive.find_any(serial_number=args.serial.strip(), timeout=args.timeout)
        else:
            odrv = odrive.find_any(timeout=args.timeout)
    except Exception as exc:
        print(f"ERROR: USB discovery failed: {exc}")
        return 1

    if odrv is None:
        print("ERROR: No ODrive found over USB")
        return 1

    serial_str = str(getattr(odrv, "serial_number", "unknown"))
    configured = 0

    for axis_index in args.axes:
        axis_obj = getattr(odrv, f"axis{axis_index}", None)
        if axis_obj is None:
            print(f"WARN: axis{axis_index} not found on ODrive {serial_str}")
            continue

        axis_obj.controller.config.spinout_mechanical_power_threshold = float(args.mech)
        axis_obj.controller.config.spinout_electrical_power_threshold = float(args.elec)
        configured += 1
        print(
            f"Set axis{axis_index}: mech={float(args.mech):.3f}, "
            f"elec={float(args.elec):.3f}"
        )

    if configured == 0:
        print("ERROR: No axes were configured")
        return 2

    if args.save:
        try:
            odrv.save_configuration()
            print("Saved configuration to flash")
            if args.reboot_wait > 0.0:
                time.sleep(args.reboot_wait)
        except Exception as exc:
            print(f"ERROR: save_configuration() failed: {exc}")
            return 3
    else:
        print("Applied thresholds in RAM only (not saved)")

    print(f"Done on ODrive {serial_str}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
