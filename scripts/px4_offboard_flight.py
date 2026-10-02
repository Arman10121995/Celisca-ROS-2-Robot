#!/usr/bin/env python3
"""R5.4: fly the live PX4 SITL FCU in offboard mode; measure, don't assume.

Part of the R5.4 multirotor integration: a dependency-light MAVLink offboard
client (pymavlink only) for the PX4 SITL FCU, used because ros-humble-mavros
needs apt/root and this host has neither. Requires the FCU to be running with
the vehicle inserted (see scripts/px4_sitl_model.py).

Waits for a position estimate, enters OFFBOARD, arms, takes off to 3 m, flies
three waypoints, tests the command-loss failsafe by stopping the setpoint
stream, then lands and disarms.  Every sample is the FCU's own
GLOBAL_POSITION_INT / VFR_HUD state.  Prints one JSON object.

    python3 px4_offboard_flight.py --port udpout:127.0.0.1:14580 --out f.json
"""
import argparse
import json
import time

from pymavlink import mavutil

# Scripts also run directly from a source checkout, before a colcon install.
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/robot_lab_adapter"))
from robot_lab_adapter.px4_mavlink import Flight, OFFBOARD_CUSTOM_MODE


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="udpout:127.0.0.1:14580")
    parser.add_argument("--out", default="")
    parser.add_argument("--takeoff-alt", type=float, default=3.0)
    args = parser.parse_args()

    master = mavutil.mavlink_connection(args.port, source_system=255,
                                         source_component=190)
    flight = Flight(master)
    report = {"port": args.port, "phases": {}, "takeoff_alt_m": args.takeoff_alt}

    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        flight.pump(1.0)
        flight.heartbeat()
        if flight.state is not None and flight.position_ok():
            break
    report["fcu_present"] = flight.state is not None
    report["position_estimate_ok"] = flight.position_ok()
    if flight.state is None or not flight.position_ok():
        report["error"] = "no heartbeat or valid position estimate from the FCU"
        report["passed"] = False
        text = json.dumps(report, indent=1)
        print(text)
        if args.out:
            Path(args.out).write_text(text)
        return 1
    report["start"] = flight.reading()

    # Configure a measured Offboard-loss landing. Keep normal GPS/arming checks.
    report["params_set"] = {}
    for pname, value, ptype in (("COM_OBL_RC_ACT", 4.0, 6), ("COM_OF_LOSS_T", 0.5, 9)):
        flight.set_param(pname, value, ptype)
        report["params_set"][pname] = flight.param_value(pname)

    flight.stream(0.0, 0.0, -0.3, 0.0, 2.0)
    flight.set_mode(OFFBOARD_CUSTOM_MODE, hold=(0.0, 0.0, -0.3, 0.0))
    flight.stream(0.0, 0.0, -0.3, 0.0, 1.0)
    report["custom_mode_after_set"] = int(flight.state.custom_mode) if flight.state else None
    flight.arm(True)
    flight.stream(0.0, 0.0, -0.3, 0.0, 1.0)
    report["armed_after_arm_command"] = flight.armed()

    samples = []
    for step in range(1, 13):
        z = -0.3 - (args.takeoff_alt - 0.3) * step / 12.0
        samples += flight.stream(0.0, 0.0, z, 0.0, 0.5)
    report["phases"]["takeoff"] = samples
    report["phases"]["hover"] = flight.stream(0.0, 0.0, -args.takeoff_alt, 0.0, 6.0)
    report["alt_after_takeoff"] = flight.reading()

    for index, (x, y) in enumerate([(3.0, 0.0), (3.0, 3.0), (0.0, 3.0)], 1):
        leg = flight.stream(x, y, -args.takeoff_alt, 0.0, 6.0)
        if leg:
            leg[-1]["setpoint_ned_m"] = [x, y, -args.takeoff_alt]
        report["phases"]["waypoint_%d" % index] = leg

    before = flight.reading()
    flight.pump(6.0)
    report["phases"]["command_loss"] = {
        "before": before, "after": flight.reading(),
        "still_armed": flight.armed(),
        "note": "setpoint stream stopped for 6 s; offboard-loss action is FCU-side"}

    flight.master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 6, 0, 0, 0, 0)
    land = []
    for _ in range(15):
        flight.heartbeat()
        flight.pump(1.0)
        land.append(flight.reading())
        if flight.ext_sys is not None and flight.ext_sys.landed_state == 1 and not flight.armed():
            break
    if land:
        land[-1]["armed_before_disarm"] = flight.armed()
    report["phases"]["land"] = land
    flight.arm(False)
    flight.pump(2.0)
    report["end"] = flight.reading()
    flight.pump(1.0)
    report["armed_end"] = flight.armed()
    report["mavlink"] = {"sent": master.mav.total_packets_sent,
                         "received": master.mav.total_packets_received}
    import math
    errors = {}
    for index, target in enumerate(((3.0, 0.0), (3.0, 3.0), (0.0, 3.0)), 1):
        leg = report["phases"]["waypoint_%d" % index]
        measured = leg[-1].get("local_ned_m") if leg else None
        errors[str(index)] = (math.dist(measured, [*target, -args.takeoff_alt])
                             if measured is not None else None)
    hover = report["phases"]["hover"]
    settled = [sample["local_ned_m"] for sample in hover[-3:] if "local_ned_m" in sample]
    loss = report["phases"]["command_loss"]
    report["acceptance"] = {
        "takeoff_hover": bool(settled) and all(abs(p[2] + args.takeoff_alt) < 0.35 for p in settled),
        "waypoint_errors_m": errors,
        "waypoints": all(error is not None and error < 0.35 for error in errors.values()),
        "command_loss_left_offboard": (loss["after"].get("fcu_custom_mode", 0) >> 16 & 0xFF) != 6,
        "landed_disarmed": not flight.armed() and flight.ext_sys is not None and flight.ext_sys.landed_state == 1,
    }
    report["passed"] = all(report["acceptance"][key] for key in
                            ("takeoff_hover", "waypoints", "command_loss_left_offboard", "landed_disarmed"))
    text = json.dumps(report, indent=1)
    print(text)
    if args.out:
        with open(args.out, "w") as handle:
            handle.write(text)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
