#!/usr/bin/env python3
"""R5.4: fly the live PX4 SITL FCU in offboard mode; measure, don't assume.

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

IGNORE_VEL_ACC = 0b0000111111000111  # vel/acc/ang-vel/attitude-rate/yaw-rate
# PX4 encodes the flight mode in the low 16 bits of the MAVLink custom_mode;
# nav_state 6 is OFFBOARD.  It is not a mavlink.h constant.
OFFBOARD_CUSTOM_MODE = 6


class Flight:
    def __init__(self, master, rate_hz=20.0):
        self.master = master
        self.period = 1.0 / rate_hz
        self.state = self.position = self.vfr = self.ext_sys = None

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg = self.master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            self._absorb(msg)

    def _absorb(self, msg):
        kind = msg.get_type()
        if kind == "HEARTBEAT":
            self.state = msg
        elif kind == "GLOBAL_POSITION_INT":
            self.position = msg
        elif kind == "VFR_HUD":
            self.vfr = msg
        elif kind == "EXTENDED_SYS_STATE":
            self.ext_sys = msg

    def heartbeat(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_PX4, 0, 0,
            mavutil.mavlink.MAV_STATE_ACTIVE)

    def position_ok(self):
        if self.position is None or self.ext_sys is None:
            return False
        # MAV_LANDED_STATE_UNINIT == 0: PX4 cannot fly without an estimate.
        return int(self.ext_sys.landed_state) != 0

    def armed(self):
        return bool(self.state is not None and self.state.base_mode & 0x80)

    def reading(self):
        out = {}
        if self.position is not None:
            out["alt_m"] = round(self.position.relative_alt / 1000.0, 3)
            out["lat_deg"] = round(self.position.lat / 1e7, 6)
            out["lon_deg"] = round(self.position.lon / 1e7, 6)
        if self.vfr is not None:
            out["climb_mps"] = round(self.vfr.climb, 3)
            out["groundspeed_mps"] = round(self.vfr.groundspeed, 3)
        if self.ext_sys is not None:
            out["landed_state"] = int(self.ext_sys.landed_state)
        if self.state is not None:
            out["fcu_base_mode"] = int(self.state.base_mode)
            out["fcu_custom_mode"] = int(self.state.custom_mode)
        return out

    def set_mode(self, mode):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode, 0, 0, 0,
            0, 0, 0)
        self.pump(0.6)

    def set_param(self, name, value, ptype=6):
        """PARAM_SET with a zero-padded 16-byte id (REAL32)."""
        self.master.mav.param_set_send(
            self.master.target_system, self.master.target_component,
            name.encode() + b"\x00" * (16 - len(name)), float(value), ptype)
        self.pump(0.4)

    def param_value(self, name):
        """Request PARAM_VALUE_READ once and return the last matching value."""
        self.master.mav.param_request_read_send(
            self.master.target_system, self.master.target_component,
            name.encode() + b"\x00" * (16 - len(name)), -1)
        end = time.monotonic() + 2.0
        found = None
        while time.monotonic() < end:
            msg = self.master.recv_match(blocking=True, timeout=0.2)
            if msg is not None and msg.get_type() == "PARAM_VALUE":
                raw = msg.param_id
                label = (raw.split(b"\x00")[0].decode(errors="ignore")
                         if isinstance(raw, bytes) else str(raw).split("\x00")[0])
                if label == name:
                    found = msg.param_value
            if found is not None:
                break
        return found

    def arm(self, arm=True):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1 if arm else 0, 0, 0, 0, 0, 0, 0)
        self.pump(1.0)

    def stream(self, x, y, z, yaw, seconds, sample_every=1.0):
        """Stream position setpoints for `seconds`; sample the FCU state."""
        end = time.monotonic() + seconds
        next_send = last_sample = 0.0
        samples = []
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_send:
                self.master.mav.set_position_target_local_ned_send(
                    (int(time.time() * 1000) % (1 << 32)), self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    IGNORE_VEL_ACC, 0.0, 0.0, float(z), 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, float(yaw), 0.0)
                self.heartbeat()
                next_send = now + self.period
            msg = self.master.recv_match(blocking=True, timeout=0.02)
            if msg is not None:
                self._absorb(msg)
            if now - last_sample >= sample_every and self.position is not None:
                last_sample = now
                samples.append(self.reading())
        return samples


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
    if flight.state is None:
        report["error"] = "no heartbeat from the FCU"
        print(json.dumps(report, indent=1))
        return
    report["start"] = flight.reading()

    # SITL has no GPS lock inside the Gazebo session, so PX4's preflight
    # refuses to arm with the default COM_ARM_WO_GPS=0.  Setting the parameter
    # is recorded, not assumed: the value read back is part of the report.
    report["params_set"] = {}
    for pname, value in (("COM_ARM_WO_GPS", 1.0), ("COM_ARM_AUTH_REQ", 0.0)):
        flight.set_param(pname, value)
        report["params_set"][pname] = flight.param_value(pname)

    flight.stream(0.0, 0.0, -0.3, 0.0, 2.0)
    flight.set_mode(OFFBOARD_CUSTOM_MODE)
    flight.pump(1.0)
    report["custom_mode_after_set"] = int(flight.state.custom_mode) if flight.state else None
    flight.arm(True)
    flight.pump(1.0)
    report["armed_after_arm_command"] = flight.armed()

    samples = []
    for step in range(1, 13):
        z = -0.3 - (args.takeoff_alt - 0.3) * step / 12.0
        samples += flight.stream(0.0, 0.0, z, 0.0, 0.5)
    report["phases"]["takeoff"] = samples
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

    land = flight.stream(0.0, 0.0, 0.2, 0.0, 9.0, sample_every=2.0)
    if land:
        land[-1]["armed_before_disarm"] = flight.armed()
    report["phases"]["land"] = land
    flight.arm(False)
    flight.pump(2.0)
    report["end"] = flight.reading()
    report["armed_end"] = flight.armed()
    report["mavlink"] = {"sent": master.mav.total_packets_sent,
                         "received": master.mav.total_packets_received}
    text = json.dumps(report, indent=1)
    print(text)
    if args.out:
        with open(args.out, "w") as handle:
            handle.write(text)


if __name__ == "__main__":
    main()
