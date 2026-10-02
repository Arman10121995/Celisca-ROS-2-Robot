"""PX4 SITL MAVLink position control. Frames: MAVLink NED, ROS ENU.

See https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK.
This module is shared by the flight probe and the ROS 2 bridge.
"""
import time
from pymavlink import mavutil

# MAVLink POSITION_TARGET_TYPEMASK: keep XYZ and yaw, ignore velocity,
# acceleration and yaw rate. The old 0xFC7 ignored XYZ and requested zero
# velocity, which holds an armed vehicle on the ground rather than taking off.
IGNORE_VEL_ACC = 0b100111111000  # 0x9F8
# PX4 main mode 6 is OFFBOARD; HEARTBEAT encodes it in bits 16..23.
OFFBOARD_CUSTOM_MODE = 6


class Flight:
    def __init__(self, master, rate_hz=20.0):
        self.master = master
        self.period = 1.0 / rate_hz
        self.state = self.position = self.vfr = self.ext_sys = self.local_position = self.attitude = None
        self.last_position_time = 0.0

    def poll(self, limit=200):
        """Drain available telemetry without blocking the ROS setpoint timer."""
        for _ in range(limit):
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            self._absorb(msg)

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
        elif kind == "LOCAL_POSITION_NED":
            self.local_position = msg
            self.last_position_time = time.monotonic()
        elif kind == "ATTITUDE":
            self.attitude = msg
        elif kind == "VFR_HUD":
            self.vfr = msg
        elif kind == "EXTENDED_SYS_STATE":
            self.ext_sys = msg

    def heartbeat(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
            mavutil.mavlink.MAV_STATE_ACTIVE)

    def position_ok(self):
        if self.position is None or self.ext_sys is None or self.local_position is None:
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
        if self.local_position is not None:
            out["local_ned_m"] = [round(getattr(self.local_position, k), 3) for k in ("x", "y", "z")]
        if self.vfr is not None:
            out["climb_mps"] = round(self.vfr.climb, 3)
            out["groundspeed_mps"] = round(self.vfr.groundspeed, 3)
        if self.ext_sys is not None:
            out["landed_state"] = int(self.ext_sys.landed_state)
        if self.state is not None:
            out["fcu_base_mode"] = int(self.state.base_mode)
            out["fcu_custom_mode"] = int(self.state.custom_mode)
        return out

    def set_mode(self, mode, hold=None):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode, 0, 0, 0,
            0, 0, 0)
        if hold is None:
            self.pump(0.6)
        else:
            self.stream(*hold, seconds=0.8)

    def set_param(self, name, value, ptype=6):
        """PARAM_SET: MAV_PARAM_TYPE_INT32=6, REAL32=9."""
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

    def send_setpoint(self, x, y, z, yaw):
        self.master.mav.set_position_target_local_ned_send(
            (int(time.time() * 1000) % (1 << 32)), self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, IGNORE_VEL_ACC,
            float(x), float(y), float(z), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            float(yaw), 0.0)
        self.heartbeat()

    def arm(self, arm=True, hold=(0.0, 0.0, -0.3, 0.0), wait=8.0):
        """Arm while *keeping the setpoint stream alive*.

        PX4 leaves OFFBOARD about half a second after the last setpoint, and
        the arm check then runs in a manual mode and is refused: streaming
        the hold through the arm request is what makes it succeed (measured
        2026-10-01: ARMED base_mode 145, COMMAND_ACK result 0).
        """
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1 if arm else 0, 0, 0, 0, 0, 0, 0)
        end = time.monotonic() + wait
        while time.monotonic() < end:
            self.send_setpoint(*hold)
            msg = self.master.recv_match(blocking=True, timeout=0.02)
            if msg is None:
                continue
            self._absorb(msg)
            if msg.get_type() == "HEARTBEAT" and self.armed() == arm:
                return True
        return self.armed() == arm

    def stream(self, x, y, z, yaw, seconds, sample_every=1.0):
        """Stream position setpoints for `seconds`; sample the FCU state."""
        end = time.monotonic() + seconds
        next_send = last_sample = 0.0
        samples = []
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_send:
                self.send_setpoint(x, y, z, yaw)
                next_send = now + self.period
            msg = self.master.recv_match(blocking=True, timeout=0.02)
            if msg is not None:
                self._absorb(msg)
            if now - last_sample >= sample_every and self.position is not None:
                last_sample = now
                samples.append(self.reading())
        return samples
