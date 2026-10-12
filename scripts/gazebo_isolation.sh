# Sourced by the measurement scripts: robot_lab_isolate_gazebo NAME gives the
# run its own Gazebo Fortress transport partition and discovery ports.
#
# The partition separates concurrent runs.  The ports keep a Gazebo Harmonic
# instance on the default ports (a PX4 flight started before 2026-10-12, or
# any other gz-transport13 tool) out of the run: Fortress cannot parse
# Harmonic discovery messages, and with one flight running every Fortress
# process logged "Unknown message type" ~6,500 times per second (~1M launch
# log lines per Gazebo mission).  The partition is checked only after parsing,
# so it does not help.  Fortress reads IGN_*; GZ_PARTITION is kept for tools
# that read the newer name.
robot_lab_isolate_gazebo() {
  export IGN_PARTITION="$1" GZ_PARTITION="$1"
  export IGN_DISCOVERY_MSG_PORT="${IGN_DISCOVERY_MSG_PORT:-11317}"
  export IGN_DISCOVERY_SRV_PORT="${IGN_DISCOVERY_SRV_PORT:-11318}"
}
