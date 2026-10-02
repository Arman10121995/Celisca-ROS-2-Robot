#!/usr/bin/env bash
# Source this helper before large Robot Lab builds/trials on the Jetson.
# Does not modify login configuration or move any files.
_robot_lab_ssd_env() {
    local lab_storage_base
    lab_storage_base=${ROBOT_LAB_RUNTIME_ROOT:-/workspace/molar/robot_lab_runtime}
    if [[ ! -d /workspace ]] ||
       [[ $(stat -c %d /workspace) == $(stat -c %d /) ]]; then
        printf '%s\n' 'Robot Lab: the workspace SSD is not mounted; refusing internal-disk runtime output.' >&2
        return 1
    fi
    if [[ $lab_storage_base != /workspace/* ]]; then
        printf '%s\n' 'Robot Lab: ROBOT_LAB_RUNTIME_ROOT must be under /workspace on this Jetson.' >&2
        return 1
    fi
    mkdir -p "$lab_storage_base" || return 1
    if [[ $(stat -c %d "$lab_storage_base") == $(stat -c %d /) ]]; then
        printf '%s\n' 'Robot Lab: runtime path resolves to internal storage.' >&2
        return 1
    fi
    mkdir -p "$lab_storage_base/tmp" "$lab_storage_base/ros/log" || return 1
    export ROBOT_LAB_RUNTIME_ROOT="$lab_storage_base"
    export TMPDIR="$lab_storage_base/tmp"
    export TMP="$TMPDIR" TEMP="$TMPDIR"
    export ROS_LOG_DIR="$lab_storage_base/ros/log"
    printf 'Robot Lab runtime: %s\n' "$ROBOT_LAB_RUNTIME_ROOT"
}
_robot_lab_ssd_env
_lab_storage_result=$?
unset -f _robot_lab_ssd_env
return "$_lab_storage_result" 2>/dev/null || exit "$_lab_storage_result"
