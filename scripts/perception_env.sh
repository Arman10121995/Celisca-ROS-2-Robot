#!/usr/bin/env bash
# Optional official Humble converter overlay, provisioned on the workspace SSD.
_lab_perception_prefix="${ROBOT_LAB_RUNTIME_ROOT:-/workspace/molar/robot_lab_runtime}/perception/current/prefix/opt/ros/humble"
if [[ -f "$_lab_perception_prefix/share/ament_index/resource_index/packages/pointcloud_to_laserscan" ]]; then
    for _lab_perception_var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH; do
        _lab_perception_value="${!_lab_perception_var:-}"
        if [[ ":$_lab_perception_value:" != *":$_lab_perception_prefix:"* ]]; then
            export "$_lab_perception_var=$_lab_perception_prefix${_lab_perception_value:+:$_lab_perception_value}"
        fi
    done
    if [[ ":${LD_LIBRARY_PATH:-}:" != *":$_lab_perception_prefix/lib:"* ]]; then
        export LD_LIBRARY_PATH="$_lab_perception_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
fi
unset _lab_perception_prefix _lab_perception_var _lab_perception_value
