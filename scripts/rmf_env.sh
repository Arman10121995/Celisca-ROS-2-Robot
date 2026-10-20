#!/usr/bin/env bash
# Optional official Humble Open-RMF overlay, provisioned on the workspace SSD.
_lab_rmf_prefix="${ROBOT_LAB_RUNTIME_ROOT:-/workspace/molar/robot_lab_runtime}/rmf/current/prefix/opt/ros/humble"
if [[ -d "$_lab_rmf_prefix/share/rmf_fleet_adapter_python" ]]; then
    export AMENT_PREFIX_PATH="$_lab_rmf_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export CMAKE_PREFIX_PATH="$_lab_rmf_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$_lab_rmf_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    for _lab_rmf_lib in "$_lab_rmf_prefix/lib/"*-linux-gnu; do
        [[ -d "$_lab_rmf_lib" ]] && export LD_LIBRARY_PATH="$_lab_rmf_lib:$LD_LIBRARY_PATH"
    done
    export PYTHONPATH="$_lab_rmf_prefix/local/lib/python3.10/dist-packages:$_lab_rmf_prefix/lib/python/site-packages:$_lab_rmf_prefix/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
unset _lab_rmf_prefix _lab_rmf_lib
