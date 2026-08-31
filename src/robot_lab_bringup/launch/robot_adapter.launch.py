#!/usr/bin/env python3
"""
Robot adapter launch file for wrapping legacy robot bringup files.

This is the first adapter (P2.4) that wraps bumperbot_bringup/simulated_robot.launch.py
and preserves its public arguments until a documented deprecation.

The adapter translates robot_lab_bringup selector choices into the legacy
bumperbot_bringup configuration format.
"""

from robot_lab_bringup.adapter import generate_adapter_launch_description

# Import the adapter's launch description generator
# This wraps bumperbot_bringup/simulated_robot.launch.py with argument translation

def generate_launch_description():
    """
    Generate launch description for robot adapter.
    
    This launch file accepts robot_lab selector arguments and translates them
    to legacy bumperbot_bringup arguments, preserving all public arguments.
    """
    return generate_adapter_launch_description()
