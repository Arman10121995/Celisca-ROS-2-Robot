# Copyright 2026 Bumperbot contributors
# Licensed under the Apache License, Version 2.0

import ast
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
MODULE_DIR = PACKAGE_DIR / "bumperbot_py_examples"


ENTRY_POINTS = {
    "simple_publisher": "simple_publisher",
    "simple_subscriber": "simple_subscriber",
    "simple_parameter": "simple_parameter",
    "simple_turtlesim_kinematics": "simple_turtlesim_kinematics",
    "simple_service_server": "simple_service_server",
    "simple_service_client": "simple_service_client",
    "simple_tf_kinematics": "simple_tf_kinematics",
    "simple_action_server": "simple_action_server",
    "simple_action_client": "simple_action_client",
    "simple_lifecycle_node": "simple_lifecycle_node",
    "simple_qos_publisher": "simple_qos_publisher",
    "simple_qos_subscriber": "simple_qos_subscriber",
    "lidar_subscriber": "lidar_subscriber",
}


def test_all_console_entry_points_have_a_main_function():
    setup_text = (PACKAGE_DIR / "setup.py").read_text(encoding="utf-8")
    for executable, module in ENTRY_POINTS.items():
        module_path = MODULE_DIR / f"{module}.py"
        tree = ast.parse(module_path.read_text(encoding="utf-8"), module_path)
        functions = {
            node.name for node in tree.body if isinstance(node, ast.FunctionDef)
        }
        assert "main" in functions, f"{module_path} has no main()"
        expected_registration = (
            f"{executable} = bumperbot_py_examples.{module}:main"
        )
        assert expected_registration in setup_text
