"""
Schemas for Robot Lab Registry entities.

All schemas follow JSON Schema Draft 7 specification.
Each entity type has:
- Required fields
- Type constraints
- Pattern constraints where applicable
- Description of each field
"""

import json
from typing import Dict, Any, List, Optional
import jsonschema
from jsonschema import validate as jsonschema_validate
from jsonschema.validators import Draft7Validator


# ============================================================================
# Common Definitions
# ============================================================================

ID_PATTERN = r"^[a-zA-Z][a-zA-Z0-9_\-]*$"
VERSION_PATTERN = r"^[0-9]+\.[0-9]+\.[0-9]+$"
STATUS_OPTIONS = ["cataloged", "available", "integrated", "benchmarked", "blocked"]
ROBOT_CLASS_OPTIONS = ["mobile", "legged", "humanoid", "aerial", "manipulator", "hybrid"]
ENVIRONMENT_DIMENSION_OPTIONS = ["2D", "3D", "2.5D"]
ALGORITHM_CATEGORY_OPTIONS = [
    "perception",
    "localization", 
    "state_estimation",
    "sensor_fusion",
    "global_planning",
    "local_planning",
    "control",
]

# Common schema components
COMMON_FIELDS = {
    "type": "object",
    "properties": {
        "id": {
            "type": "string",
            "pattern": ID_PATTERN,
            "description": "Unique identifier, alphanumeric with underscores and hyphens"
        },
        "version": {
            "type": "string",
            "pattern": VERSION_PATTERN,
            "description": "Semantic version of this entity definition"
        },
        "name": {
            "type": "string",
            "description": "Human-readable name"
        },
        "description": {
            "type": "string",
            "description": "Detailed description of the entity"
        },
        "status": {
            "type": "string",
            "enum": STATUS_OPTIONS,
            "description": "Maturity status of this entity"
        },
        "source": {
            "type": "object",
            "properties": {
                "repository": {"type": "string"},
                "version": {"type": "string"},
                "license": {"type": "string"},
                "maintainer": {"type": "string"}
            },
            "required": ["repository"],
            "description": "Provenance and source information"
        },
        "tags": {
            "type": "array",
            "items": {"type": "string"},
            "description": "Free-form tags for categorization"
        },
        "created": {
            "type": "string",
            "format": "date",
            "description": "Creation date in ISO format"
        },
        "updated": {
            "type": "string",
            "format": "date",
            "description": "Last update date in ISO format"
        }
    },
    "required": ["id", "version", "name", "status"]
}

# ============================================================================
# Robot Schema
# ============================================================================

ROBOT_SCHEMA = {
    "$schema": "http://json-schema.org/draft-07/schema#",
    "$id": "https://robot-lab.schema/robot/v1",
    "title": "Robot",
    "description": "Robot description and capabilities",
    "allOf": [
        COMMON_FIELDS,
        {
            "type": "object",
            "properties": {
                "robot_class": {
                    "type": "string",
                    "enum": ROBOT_CLASS_OPTIONS,
                    "description": "Primary locomotion class"
                },
                "maturity": {
                    "type": "string",
                    "enum": ["prototype", "simulated", "tested", "production"],
                    "description": "Development maturity level"
                },
                "supported_simulators": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "List of supported simulators (e.g., gazebo, ignition, pybullet)"
                },
                "locomotion": {
                    "type": "object",
                    "properties": {
                        "type": {"type": "string"},
                        "dof": {"type": "integer", "minimum": 0},
                        "max_velocity": {"type": "number", "minimum": 0},
                        "max_acceleration": {"type": "number", "minimum": 0}
                    },
                    "required": ["type"]
                },
                "sensors": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "type": {"type": "string"},
                            "name": {"type": "string"},
                            "frame": {"type": "string"},
                            "topic": {"type": "string"},
                            "message_type": {"type": "string"}
                        },
                        "required": ["type", "name"]
                    },
                    "description": "List of onboard sensors"
                },
                "actuators": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "type": {"type": "string"},
                            "name": {"type": "string"},
                            "joint": {"type": "string"}
                        },
                        "required": ["type", "name"]
                    },
                    "description": "List of actuators"
                },
                "command_interfaces": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Supported command interface types (e.g., Twist, JointTrajectory)"
                },
                "state_interfaces": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Supported state interface types (e.g., Odometry, JointState)"
                },
                "frames": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "TF frame names defined by this robot"
                },
                "capabilities": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Declared capabilities (e.g., navigation, manipulation, perception)"
                },
                "dependencies": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "ROS package dependencies"
                },
                "assets": {
                    "type": "object",
                    "properties": {
                        "urdf": {"type": "string"},
                        "xacro": {"type": "string"},
                        "sdf": {"type": "string"},
                        "meshes": {
                            "type": "array",
                            "items": {"type": "string"}
                        }
                    },
                    "description": "Asset files for this robot"
                },
                "smoke_experiments": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Experiment IDs that justify this robot's status"
                }
            },
            "required": ["robot_class", "capabilities"]
        }
    ]
}


# ============================================================================
# Environment Schema
# ============================================================================

ENVIRONMENT_SCHEMA = {
    "$schema": "http://json-schema.org/draft-07/schema#",
    "$id": "https://robot-lab.schema/environment/v1",
    "title": "Environment",
    "description": "Simulation or real-world environment definition",
    "allOf": [
        COMMON_FIELDS,
        {
            "type": "object",
            "properties": {
                "dimension": {
                    "type": "string",
                    "enum": ENVIRONMENT_DIMENSION_OPTIONS,
                    "description": "Environment dimensionality"
                },
                "simulator": {
                    "type": "string",
                    "description": "Target simulator (e.g., gazebo, ignition, pybullet, real)"
                },
                "world_file": {
                    "type": "string",
                    "description": "Path to world/sdf file"
                },
                "occupancy_map": {
                    "type": "string",
                    "description": "Path to occupancy map if available"
                },
                "ground_truth_available": {
                    "type": "boolean",
                    "description": "Whether ground truth is available"
                },
                "dynamics": {
                    "type": "object",
                    "properties": {
                        "static_obstacles": {"type": "boolean"},
                        "dynamic_obstacles": {"type": "boolean"},
                        "max_dynamic_count": {"type": "integer", "minimum": 0}
                    },
                    "description": "Environment dynamics"
                },
                "supported_robot_classes": {
                    "type": "array",
                    "items": {"type": "string", "enum": ROBOT_CLASS_OPTIONS},
                    "description": "Robot classes this environment supports"
                },
                "spawn_zones": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "id": {"type": "string"},
                            "pose": {"type": "object"},
                            "size": {"type": "object"}
                        },
                        "required": ["id"]
                    },
                    "description": "Designated spawn zones"
                },
                "seed": {
                    "type": "integer",
                    "minimum": 0,
                    "description": "Deterministic random seed for environment stochasticity (P4.6)"
                },
                "goals": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "id": {"type": "string"},
                            "pose": {"type": "object"}
                        },
                        "required": ["id"]
                    },
                    "description": "Designated goal poses (P4.6)"
                },
                "reference_paths": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "id": {"type": "string"},
                            "waypoints": {
                                "type": "array",
                                "items": {"type": "object"}
                            }
                        },
                        "required": ["id", "waypoints"]
                    },
                    "description": "Reference navigation waypoint paths (P4.6)"
                },
                "reset_service": {
                    "type": "string",
                    "description": "Name of the environment reset service (P4.6)"
                },
                "size": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number", "minimum": 0},
                        "y": {"type": "number", "minimum": 0},
                        "z": {"type": "number", "minimum": 0}
                    },
                    "description": "Physical dimensions in meters"
                }
            },
            "required": ["dimension", "simulator"]
        }
    ]
}


# ============================================================================
# Algorithm Schema
# ============================================================================

ALGORITHM_SCHEMA = {
    "$schema": "http://json-schema.org/draft-07/schema#",
    "$id": "https://robot-lab.schema/algorithm/v1",
    "title": "Algorithm",
    "description": "Algorithm/implementation catalog entry",
    "allOf": [
        COMMON_FIELDS,
        {
            "type": "object",
            "properties": {
                "category": {
                    "type": "string",
                    "enum": ALGORITHM_CATEGORY_OPTIONS,
                    "description": "Algorithm category"
                },
                "family": {
                    "type": "string",
                    "description": "Algorithm family (e.g., kalman_filter, mpc, rrt)"
                },
                "implementation": {
                    "type": "object",
                    "properties": {
                        "package": {"type": "string"},
                        "plugin": {"type": "string"},
                        "executable": {"type": "string"},
                        "language": {"type": "string", "enum": ["cpp", "python", "rust"]},
                        "entry_point": {"type": "string"}
                    },
                    "required": ["package"],
                    "description": "Implementation details"
                },
                "input_contract": {
                    "type": "object",
                    "properties": {
                        "required_topics": {
                            "type": "array",
                            "items": {"type": "string"}
                        },
                        "required_actions": {
                            "type": "array",
                            "items": {"type": "string"}
                        },
                        "required_services": {
                            "type": "array",
                            "items": {"type": "string"}
                        },
                        "required_parameters": {
                            "type": "array",
                            "items": {"type": "string"}
                        }
                    },
                    "description": "Input interface contract"
                },
                "output_contract": {
                    "type": "object",
                    "properties": {
                        "provided_topics": {
                            "type": "array",
                            "items": {"type": "string"}
                        },
                        "provided_actions": {
                            "type": "array",
                            "items": {"type": "string"}
                        },
                        "provided_services": {
                            "type": "array",
                            "items": {"type": "string"}
                        }
                    },
                    "description": "Output interface contract"
                },
                "required_capabilities": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Robot capabilities required by this algorithm"
                },
                "supported_robot_classes": {
                    "type": "array",
                    "items": {"type": "string", "enum": ROBOT_CLASS_OPTIONS},
                    "description": "Robot classes this algorithm supports"
                },
                "parameters": {
                    "type": "object",
                    "additionalProperties": True,
                    "description": "Default/required parameters"
                },
                "smoke_experiment": {
                    "type": "string",
                    "description": "Experiment ID that demonstrates this algorithm"
                }
            },
            "required": ["category", "implementation", "input_contract", "output_contract"]
        }
    ]
}


# ============================================================================
# Scenario Schema
# ============================================================================

SCENARIO_SCHEMA = {
    "$schema": "http://json-schema.org/draft-07/schema#",
    "$id": "https://robot-lab.schema/scenario/v1",
    "title": "Scenario",
    "description": "Task definition independent of specific robot/environment",
    "allOf": [
        COMMON_FIELDS,
        {
            "type": "object",
            "properties": {
                "task_type": {
                    "type": "string",
                    "enum": [
                        "navigation",
                        "exploration",
                        "relocalization",
                        "coverage",
                        "traversal",
                        "manipulation",
                        "inspection",
                        "search_and_rescue",
                        "formation",
                        "smoke_test",
                        "custom"
                    ],
                    "description": "Type of task"
                },
                "task_description": {
                    "type": "string",
                    "description": "Detailed task description"
                },
                "stopping_conditions": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "type": {"type": "string", "enum": ["success", "failure", "timeout", "custom"]},
                            "condition": {"type": "string"}
                        },
                        "required": ["type"]
                    },
                    "description": "Conditions that stop the scenario"
                },
                "success_criteria": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Criteria for successful completion"
                },
                "failure_criteria": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Criteria for failure"
                },
                "required_robot_classes": {
                    "type": "array",
                    "items": {"type": "string", "enum": ROBOT_CLASS_OPTIONS},
                    "description": "Robot classes this scenario requires"
                },
                "required_capabilities": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Capabilities required from the robot"
                },
                "timeout": {
                    "type": "number",
                    "minimum": 0,
                    "description": "Default timeout in seconds"
                },
                "metrics": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Metrics to record for this scenario"
                }
            },
            "required": ["task_type", "task_description", "stopping_conditions"]
        }
    ]
}


# ============================================================================
# Experiment Schema
# ============================================================================

EXPERIMENT_SCHEMA = {
    "$schema": "http://json-schema.org/draft-07/schema#",
    "$id": "https://robot-lab.schema/experiment/v1",
    "title": "Experiment",
    "description": "Complete experiment specification",
    "allOf": [
        COMMON_FIELDS,
        {
            "type": "object",
            "properties": {
                "robot_id": {
                    "type": "string",
                    "description": "Reference to robot catalog entry"
                },
                "environment_id": {
                    "type": "string",
                    "description": "Reference to environment catalog entry"
                },
                "simulator": {
                    "type": "string",
                    "description": "Simulator to use"
                },
                "scenario_id": {
                    "type": "string",
                    "description": "Reference to scenario catalog entry"
                },
                "algorithm_ids": {
                    "type": "object",
                    "properties": {
                        "perception": {"type": "string"},
                        "localization": {"type": "string"},
                        "state_estimation": {"type": "string"},
                        "sensor_fusion": {"type": "string"},
                        "global_planning": {"type": "string"},
                        "local_planning": {"type": "string"},
                        "control": {"type": "string"}
                    },
                    "description": "Selected algorithms for each category"
                },
                "seed": {
                    "type": "integer",
                    "minimum": 0,
                    "description": "Random seed for reproducibility"
                },
                "time_limit": {
                    "type": "number",
                    "minimum": 0,
                    "description": "Maximum wall-clock time in seconds"
                },
                "simulation_time_limit": {
                    "type": "number",
                    "minimum": 0,
                    "description": "Maximum simulation time in seconds"
                },
                "parameters": {
                    "type": "object",
                    "additionalProperties": True,
                    "description": "Parameter overrides for this experiment"
                },
                "metrics": {
                    "type": "object",
                    "additionalProperties": {"type": "boolean"},
                    "description": "Which metrics to record"
                },
                "artifacts": {
                    "type": "object",
                    "properties": {
                        "record_bag": {"type": "boolean"},
                        "record_logs": {"type": "boolean"},
                        "record_maps": {"type": "boolean"},
                        "record_trajectories": {"type": "boolean"},
                        "record_plots": {"type": "boolean"}
                    },
                    "description": "Artifact recording policy"
                },
                "namespace": {
                    "type": "string",
                    "description": "ROS namespace for multi-robot experiments"
                },
                "namespaced": {
                    "type": "boolean",
                    "description": "Whether to use namespacing"
                }
            },
            "required": ["robot_id", "environment_id", "simulator", "algorithm_ids"]
        }
    ]
}


# ============================================================================
# Schema Registry
# ============================================================================

SCHEMAS = {
    "robot": ROBOT_SCHEMA,
    "environment": ENVIRONMENT_SCHEMA,
    "algorithm": ALGORITHM_SCHEMA,
    "scenario": SCENARIO_SCHEMA,
    "experiment": EXPERIMENT_SCHEMA
}


def validate_entity(entity_type: str, data: Dict[str, Any]) -> tuple:
    """
    Validate an entity against its schema.
    
    Args:
        entity_type: One of 'robot', 'environment', 'algorithm', 'scenario', 'experiment'
        data: Entity data as dictionary
        
    Returns:
        Tuple of (is_valid: bool, errors: List[str])
    """
    if entity_type not in SCHEMAS:
        return False, [f"Unknown entity type: {entity_type}. Valid types: {list(SCHEMAS.keys())}"]
    
    schema = SCHEMAS[entity_type]
    validator = Draft7Validator(schema)
    errors = []
    
    for error in validator.iter_errors(data):
        # Build a readable error message
        path = ".".join(str(p) for p in error.absolute_path) if error.absolute_path else "root"
        if error.validator == 'required':
            errors.append(f"{path}: missing required field '{error.message}'")
        elif error.validator == 'enum':
            errors.append(f"{path}: '{data.get(path.split('.')[-1], '?')}' is not one of {error.schema['enum']}")
        elif error.validator == 'pattern':
            errors.append(f"{path}: '{data.get(path.split('.')[-1], '?')}' does not match pattern '{error.schema['pattern']}'")
        elif error.validator == 'type':
            expected = error.schema['type']
            if isinstance(expected, list):
                expected_str = " or ".join(expected)
            else:
                expected_str = expected
            errors.append(f"{path}: expected {expected_str}, got {type(data.get(path.split('.')[-1], None)).__name__}")
        else:
            errors.append(f"{path}: {error.message}")
    
    return len(errors) == 0, errors


def get_schema(entity_type: str) -> Optional[Dict]:
    """Get schema for a given entity type."""
    return SCHEMAS.get(entity_type)


def get_all_schemas() -> Dict[str, Dict]:
    """Get all schemas."""
    return SCHEMAS.copy()
