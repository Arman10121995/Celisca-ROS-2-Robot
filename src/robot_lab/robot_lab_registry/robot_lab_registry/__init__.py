# Robot Lab Registry Package
# Version 0.1.0
# Canonical catalogs, schemas, and validation for the unified multi-robot platform

"""
Robot Lab Registry provides:
- Schemas for robots, environments, algorithms, scenarios, and experiments
- Catalog management and query APIs
- Validation CLI for composition compatibility
- Status tracking and ledger integration
"""

from .schemas import (
    ROBOT_SCHEMA,
    ENVIRONMENT_SCHEMA,
    ALGORITHM_SCHEMA,
    SCENARIO_SCHEMA,
    EXPERIMENT_SCHEMA,
    validate_entity,
)

from .catalog import (
    RobotCatalog,
    EnvironmentCatalog,
    AlgorithmCatalog,
    ScenarioCatalog,
    ExperimentCatalog,
)

from .validation import (
    check_composition,
    check_capabilities,
    validate_cross_references,
    Composition,
    ValidationResult,
)

from .query import (
    list_robots,
    list_environments,
    list_algorithms,
    list_scenarios,
    list_experiments,
    get_entity,
    search_entities,
)

__version__ = "0.1.0"
__schema_version__ = 1

# Package metadata
PACKAGE_NAME = "robot_lab_registry"
CONFIG_DIR = "config"
RESOURCE_DIR = "resource"

# Default catalog paths
DEFAULT_ROBOT_CATALOG = f"{CONFIG_DIR}/robots.yaml"
DEFAULT_ENVIRONMENT_CATALOG = f"{CONFIG_DIR}/environments.yaml"
DEFAULT_ALGORITHM_CATALOG = f"{CONFIG_DIR}/algorithms.yaml"
DEFAULT_SCENARIO_CATALOG = f"{CONFIG_DIR}/scenarios.yaml"
DEFAULT_EXPERIMENT_CATALOG = f"{CONFIG_DIR}/experiments.yaml"
