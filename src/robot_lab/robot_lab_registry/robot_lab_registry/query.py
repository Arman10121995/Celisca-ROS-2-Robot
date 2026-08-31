"""
Query interface for Robot Lab Registry.

Provides convenient functions for listing, describing, and searching entities.
"""

from typing import Dict, List, Any, Optional, Union
from pathlib import Path

from .catalog import Registry, RobotCatalog, EnvironmentCatalog, AlgorithmCatalog, ScenarioCatalog, ExperimentCatalog
from .schemas import SCHEMAS, ALGORITHM_CATEGORY_OPTIONS, ROBOT_CLASS_OPTIONS


def get_registry(config_dir: Optional[Union[str, Path]] = None) -> Registry:
    """
    Get or create a registry instance.
    
    Args:
        config_dir: Directory containing catalog files
        
    Returns:
        Registry instance
    """
    registry = Registry(config_dir)
    if config_dir:
        registry.load(config_dir)
    return registry


# ============================================================================
# List Functions
# ============================================================================

def list_robots(
    config_dir: Optional[Union[str, Path]] = None,
    robot_class: Optional[str] = None,
    status: Optional[str] = None
) -> List[Dict[str, Any]]:
    """
    List all robots, optionally filtered by class or status.
    
    Args:
        config_dir: Directory containing catalog files
        robot_class: Filter by robot class (mobile, legged, humanoid, aerial)
        status: Filter by status (cataloged, available, integrated, benchmarked, blocked)
        
    Returns:
        List of robot entities
    """
    registry = get_registry(config_dir)
    robots = list(registry.robots.get_all().values())
    
    if robot_class:
        robots = [r for r in robots if r.get('robot_class') == robot_class]
    if status:
        robots = [r for r in robots if r.get('status') == status]
    
    return robots


def list_environments(
    config_dir: Optional[Union[str, Path]] = None,
    simulator: Optional[str] = None,
    dimension: Optional[str] = None,
    status: Optional[str] = None
) -> List[Dict[str, Any]]:
    """
    List all environments, optionally filtered.
    
    Args:
        config_dir: Directory containing catalog files
        simulator: Filter by simulator
        dimension: Filter by dimension (2D, 3D, 2.5D)
        status: Filter by status
        
    Returns:
        List of environment entities
    """
    registry = get_registry(config_dir)
    environments = list(registry.environments.get_all().values())
    
    if simulator:
        environments = [e for e in environments if e.get('simulator') == simulator]
    if dimension:
        environments = [e for e in environments if e.get('dimension') == dimension]
    if status:
        environments = [e for e in environments if e.get('status') == status]
    
    return environments


def list_algorithms(
    config_dir: Optional[Union[str, Path]] = None,
    category: Optional[str] = None,
    family: Optional[str] = None,
    status: Optional[str] = None,
    robot_class: Optional[str] = None
) -> List[Dict[str, Any]]:
    """
    List all algorithms, optionally filtered.
    
    Args:
        config_dir: Directory containing catalog files
        category: Filter by category
        family: Filter by algorithm family
        status: Filter by status
        robot_class: Filter by supported robot class
        
    Returns:
        List of algorithm entities
    """
    registry = get_registry(config_dir)
    algorithms = list(registry.algorithms.get_all().values())
    
    if category:
        algorithms = [a for a in algorithms if a.get('category') == category]
    if family:
        algorithms = [a for a in algorithms if a.get('family') == family]
    if status:
        algorithms = [a for a in algorithms if a.get('status') == status]
    if robot_class:
        algorithms = [
            a for a in algorithms 
            if robot_class in a.get('supported_robot_classes', [])
        ]
    
    return algorithms


def list_scenarios(
    config_dir: Optional[Union[str, Path]] = None,
    task_type: Optional[str] = None,
    status: Optional[str] = None
) -> List[Dict[str, Any]]:
    """
    List all scenarios, optionally filtered.
    
    Args:
        config_dir: Directory containing catalog files
        task_type: Filter by task type
        status: Filter by status
        
    Returns:
        List of scenario entities
    """
    registry = get_registry(config_dir)
    scenarios = list(registry.scenarios.get_all().values())
    
    if task_type:
        scenarios = [s for s in scenarios if s.get('task_type') == task_type]
    if status:
        scenarios = [s for s in scenarios if s.get('status') == status]
    
    return scenarios


def list_experiments(
    config_dir: Optional[Union[str, Path]] = None,
    robot_id: Optional[str] = None,
    environment_id: Optional[str] = None,
    status: Optional[str] = None
) -> List[Dict[str, Any]]:
    """
    List all experiments, optionally filtered.
    
    Args:
        config_dir: Directory containing catalog files
        robot_id: Filter by robot ID
        environment_id: Filter by environment ID
        status: Filter by status
        
    Returns:
        List of experiment entities
    """
    registry = get_registry(config_dir)
    experiments = list(registry.experiments.get_all().values())
    
    if robot_id:
        experiments = [e for e in experiments if e.get('robot_id') == robot_id]
    if environment_id:
        experiments = [e for e in experiments if e.get('environment_id') == environment_id]
    if status:
        experiments = [e for e in experiments if e.get('status') == status]
    
    return experiments


# ============================================================================
# Get/Describe Functions
# ============================================================================

def get_entity(
    entity_type: str,
    entity_id: str,
    config_dir: Optional[Union[str, Path]] = None
) -> Optional[Dict[str, Any]]:
    """
    Get a specific entity by type and ID.
    
    Args:
        entity_type: Type of entity (robot, environment, algorithm, scenario, experiment)
        entity_id: ID of the entity
        config_dir: Directory containing catalog files
        
    Returns:
        Entity dictionary or None if not found
    """
    if entity_type not in SCHEMAS:
        return None
    
    registry = get_registry(config_dir)
    
    if entity_type == 'robot':
        return registry.robots.get(entity_id)
    elif entity_type == 'environment':
        return registry.environments.get(entity_id)
    elif entity_type == 'algorithm':
        return registry.algorithms.get(entity_id)
    elif entity_type == 'scenario':
        return registry.scenarios.get(entity_id)
    elif entity_type == 'experiment':
        return registry.experiments.get(entity_id)
    
    return None


def describe_entity(
    entity_type: str,
    entity_id: str,
    config_dir: Optional[Union[str, Path]] = None,
    verbose: bool = False
) -> str:
    """
    Get a formatted description of an entity.
    
    Args:
        entity_type: Type of entity
        entity_id: ID of the entity
        config_dir: Directory containing catalog files
        verbose: Include all fields
        
    Returns:
        Formatted string description
    """
    entity = get_entity(entity_type, entity_id, config_dir)
    if not entity:
        return f"Entity '{entity_id}' of type '{entity_type}' not found"
    
    lines = []
    lines.append(f"=== {entity_type.upper()}: {entity_id} ===")
    lines.append(f"Name: {entity.get('name', 'N/A')}")
    lines.append(f"Version: {entity.get('version', 'N/A')}")
    lines.append(f"Status: {entity.get('status', 'N/A')}")
    
    if verbose:
        lines.append("")
        lines.append("--- Full Details ---")
        for key, value in entity.items():
            if key not in ['id', 'name', 'version', 'status']:
                lines.append(f"{key}: {value}")
    else:
        # Type-specific highlights
        if entity_type == 'robot':
            lines.append(f"Class: {entity.get('robot_class', 'N/A')}")
            lines.append(f"Capabilities: {', '.join(entity.get('capabilities', []))}")
        elif entity_type == 'environment':
            lines.append(f"Dimension: {entity.get('dimension', 'N/A')}")
            lines.append(f"Simulator: {entity.get('simulator', 'N/A')}")
        elif entity_type == 'algorithm':
            lines.append(f"Category: {entity.get('category', 'N/A')}")
            lines.append(f"Family: {entity.get('family', 'N/A')}")
        elif entity_type == 'scenario':
            lines.append(f"Task Type: {entity.get('task_type', 'N/A')}")
        elif entity_type == 'experiment':
            lines.append(f"Robot: {entity.get('robot_id', 'N/A')}")
            lines.append(f"Environment: {entity.get('environment_id', 'N/A')}")
    
    return "\n".join(lines)


# ============================================================================
# Search Functions
# ============================================================================

def search_entities(
    query: str,
    entity_type: Optional[str] = None,
    config_dir: Optional[Union[str, Path]] = None
) -> Dict[str, List[Dict[str, Any]]]:
    """
    Search across all entity types.
    
    Args:
        query: Text to search for
        entity_type: Optional filter by entity type
        config_dir: Directory containing catalog files
        
    Returns:
        Dictionary mapping entity types to matching entities
    """
    registry = get_registry(config_dir)
    results = {}
    
    entity_types = [entity_type] if entity_type else ['robot', 'environment', 'algorithm', 'scenario', 'experiment']
    
    for etype in entity_types:
        if etype not in SCHEMAS:
            continue
        
        if etype == 'robot':
            matches = registry.robots.search(query)
        elif etype == 'environment':
            matches = registry.environments.search(query)
        elif etype == 'algorithm':
            matches = registry.algorithms.search(query)
        elif etype == 'scenario':
            matches = registry.scenarios.search(query)
        elif etype == 'experiment':
            matches = registry.experiments.search(query)
        else:
            continue
        
        if matches:
            results[etype] = list(matches.values())
    
    return results


# ============================================================================
# Summary/Statistics Functions
# ============================================================================

def get_summary(
    config_dir: Optional[Union[str, Path]] = None
) -> Dict[str, Any]:
    """
    Get a summary of the registry.
    
    Args:
        config_dir: Directory containing catalog files
        
    Returns:
        Summary dictionary with counts by type and status
    """
    registry = get_registry(config_dir)
    
    summary = {
        "total_robots": registry.robots.count(),
        "total_environments": registry.environments.count(),
        "total_algorithms": registry.algorithms.count(),
        "total_scenarios": registry.scenarios.count(),
        "total_experiments": registry.experiments.count(),
        "by_category": {},
        "by_robot_class": {},
        "by_status": {}
    }
    
    # Count by category (algorithms)
    for algo in registry.algorithms.get_all().values():
        category = algo.get('category', 'unknown')
        summary["by_category"][category] = summary["by_category"].get(category, 0) + 1
    
    # Count by robot class
    for robot in registry.robots.get_all().values():
        rclass = robot.get('robot_class', 'unknown')
        summary["by_robot_class"][rclass] = summary["by_robot_class"].get(rclass, 0) + 1
    
    # Count by status for all entity types
    for etype in ['robot', 'environment', 'algorithm', 'scenario', 'experiment']:
        catalog = getattr(registry, f"{etype}s")
        for entity in catalog.get_all().values():
            status = entity.get('status', 'unknown')
            key = f"{etype}_by_status"
            if key not in summary:
                summary[key] = {}
            summary[key][status] = summary[key].get(status, 0) + 1
    
    return summary


def get_status_counts(
    config_dir: Optional[Union[str, Path]] = None
) -> Dict[str, Dict[str, int]]:
    """
    Get counts of entities by status for each type.
    
    Args:
        config_dir: Directory containing catalog files
        
    Returns:
        Nested dictionary with status counts
    """
    summary = get_summary(config_dir)
    counts = {}
    
    for etype in ['robot', 'environment', 'algorithm', 'scenario', 'experiment']:
        key = f"{etype}_by_status"
        if key in summary:
            counts[etype] = summary[key]
    
    return counts
