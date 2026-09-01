#!/usr/bin/env python3
"""
Command-line interface for Robot Lab Registry.

Provides validation, querying, and management of the registry catalogs.
"""

import argparse
import sys
import json
import yaml
from pathlib import Path
from typing import Optional, List, Dict, Any

from .schemas import (
    SCHEMAS,
    validate_entity,
    get_schema,
    get_all_schemas
)
from .catalog import Registry
from .validation import (
    validate_composition_from_dict,
    validate_cross_references,
    check_composition,
    Composition
)
from .query import (
    list_robots,
    list_environments,
    list_algorithms,
    list_scenarios,
    list_experiments,
    get_entity,
    describe_entity,
    search_entities,
    get_summary,
    get_status_counts
)


def print_yaml(data: Any) -> None:
    """Print data as YAML."""
    print(yaml.dump(data, default_flow_style=False, sort_keys=False))


def print_json(data: Any) -> None:
    """Print data as JSON."""
    print(json.dumps(data, indent=2))


def print_table(headers: List[str], rows: List[List[str]]) -> None:
    """Print data as a simple table."""
    # Calculate column widths
    col_widths = [max(len(str(row[i])) for row in [headers] + rows) for i in range(len(headers))]
    
    # Print header
    header_line = " | ".join(headers[i].ljust(col_widths[i]) for i in range(len(headers)))
    print(header_line)
    print("-" * len(header_line))
    
    # Print rows
    for row in rows:
        print(" | ".join(str(row[i]).ljust(col_widths[i]) for i in range(len(headers))))


# ============================================================================
# Main CLI Functions
# ============================================================================

def cmd_list(args) -> int:
    """Handle 'list' command."""
    entity_type = args.entity_type
    
    # Map verb to list function
    list_funcs = {
        'robots': list_robots,
        'environments': list_environments,
        'algorithms': list_algorithms,
        'scenarios': list_scenarios,
        'experiments': list_experiments
    }
    
    if entity_type not in list_funcs:
        print(f"Unknown entity type: {entity_type}")
        print(f"Valid types: {', '.join(list_funcs.keys())}")
        return 1
    
    # Build kwargs
    kwargs = {}
    if entity_type == 'robots':
        if hasattr(args, 'robot_class') and args.robot_class:
            kwargs['robot_class'] = args.robot_class
    elif entity_type == 'environments':
        if hasattr(args, 'simulator') and args.simulator:
            kwargs['simulator'] = args.simulator
        if hasattr(args, 'dimension') and args.dimension:
            kwargs['dimension'] = args.dimension
    elif entity_type == 'algorithms':
        if hasattr(args, 'category') and args.category:
            kwargs['category'] = args.category
        if hasattr(args, 'family') and args.family:
            kwargs['family'] = args.family
        if hasattr(args, 'robot_class') and args.robot_class:
            kwargs['robot_class'] = args.robot_class
    elif entity_type == 'scenarios':
        if hasattr(args, 'task_type') and args.task_type:
            kwargs['task_type'] = args.task_type
    elif entity_type == 'experiments':
        if hasattr(args, 'robot_id') and args.robot_id:
            kwargs['robot_id'] = args.robot_id
        if hasattr(args, 'environment_id') and args.environment_id:
            kwargs['environment_id'] = args.environment_id
    
    if hasattr(args, 'status') and args.status:
        kwargs['status'] = args.status
    
    # Get config directory
    config_dir = args.config_dir if hasattr(args, 'config_dir') else None
    
    # Call list function
    entities = list_funcs[entity_type](config_dir=config_dir, **kwargs)
    
    if not entities:
        print(f"No {entity_type} found")
        return 0
    
    # Format output
    if args.format == 'json':
        print_json(entities)
    elif args.format == 'yaml':
        print_yaml(entities)
    else:
        # Table format
        if entity_type == 'robots':
            headers = ['ID', 'Name', 'Class', 'Status']
            rows = [[e.get('id', ''), e.get('name', ''), e.get('robot_class', ''), e.get('status', '')] for e in entities]
        elif entity_type == 'environments':
            headers = ['ID', 'Name', 'Dimension', 'Simulator', 'Status']
            rows = [[e.get('id', ''), e.get('name', ''), e.get('dimension', ''), e.get('simulator', ''), e.get('status', '')] for e in entities]
        elif entity_type == 'algorithms':
            headers = ['ID', 'Name', 'Category', 'Family', 'Status']
            rows = [[e.get('id', ''), e.get('name', ''), e.get('category', ''), e.get('family', ''), e.get('status', '')] for e in entities]
        elif entity_type == 'scenarios':
            headers = ['ID', 'Name', 'Task Type', 'Status']
            rows = [[e.get('id', ''), e.get('name', ''), e.get('task_type', ''), e.get('status', '')] for e in entities]
        elif entity_type == 'experiments':
            headers = ['ID', 'Name', 'Robot', 'Environment', 'Status']
            rows = [[e.get('id', ''), e.get('name', ''), e.get('robot_id', ''), e.get('environment_id', ''), e.get('status', '')] for e in entities]
        else:
            headers = ['ID', 'Name', 'Status']
            rows = [[e.get('id', ''), e.get('name', ''), e.get('status', '')] for e in entities]
        
        print_table(headers, rows)
    
    return 0


def cmd_describe(args) -> int:
    """Handle 'describe' command."""
    entity = get_entity(args.entity_type, args.entity_id, args.config_dir)
    
    if not entity:
        print(f"Entity '{args.entity_id}' of type '{args.entity_type}' not found")
        return 1
    
    if args.format == 'json':
        print_json(entity)
    elif args.format == 'yaml':
        print_yaml(entity)
    else:
        print(describe_entity(args.entity_type, args.entity_id, args.config_dir, args.verbose))
    
    return 0


def cmd_search(args) -> int:
    """Handle 'search' command."""
    results = search_entities(args.query, args.entity_type, args.config_dir)
    
    if not results:
        print(f"No matches found for '{args.query}'")
        return 0
    
    if args.format == 'json':
        print_json(results)
    elif args.format == 'yaml':
        print_yaml(results)
    else:
        for entity_type, entities in results.items():
            print(f"\n{entity_type.upper()} ({len(entities)}):")
            for entity in entities:
                print(f"  - {entity.get('id')}: {entity.get('name')}")
    
    return 0


def cmd_validate(args) -> int:
    """Handle 'validate' command."""
    registry = Registry(args.config_dir)
    
    if not registry.load(args.config_dir):
        print(f"Failed to load registry from {args.config_dir}")
        return 1
    
    # Determine what to validate
    if args.entity_type:
        # Validate specific entity type
        catalog = getattr(registry, f"{args.entity_type}s", None)
        if not catalog:
            print(f"Unknown entity type: {args.entity_type}")
            return 1
        
        is_valid, errors = catalog.validate_all()
        
    elif args.cross_references:
        # Validate cross-references
        result = validate_cross_references(registry)
        is_valid = result.valid
        errors = result.errors + result.warnings
    
    else:
        # Validate everything
        is_valid, errors = registry.validate()
        
        # Also check cross-references
        xref_result = validate_cross_references(registry)
        is_valid = is_valid and xref_result.valid
        errors.extend(xref_result.errors)
    
    if is_valid:
        print("Validation passed")
        return 0
    else:
        print("Validation failed:")
        for error in errors:
            print(f"  - {error}")
        return 1


def cmd_check_composition(args) -> int:
    """Handle 'check-composition' command."""
    # Try to load from file first
    composition_data = None
    
    if args.composition_file:
        try:
            with open(args.composition_file, 'r') as f:
                if args.composition_file.endswith('.yaml') or args.composition_file.endswith('.yml'):
                    composition_data = yaml.safe_load(f)
                else:
                    composition_data = json.load(f)
        except Exception as e:
            print(f"Failed to load composition file: {e}")
            return 1
    elif args.composition:
        # Parse inline JSON
        try:
            composition_data = json.loads(args.composition)
        except json.JSONDecodeError as e:
            print(f"Invalid JSON in composition: {e}")
            return 1
    else:
        print("Please provide a composition file or inline JSON")
        return 1
    
    # Load registry
    registry = Registry(args.config_dir)
    if not registry.load(args.config_dir):
        print(f"Failed to load registry")
        return 1
    
    # Convert to Composition object
    try:
        composition = Composition(
            robot_id=composition_data.get('robot_id', ''),
            environment_id=composition_data.get('environment_id', ''),
            simulator=composition_data.get('simulator', ''),
            scenario_id=composition_data.get('scenario_id'),
            algorithm_ids=composition_data.get('algorithm_ids', {})
        )
    except Exception as e:
        print(f"Invalid composition format: {e}")
        return 1
    
    # Validate
    from .validation import check_composition
    result = check_composition(registry, composition)
    
    if result.valid:
        print("Composition is valid")
        if result.warnings:
            print("\nWarnings:")
            for warning in result.warnings:
                print(f"  - {warning}")
        return 0
    else:
        print("Composition is invalid:")
        for error in result.errors:
            print(f"  - {error}")
        if result.warnings:
            print("\nWarnings:")
            for warning in result.warnings:
                print(f"  - {warning}")
        return 1


def cmd_summary(args) -> int:
    """Handle 'summary' command."""
    summary = get_summary(args.config_dir)
    
    if args.format == 'json':
        print_json(summary)
    elif args.format == 'yaml':
        print_yaml(summary)
    else:
        print("Robot Lab Registry Summary")
        print("=" * 40)
        print(f"Total Robots: {summary['total_robots']}")
        print(f"Total Environments: {summary['total_environments']}")
        print(f"Total Algorithms: {summary['total_algorithms']}")
        print(f"Total Scenarios: {summary['total_scenarios']}")
        print(f"Total Experiments: {summary['total_experiments']}")
        
        print("\nBy Algorithm Category:")
        for category, count in summary.get('by_category', {}).items():
            print(f"  {category}: {count}")
        
        print("\nBy Robot Class:")
        for rclass, count in summary.get('by_robot_class', {}).items():
            print(f"  {rclass}: {count}")
    
    return 0


def cmd_schema(args) -> int:
    """Handle 'schema' command."""
    if args.entity_type:
        schema = get_schema(args.entity_type)
        if not schema:
            print(f"Unknown entity type: {args.entity_type}")
            return 1
        
        if args.format == 'json':
            print_json(schema)
        else:
            print_yaml(schema)
    else:
        # List all schemas
        print("Available schemas:")
        for name in SCHEMAS.keys():
            print(f"  - {name}")
    
    return 0


def cmd_launch(args) -> int:
    """
    Handle 'launch' command.
    
    Generates launch configuration for a composition, with --dry-run showing
    the resolved configuration without executing it.
    """
    import os
    from pathlib import Path
    
    # Load registry
    registry = Registry(args.config_dir)
    if not registry.load(args.config_dir):
        print(f"Failed to load registry from {args.config_dir}")
        return 1
    
    # Parse composition specification
    composition_data = {}
    if args.composition_file:
        try:
            with open(args.composition_file, 'r') as f:
                composition_data = yaml.safe_load(f)
        except Exception as e:
            print(f"Failed to load composition file: {e}")
            return 1
    elif args.composition:
        try:
            composition_data = json.loads(args.composition)
        except json.JSONDecodeError as e:
            print(f"Invalid JSON composition: {e}")
            return 1
    else:
        print("Please provide a composition file or inline JSON")
        return 1
    
    # Convert to Composition object
    from .validation import Composition, check_composition
    try:
        composition = Composition(
            robot_id=composition_data.get('robot_id', ''),
            environment_id=composition_data.get('environment_id', ''),
            simulator=composition_data.get('simulator', ''),
            scenario_id=composition_data.get('scenario_id'),
            algorithm_ids=composition_data.get('algorithm_ids', {})
        )
    except Exception as e:
        print(f"Invalid composition: {e}")
        return 1
    
    # Validate composition
    result = check_composition(registry, composition)
    if not result.valid:
        print("Composition is invalid:")
        for error in result.errors:
            print(f"  - {error}")
        return 1
    
    # Generate launch configuration (dry-run)
    print("Launch Configuration (dry-run)")
    print("=" * 60)
    print(f"\nComposition:")
    print(f"  Robot: {composition.robot_id}")
    print(f"  Environment: {composition.environment_id}")
    print(f"  Simulator: {composition.simulator}")
    if composition.scenario_id:
        print(f"  Scenario: {composition.scenario_id}")
    
    print(f"\nAlgorithms:")
    for category, algo_id in composition.algorithm_ids.items():
        if algo_id:
            algo = registry.algorithms.get(algo_id)
            if algo:
                print(f"  {category}: {algo_id} ({algo.get('name', 'Unknown')})")
            else:
                print(f"  {category}: {algo_id}")
    
    # Print resolved configuration
    print(f"\nResolved Configuration:")
    robot = registry.robots.get(composition.robot_id)
    if robot:
        print(f"  Robot Package: {robot.get('ros_package', 'unknown')}")
        print(f"  Robot Name: {robot.get('name', 'unknown')}")
    
    environment = registry.environments.get(composition.environment_id)
    if environment:
        print(f"  Environment Package: {environment.get('ros_package', 'unknown')}")
        print(f"  World File: {environment.get('world_file', 'unknown')}")
    
    print(f"\nDependencies:")
    deps = set()
    deps.add(robot.get('ros_package', 'unknown')) if robot else None
    deps.add(environment.get('ros_package', 'unknown')) if environment else None
    for algo_id in composition.algorithm_ids.values():
        if algo_id:
            algo = registry.algorithms.get(algo_id)
            if algo and algo.get('ros_package'):
                deps.add(algo.get('ros_package'))
    
    if deps:
        for dep in sorted(deps):
            if dep != 'unknown':
                print(f"  - {dep}")
    
    print(f"\nWarnings:" if result.warnings else "\nNo warnings.")
    for warning in result.warnings:
        print(f"  - {warning}")
    
    if not args.dry_run:
        print("\nNote: Pass --no-dry-run to actually launch (not yet implemented)")
    
    return 0


def cmd_doctor(args) -> int:
    """
    Handle 'doctor' command.
    
    Performs diagnostic checks on the Robot Lab environment:
    - ROS installation and version
    - Required packages and dependencies
    - Registry integrity
    - Asset availability
    - Simulation environment setup
    """
    import subprocess
    import shutil
    
    print("Robot Lab Doctor - Workspace Diagnostics")
    print("=" * 60)
    
    # Check 1: ROS installation
    print("\n[1] ROS Installation")
    print("-" * 40)
    try:
        result = subprocess.run(['ros2', '--version'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✓ ROS 2 installed: {result.stdout.strip()}")
        else:
            print(f"✗ ROS 2 found but error: {result.stderr}")
    except FileNotFoundError:
        print("✗ ROS 2 not found in PATH")
    except subprocess.TimeoutExpired:
        print("✗ ROS 2 command timed out")
    
    # Check 2: Required tools
    print("\n[2] Required Tools")
    print("-" * 40)
    tools = ['colcon', 'python3', 'gazebo']
    for tool in tools:
        if shutil.which(tool):
            try:
                result = subprocess.run([tool, '--version'], capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    print(f"✓ {tool}: {result.stdout.strip().split(chr(10))[0]}")
                else:
                    print(f"✓ {tool}: found")
            except (subprocess.TimeoutExpired, Exception):
                print(f"✓ {tool}: found")
        else:
            print(f"✗ {tool}: not found in PATH")
    
    # Check 3: Registry
    print("\n[3] Registry Status")
    print("-" * 40)
    try:
        registry = Registry(args.config_dir)
        if registry.load(args.config_dir):
            robots = len(registry.robots.get_all())
            envs = len(registry.environments.get_all())
            algos = len(registry.algorithms.get_all())
            print(f"✓ Registry loaded successfully")
            print(f"  - Robots: {robots}")
            print(f"  - Environments: {envs}")
            print(f"  - Algorithms: {algos}")
            
            # Validate registry
            is_valid, errors = registry.validate()
            if is_valid:
                print(f"✓ Registry validation passed")
            else:
                print(f"✗ Registry validation failed: {len(errors)} errors")
                for error in errors[:3]:  # Show first 3 errors
                    print(f"    - {error}")
                if len(errors) > 3:
                    print(f"    ... and {len(errors) - 3} more errors")
        else:
            print(f"✗ Failed to load registry")
    except Exception as e:
        print(f"✗ Registry error: {e}")
    
    # Check 4: Core packages
    print("\n[4] Core Packages")
    print("-" * 40)
    core_packages = [
        'robot_lab_registry',
        'robot_lab_adapter',
        'robot_lab_bringup',
        'robot_lab_description',
        'gazebo_models'
    ]
    
    for pkg in core_packages:
        try:
            result = subprocess.run(['ros2', 'pkg', 'prefix', pkg],
                                   capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print(f"✓ {pkg}")
            else:
                print(f"✗ {pkg}: not found")
        except Exception:
            print(f"✗ {pkg}: check failed")
    
    # Check 5: Environment validation
    print("\n[5] Environment Setup")
    print("-" * 40)
    
    # Check for valid build
    if Path(args.config_dir or '.').parent.joinpath('build').exists():
        print(f"✓ Build directory found")
    else:
        print(f"⚠ Build directory not found (run colcon build)")
    
    if Path(args.config_dir or '.').parent.joinpath('install').exists():
        print(f"✓ Install directory found")
    else:
        print(f"⚠ Install directory not found (run colcon build)")
    
    # Check for simulation assets
    try:
        result = subprocess.run(['ros2', 'pkg', 'prefix', 'gazebo_models'],
                               capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            models_dir = Path(result.stdout.strip()) / 'share' / 'gazebo_models'
            if models_dir.exists():
                model_count = len(list(models_dir.glob('*.sdf'))) + len(list(models_dir.glob('*/model.sdf')))
                print(f"✓ Gazebo models found ({model_count} models)")
            else:
                print(f"⚠ Gazebo models directory not found")
    except Exception:
        print(f"⚠ Could not verify Gazebo models")
    
    print("\n[Summary]")
    print("-" * 40)
    print("Run 'robot-lab list robots' to see available robots")
    print("Run 'robot-lab list environments' to see available environments")
    print("Run 'robot-lab list algorithms' to see available algorithms")
    print("Run 'robot-lab launch --dry-run <composition>' to test a launch configuration")
    
    return 0


# ============================================================================
# CLI Setup
# ============================================================================

def create_parser() -> argparse.ArgumentParser:
    """Create the main argument parser."""
    parser = argparse.ArgumentParser(
        prog='robot-lab',
        description='Robot Lab Registry CLI - Manage and validate registry catalogs'
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # List command
    list_parser = subparsers.add_parser('list', help='List entities')
    list_parser.add_argument('entity_type', nargs='?', default='robots',
                              choices=['robots', 'environments', 'algorithms', 'scenarios', 'experiments'],
                              help='Type of entities to list')
    list_parser.add_argument('-c', '--config-dir', default=None,
                              help='Directory containing catalog files')
    list_parser.add_argument('--format', choices=['table', 'json', 'yaml'], default='table',
                              help='Output format')
    list_parser.add_argument('--status', help='Filter by status')
    # Robot filters
    list_parser.add_argument('--robot-class', help='Filter robots by class')
    # Environment filters
    list_parser.add_argument('--simulator', help='Filter environments by simulator')
    list_parser.add_argument('--dimension', help='Filter environments by dimension')
    # Algorithm filters
    list_parser.add_argument('--category', help='Filter algorithms by category')
    list_parser.add_argument('--family', help='Filter algorithms by family')
    # Experiment filters
    list_parser.add_argument('--robot-id', help='Filter experiments by robot ID')
    list_parser.add_argument('--environment-id', help='Filter experiments by environment ID')
    
    # Describe command
    describe_parser = subparsers.add_parser('describe', help='Describe an entity')
    describe_parser.add_argument('entity_type',
                                  choices=['robot', 'environment', 'algorithm', 'scenario', 'experiment'],
                                  help='Type of entity')
    describe_parser.add_argument('entity_id', help='ID of the entity')
    describe_parser.add_argument('-c', '--config-dir', default=None,
                                  help='Directory containing catalog files')
    describe_parser.add_argument('--format', choices=['text', 'json', 'yaml'], default='text',
                                  help='Output format')
    describe_parser.add_argument('-v', '--verbose', action='store_true',
                                  help='Show all fields')
    
    # Search command
    search_parser = subparsers.add_parser('search', help='Search entities')
    search_parser.add_argument('query', help='Text to search for')
    search_parser.add_argument('-t', '--entity-type', default=None,
                                choices=['robot', 'environment', 'algorithm', 'scenario', 'experiment'],
                                help='Filter by entity type')
    search_parser.add_argument('-c', '--config-dir', default=None,
                                help='Directory containing catalog files')
    search_parser.add_argument('--format', choices=['text', 'json', 'yaml'], default='text',
                                help='Output format')
    
    # Validate command
    validate_parser = subparsers.add_parser('validate', help='Validate registry')
    validate_parser.add_argument('-c', '--config-dir', required=True,
                                  help='Directory containing catalog files')
    validate_parser.add_argument('-t', '--entity-type', default=None,
                                  choices=['robot', 'environment', 'algorithm', 'scenario', 'experiment'],
                                  help='Validate specific entity type')
    validate_parser.add_argument('--cross-references', action='store_true',
                                  help='Validate cross-references only')
    
    # Check-composition command
    check_parser = subparsers.add_parser('check-composition', help='Check composition validity')
    check_parser.add_argument('-c', '--config-dir', required=True,
                                help='Directory containing catalog files')
    check_parser.add_argument('-f', '--composition-file', default=None,
                                help='File containing composition (JSON/YAML)')
    check_parser.add_argument('--composition', default=None,
                                help='Inline JSON composition')
    
    # Summary command
    summary_parser = subparsers.add_parser('summary', help='Show registry summary')
    summary_parser.add_argument('-c', '--config-dir', default=None,
                                  help='Directory containing catalog files')
    summary_parser.add_argument('--format', choices=['text', 'json', 'yaml'], default='text',
                                  help='Output format')
    
    # Schema command
    schema_parser = subparsers.add_parser('schema', help='Show schema')
    schema_parser.add_argument('-t', '--entity-type', default=None,
                                choices=['robot', 'environment', 'algorithm', 'scenario', 'experiment'],
                                help='Entity type schema to show')
    schema_parser.add_argument('--format', choices=['json', 'yaml'], default='yaml',
                                help='Output format')
    
    # Launch command
    launch_parser = subparsers.add_parser('launch', help='Launch a composition')
    launch_parser.add_argument('-c', '--config-dir', required=True,
                               help='Directory containing catalog files')
    launch_parser.add_argument('-f', '--composition-file', default=None,
                              help='File containing composition (JSON/YAML)')
    launch_parser.add_argument('--composition', default=None,
                              help='Inline JSON composition')
    launch_parser.add_argument('--dry-run', action='store_true', default=True,
                              help='Show launch config without executing (default)')
    launch_parser.add_argument('--no-dry-run', dest='dry_run', action='store_false',
                              help='Actually execute the launch (not yet implemented)')
    
    # Doctor command
    doctor_parser = subparsers.add_parser('doctor', help='Diagnose Robot Lab environment')
    doctor_parser.add_argument('-c', '--config-dir', default=None,
                               help='Directory containing catalog files')
    
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    """
    Main entry point for the CLI.
    
    Args:
        argv: Command-line arguments (defaults to sys.argv)
        
    Returns:
        Exit code (0 for success, non-zero for error)
    """
    if argv is None:
        argv = sys.argv[1:]  # Skip program name
    
    parser = create_parser()
    args = parser.parse_args(argv)
    
    if not args.command:
        parser.print_help()
        return 1
    
    # Dispatch command
    command_handlers = {
        'list': cmd_list,
        'describe': cmd_describe,
        'search': cmd_search,
        'validate': cmd_validate,
        'check-composition': cmd_check_composition,
        'summary': cmd_summary,
        'schema': cmd_schema,
        'launch': cmd_launch,
        'doctor': cmd_doctor
    }
    
    handler = command_handlers.get(args.command)
    if not handler:
        print(f"Unknown command: {args.command}")
        return 1
    
    return handler(args)


if __name__ == '__main__':
    sys.exit(main())
