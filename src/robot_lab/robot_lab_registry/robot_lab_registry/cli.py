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
        if args.robot_class:
            kwargs['robot_class'] = args.robot_class
    elif entity_type == 'environments':
        if args.simulator:
            kwargs['simulator'] = args.simulator
        if args.dimension:
            kwargs['dimension'] = args.dimension
    elif entity_type == 'algorithms':
        if args.category:
            kwargs['category'] = args.category
        if args.family:
            kwargs['family'] = args.family
        if args.robot_class:
            kwargs['robot_class'] = args.robot_class
    elif entity_type == 'scenarios':
        if args.task_type:
            kwargs['task_type'] = args.task_type
    elif entity_type == 'experiments':
        if args.robot_id:
            kwargs['robot_id'] = args.robot_id
        if args.environment_id:
            kwargs['environment_id'] = args.environment_id
    
    if args.status:
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
        'schema': cmd_schema
    }
    
    handler = command_handlers.get(args.command)
    if not handler:
        print(f"Unknown command: {args.command}")
        return 1
    
    return handler(args)


if __name__ == '__main__':
    sys.exit(main())
