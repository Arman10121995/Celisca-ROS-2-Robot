"""
Catalog management for Robot Lab Registry.

Provides loading, merging, querying, and validation of entity catalogs.
"""

import os
import yaml
import json
from typing import Dict, List, Any, Optional, Union
from pathlib import Path

from .schemas import (
    validate_entity,
    get_schema,
    SCHEMAS
)


class Catalog:
    """Base catalog class for managing collections of entities."""
    
    def __init__(self, entity_type: str, schema_version: int = 1):
        """
        Initialize a catalog.
        
        Args:
            entity_type: Type of entities in this catalog
            schema_version: Version of the schema to use
        """
        if entity_type not in SCHEMAS:
            raise ValueError(f"Unknown entity type: {entity_type}. Valid: {list(SCHEMAS.keys())}")
        
        self.entity_type = entity_type
        self.schema_version = schema_version
        self.entities: Dict[str, Dict[str, Any]] = {}
        self._source_files: List[str] = []
        self._errors: List[str] = []
        
    def load_file(self, file_path: Union[str, Path]) -> bool:
        """
        Load entities from a YAML or JSON file.
        
        Args:
            file_path: Path to the file to load
            
        Returns:
            True if loaded successfully, False otherwise
        """
        file_path = Path(file_path)
        if not file_path.exists():
            self._errors.append(f"File not found: {file_path}")
            return False
        
        try:
            with open(file_path, 'r') as f:
                if file_path.suffix in ['.yaml', '.yml']:
                    data = yaml.safe_load(f)
                elif file_path.suffix == '.json':
                    data = json.load(f)
                else:
                    self._errors.append(f"Unsupported file type: {file_path.suffix}")
                    return False
            
            if data is None:
                return True  # Empty file
            
            # Handle both single entity and list of entities
            if isinstance(data, dict):
                # Could be a single entity or a dict of entities
                if 'id' in data:
                    # Single entity
                    entities = [data]
                else:
                    # Dict of entities keyed by ID
                    entities = list(data.values())
            elif isinstance(data, list):
                entities = data
            else:
                self._errors.append(f"Invalid data format in {file_path}")
                return False
            
            loaded_count = 0
            for entity in entities:
                if not isinstance(entity, dict):
                    continue
                if 'id' not in entity:
                    self._errors.append(f"Entity missing 'id' field in {file_path}")
                    continue
                
                # Validate entity
                is_valid, errors = validate_entity(self.entity_type, entity)
                if not is_valid:
                    for error in errors:
                        self._errors.append(f"{file_path}: {error}")
                    continue
                
                entity_id = entity['id']
                if entity_id in self.entities:
                    self._errors.append(f"Duplicate ID '{entity_id}' in {file_path}")
                    continue
                
                self.entities[entity_id] = entity
                loaded_count += 1
            
            if loaded_count > 0:
                self._source_files.append(str(file_path))
            
            return True
            
        except yaml.YAMLError as e:
            self._errors.append(f"YAML error in {file_path}: {e}")
            return False
        except json.JSONDecodeError as e:
            self._errors.append(f"JSON error in {file_path}: {e}")
            return False
        except Exception as e:
            self._errors.append(f"Error loading {file_path}: {e}")
            return False
    
    def load_directory(self, dir_path: Union[str, Path]) -> int:
        """
        Load all valid files from a directory.
        
        Args:
            dir_path: Path to directory containing catalog files
            
        Returns:
            Number of entities loaded
        """
        dir_path = Path(dir_path)
        if not dir_path.is_dir():
            return 0
        
        count = 0
        for ext in ['*.yaml', '*.yml', '*.json']:
            for file_path in dir_path.glob(ext):
                if self.load_file(file_path):
                    count += len(self.entities) - count
        
        return count
    
    def get(self, entity_id: str) -> Optional[Dict[str, Any]]:
        """Get an entity by ID."""
        return self.entities.get(entity_id)
    
    def get_all(self) -> Dict[str, Dict[str, Any]]:
        """Get all entities."""
        return self.entities.copy()
    
    def get_ids(self) -> List[str]:
        """Get all entity IDs."""
        return list(self.entities.keys())
    
    def filter(self, **kwargs) -> Dict[str, Dict[str, Any]]:
        """
        Filter entities by field values.
        
        Args:
            **kwargs: Field name/value pairs to match
            
        Returns:
            Filtered dictionary of entities
        """
        result = {}
        for entity_id, entity in self.entities.items():
            match = True
            for key, value in kwargs.items():
                if key not in entity:
                    match = False
                    break
                if isinstance(value, list):
                    if entity[key] not in value:
                        match = False
                        break
                elif entity[key] != value:
                    match = False
                    break
            if match:
                result[entity_id] = entity
        return result
    
    def search(self, query: str) -> Dict[str, Dict[str, Any]]:
        """
        Search entities by text in name, description, or tags.
        
        Args:
            query: Text to search for
            
        Returns:
            Matching entities
        """
        query = query.lower()
        result = {}
        
        for entity_id, entity in self.entities.items():
            # Search in name
            if query in entity.get('name', '').lower():
                result[entity_id] = entity
                continue
            # Search in description
            if query in entity.get('description', '').lower():
                result[entity_id] = entity
                continue
            # Search in tags
            if query in [t.lower() for t in entity.get('tags', [])]:
                result[entity_id] = entity
                continue
            # Search in ID
            if query in entity_id.lower():
                result[entity_id] = entity
        
        return result
    
    def validate_all(self) -> tuple:
        """
        Validate all entities in the catalog.
        
        Returns:
            Tuple of (is_valid: bool, errors: List[str])
        """
        errors = []
        for entity_id, entity in self.entities.items():
            is_valid, entity_errors = validate_entity(self.entity_type, entity)
            if not is_valid:
                for error in entity_errors:
                    errors.append(f"{entity_id}: {error}")
        
        errors.extend(self._errors)
        return len(errors) == 0, errors
    
    def get_source_files(self) -> List[str]:
        """Get list of loaded source files."""
        return self._source_files.copy()
    
    def get_errors(self) -> List[str]:
        """Get list of loading/validation errors."""
        return self._errors.copy()
    
    def count(self) -> int:
        """Get number of entities."""
        return len(self.entities)


# ============================================================================
# Entity-specific Catalog Classes
# ============================================================================

class RobotCatalog(Catalog):
    """Catalog for robot entities."""
    
    def __init__(self, schema_version: int = 1):
        super().__init__('robot', schema_version)
    
    def get_by_class(self, robot_class: str) -> Dict[str, Dict[str, Any]]:
        """Get robots by class."""
        return self.filter(robot_class=robot_class)
    
    def get_by_capability(self, capability: str) -> Dict[str, Dict[str, Any]]:
        """Get robots that have a specific capability."""
        result = {}
        for entity_id, entity in self.entities.items():
            if capability in entity.get('capabilities', []):
                result[entity_id] = entity
        return result


class EnvironmentCatalog(Catalog):
    """Catalog for environment entities."""
    
    def __init__(self, schema_version: int = 1):
        super().__init__('environment', schema_version)
    
    def get_by_simulator(self, simulator: str) -> Dict[str, Dict[str, Any]]:
        """Get environments by simulator."""
        return self.filter(simulator=simulator)
    
    def get_by_dimension(self, dimension: str) -> Dict[str, Dict[str, Any]]:
        """Get environments by dimension."""
        return self.filter(dimension=dimension)


class AlgorithmCatalog(Catalog):
    """Catalog for algorithm entities."""
    
    def __init__(self, schema_version: int = 1):
        super().__init__('algorithm', schema_version)
    
    def get_by_category(self, category: str) -> Dict[str, Dict[str, Any]]:
        """Get algorithms by category."""
        return self.filter(category=category)
    
    def get_by_family(self, family: str) -> Dict[str, Dict[str, Any]]:
        """Get algorithms by family."""
        result = {}
        for entity_id, entity in self.entities.items():
            if entity.get('family', '') == family:
                result[entity_id] = entity
        return result


class ScenarioCatalog(Catalog):
    """Catalog for scenario entities."""
    
    def __init__(self, schema_version: int = 1):
        super().__init__('scenario', schema_version)
    
    def get_by_task_type(self, task_type: str) -> Dict[str, Dict[str, Any]]:
        """Get scenarios by task type."""
        return self.filter(task_type=task_type)


class ExperimentCatalog(Catalog):
    """Catalog for experiment entities."""
    
    def __init__(self, schema_version: int = 1):
        super().__init__('experiment', schema_version)
    
    def get_by_robot(self, robot_id: str) -> Dict[str, Dict[str, Any]]:
        """Get experiments by robot ID."""
        return self.filter(robot_id=robot_id)
    
    def get_by_environment(self, environment_id: str) -> Dict[str, Dict[str, Any]]:
        """Get experiments by environment ID."""
        return self.filter(environment_id=environment_id)


# ============================================================================
# Registry Class (combines all catalogs)
# ============================================================================

class Registry:
    """Complete registry combining all catalog types."""
    
    def __init__(self, config_dir: Optional[Union[str, Path]] = None):
        """
        Initialize the registry.
        
        Args:
            config_dir: Directory containing catalog configuration files
        """
        self.robots = RobotCatalog()
        self.environments = EnvironmentCatalog()
        self.algorithms = AlgorithmCatalog()
        self.scenarios = ScenarioCatalog()
        self.experiments = ExperimentCatalog()
        
        self.config_dir = Path(config_dir) if config_dir else None
        self._loaded = False
    
    def load(self, config_dir: Optional[Union[str, Path]] = None) -> bool:
        """
        Load all catalogs from configuration directory.
        
        Args:
            config_dir: Directory to load from (overrides constructor)
            
        Returns:
            True if all catalogs loaded successfully
        """
        if config_dir:
            self.config_dir = Path(config_dir)
        
        if not self.config_dir:
            return False
        
        # Load each catalog type
        robot_catalog_file = self.config_dir / "robots.yaml"
        if robot_catalog_file.exists():
            self.robots.load_file(robot_catalog_file)
        else:
            self.robots.load_directory(self.config_dir / "robots")
        
        env_catalog_file = self.config_dir / "environments.yaml"
        if env_catalog_file.exists():
            self.environments.load_file(env_catalog_file)
        else:
            self.environments.load_directory(self.config_dir / "environments")
        
        algo_catalog_file = self.config_dir / "algorithms.yaml"
        if algo_catalog_file.exists():
            self.algorithms.load_file(algo_catalog_file)
        else:
            algo_catalog_dir = self.config_dir / "algorithms"
            if algo_catalog_dir.exists():
                self.algorithms.load_directory(algo_catalog_dir)
        
        scenario_catalog_file = self.config_dir / "scenarios.yaml"
        if scenario_catalog_file.exists():
            self.scenarios.load_file(scenario_catalog_file)
        else:
            self.scenarios.load_directory(self.config_dir / "scenarios")
        
        experiment_catalog_file = self.config_dir / "experiments.yaml"
        if experiment_catalog_file.exists():
            self.experiments.load_file(experiment_catalog_file)
        else:
            self.experiments.load_directory(self.config_dir / "experiments")
        
        self._loaded = True
        return True
    
    def validate(self) -> tuple:
        """
        Validate all loaded catalogs.
        
        Returns:
            Tuple of (is_valid: bool, all_errors: List[str])
        """
        all_errors = []
        
        for catalog_name, catalog in [
            ('robots', self.robots),
            ('environments', self.environments),
            ('algorithms', self.algorithms),
            ('scenarios', self.scenarios),
            ('experiments', self.experiments)
        ]:
            is_valid, errors = catalog.validate_all()
            if not is_valid:
                for error in errors:
                    all_errors.append(f"{catalog_name}: {error}")
        
        return len(all_errors) == 0, all_errors
    
    def get_summary(self) -> Dict[str, Any]:
        """Get a summary of the registry contents."""
        return {
            "robots": self.robots.count(),
            "environments": self.environments.count(),
            "algorithms": self.algorithms.count(),
            "scenarios": self.scenarios.count(),
            "experiments": self.experiments.count(),
            "loaded": self._loaded
        }
    
    def get_all_errors(self) -> List[str]:
        """Get all errors from all catalogs."""
        all_errors = []
        for catalog in [self.robots, self.environments, self.algorithms, 
                       self.scenarios, self.experiments]:
            all_errors.extend(catalog.get_errors())
        return all_errors
