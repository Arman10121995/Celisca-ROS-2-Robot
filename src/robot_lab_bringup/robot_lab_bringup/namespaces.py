"""
Namespace and frame-prefix management for multi-robot experiments.

This module provides P2.5 functionality: namespace and frame-prefix contracts
to prevent collisions in parallel and multi-robot experiments.

The namespace system ensures that:
1. Each robot instance has a unique namespace
2. All topics, services, actions, and parameters are prefixed with the namespace
3. TF frames are prefixed consistently
4. Nodes are placed in the correct namespace
5. Cross-robot communication can be explicitly configured
"""

import re
from typing import Dict, List, Optional, Set, Tuple
from dataclasses import dataclass, field


@dataclass
class NamespaceConfig:
    """Configuration for a single namespace."""
    
    # The namespace identifier (e.g., "robot1", "robot2", "sim")
    name: str
    
    # The frame prefix for TF (e.g., "robot1/")
    frame_prefix: str = ""
    
    # Description of what this namespace is for
    description: str = ""
    
    # Priority (lower runs first, useful for base namespaces)
    priority: int = 0
    
    def __post_init__(self):
        """Validate and normalize the namespace config."""
        if not self.name:
            raise ValueError("Namespace name cannot be empty")
        
        # Normalize: ensure no leading/trailing slashes
        self.name = self.name.strip('/')
        
        # If frame_prefix is empty, default to name + '/'
        if not self.frame_prefix:
            self.frame_prefix = f"{self.name}/"
        else:
            self.frame_prefix = self.frame_prefix.strip('/') + '/'
    
    def get_topic(self, topic: str) -> str:
        """Apply namespace to a topic name."""
        if not topic:
            return topic
        
        # Absolute topics (starting with /) get namespace prepended
        if topic.startswith('/'):
            # Remove leading slash, add namespace, then add slash back
            return f"/{self.name}{topic}"
        
        # Relative topics get namespace as prefix
        return f"{self.name}/{topic}"
    
    def get_service(self, service: str) -> str:
        """Apply namespace to a service name."""
        return self.get_topic(service)
    
    def get_action(self, action: str) -> str:
        """Apply namespace to an action name."""
        return self.get_topic(action)
    
    def get_frame(self, frame: str) -> str:
        """Apply frame prefix to a TF frame."""
        if not frame:
            return frame
        
        # Absolute frames (starting with /) get frame_prefix prepended
        if frame.startswith('/'):
            # Remove leading slash, add prefix
            return f"/{self.frame_prefix}{frame[1:]}"
        
        # Relative frames get frame_prefix as prefix
        return f"{self.frame_prefix}{frame}"
    
    def get_parameter(self, param: str) -> str:
        """Apply namespace to a parameter name."""
        return self.get_topic(param)


class NamespaceManager:
    """
    Manage namespaces for multi-robot experiments.
    
    This class:
    - Generates unique namespaces for each robot instance
    - Tracks which namespaces are in use
    - Provides utilities for applying namespaces to topics/frames
    - Validates that namespaces don't conflict
    """
    
    # Default namespaces for common use cases
    DEFAULT_NAMESPACES = {
        'robot0': NamespaceConfig(name='robot0', frame_prefix='robot0/', description='First robot'),
        'robot1': NamespaceConfig(name='robot1', frame_prefix='robot1/', description='Second robot'),
        'robot2': NamespaceConfig(name='robot2', frame_prefix='robot2/', description='Third robot'),
        'sim': NamespaceConfig(name='sim', frame_prefix='', description='Simulation namespace'),
        'global': NamespaceConfig(name='global', frame_prefix='', description='Global namespace'),
    }
    
    def __init__(self):
        self._namespaces: Dict[str, NamespaceConfig] = {}
        self._used_names: Set[str] = set()
        self._reserved_names: Set[str] = set(['global', 'sim'])
        
        # Initialize with defaults
        for name, config in self.DEFAULT_NAMESPACES.items():
            self._namespaces[name] = config
            self._reserved_names.add(name)
    
    def create_namespace(self, name: Optional[str] = None, frame_prefix: str = "",
                        description: str = "") -> NamespaceConfig:
        """
        Create a new namespace.
        
        Args:
            name: Optional namespace name. If None, generates a unique name.
            frame_prefix: Optional frame prefix. If empty, uses name + '/'.
            description: Description of the namespace.
        
        Returns:
            NamespaceConfig for the new namespace.
        
        Raises:
            ValueError: If the namespace name is already in use.
        """
        if name is None:
            # Generate a unique name
            counter = 0
            while True:
                name = f"robot{counter}"
                if name not in self._used_names:
                    break
                counter += 1
        
        if name in self._used_names:
            raise ValueError(f"Namespace '{name}' is already in use")
        
        config = NamespaceConfig(
            name=name,
            frame_prefix=frame_prefix or f"{name}/",
            description=description
        )
        
        self._namespaces[name] = config
        self._used_names.add(name)
        
        return config
    
    def get_namespace(self, name: str) -> Optional[NamespaceConfig]:
        """Get a namespace configuration by name."""
        return self._namespaces.get(name)
    
    def get_or_create_namespace(self, name: Optional[str] = None, frame_prefix: str = "",
                                description: str = "") -> NamespaceConfig:
        """
        Get an existing namespace or create a new one.
        
        Args:
            name: Namespace name. If None, generates a unique name.
            frame_prefix: Frame prefix for new namespace.
            description: Description for new namespace.
        
        Returns:
            NamespaceConfig for the namespace.
        """
        if name and name in self._namespaces:
            return self._namespaces[name]
        
        return self.create_namespace(name, frame_prefix, description)
    
    def release_namespace(self, name: str) -> bool:
        """
        Release a namespace so it can be reused.
        
        Args:
            name: Name of the namespace to release.
        
        Returns:
            True if the namespace was released, False if it didn't exist.
        """
        if name in self._used_names:
            self._used_names.remove(name)
            return True
        return False
    
    def list_namespaces(self) -> List[str]:
        """List all known namespace names."""
        return list(self._namespaces.keys())
    
    def list_used_namespaces(self) -> List[str]:
        """List all currently used namespace names."""
        return list(self._used_names)
    
    def validate_namespace(self, name: str) -> Tuple[bool, str]:
        """
        Validate that a namespace name is valid.
        
        Args:
            name: Namespace name to validate.
        
        Returns:
            Tuple of (is_valid, error_message).
        """
        if not name:
            return False, "Namespace name cannot be empty"
        
        if not re.match(r'^[a-zA-Z][a-zA-Z0-9_]*$', name):
            return False, f"Invalid namespace name: {name}. Must start with letter, contain only alphanumeric and underscore."
        
        if name in self._reserved_names and name not in self._namespaces:
            return False, f"Namespace '{name}' is reserved"
        
        return True, ""
    
    def apply_to_topic(self, topic: str, namespace: str) -> str:
        """Apply a namespace to a topic."""
        ns_config = self.get_namespace(namespace)
        if ns_config:
            return ns_config.get_topic(topic)
        return topic
    
    def apply_to_frame(self, frame: str, namespace: str) -> str:
        """Apply a namespace's frame prefix to a frame."""
        ns_config = self.get_namespace(namespace)
        if ns_config:
            return ns_config.get_frame(frame)
        return frame
    
    def apply_to_parameter(self, param: str, namespace: str) -> str:
        """Apply a namespace to a parameter name."""
        ns_config = self.get_namespace(namespace)
        if ns_config:
            return ns_config.get_parameter(param)
        return param


def apply_namespace_to_dict(data: Dict, namespace: str, 
                           manager: Optional[NamespaceManager] = None) -> Dict:
    """
    Apply namespace to all relevant keys in a dictionary.
    
    This recursively applies the namespace to:
    - Topic names (keys containing 'topic', 'pub', 'sub', 'cmd', 'scan', etc.)
    - Service names
    - Action names
    - Parameter names
    - Frame names (keys containing 'frame', 'tf', 'base_link', etc.)
    
    Args:
        data: Dictionary to process.
        namespace: Namespace to apply.
        manager: Optional NamespaceManager. If None, creates a simple config.
    
    Returns:
        New dictionary with namespaced values.
    """
    if manager is None:
        # Create a simple namespace config
        ns_config = NamespaceConfig(name=namespace)
    else:
        ns_config = manager.get_namespace(namespace)
        if ns_config is None:
            ns_config = NamespaceConfig(name=namespace)
    
    result = {}
    
    for key, value in data.items():
        # Check if this key is a topic-related parameter
        # Be precise: only apply if the key IS a topic/sub/pub or contains specific topic types
        key_lower = key.lower()
        
        # Check for topic-related keys (exact match or ends with _topic)
        if key_lower in ['topic', 'pub_topic', 'sub_topic', 'cmd_vel', 'scan', 'odom', 'imu', 
                        'laser', 'camera', 'depth', 'pointcloud', 'map'] or \
           key_lower.endswith('_topic'):
            if isinstance(value, str):
                result[key] = ns_config.get_topic(value)
            elif isinstance(value, list):
                result[key] = [ns_config.get_topic(v) if isinstance(v, str) else v for v in value]
            else:
                result[key] = value
        
        # Check if this key is a topic list
        elif key_lower in ['topics', 'pub_topics', 'sub_topics']:
            if isinstance(value, list):
                result[key] = [ns_config.get_topic(v) if isinstance(v, str) else v for v in value]
            else:
                result[key] = value
        
        # Check if this key is a frame-related parameter
        elif key_lower in ['frame', 'tf_frame', 'base_link', 'odom_frame', 'map_frame', 
                          'frame_id', 'child_frame_id', 'parent_frame_id'] or \
             key_lower.endswith('_frame'):
            if isinstance(value, str):
                result[key] = ns_config.get_frame(value)
            elif isinstance(value, list):
                result[key] = [ns_config.get_frame(v) if isinstance(v, str) else v for v in value]
            else:
                result[key] = value
        
        # Check if this key is a service/action parameter
        elif key_lower in ['service', 'action', 'srv', 'client', 'server'] or \
             key_lower.endswith('_service') or key_lower.endswith('_action'):
            if isinstance(value, str):
                result[key] = ns_config.get_service(value)
            else:
                result[key] = value
        
        # Check if this key is a parameter name (only if it's exactly 'name' or contains 'parameter')
        elif key_lower == 'name' or 'parameter' in key_lower:
            if isinstance(value, str):
                result[key] = ns_config.get_parameter(value)
            else:
                result[key] = value
        
        # Recurse into nested dictionaries
        elif isinstance(value, dict):
            result[key] = apply_namespace_to_dict(value, namespace, manager)
        
        # Default: copy as-is
        else:
            result[key] = value
    
    return result


def get_default_namespace_for_robot(robot_id: str) -> str:
    """
    Get the default namespace for a robot ID.
    
    This follows the convention of using the robot_id as the namespace.
    
    Args:
        robot_id: The robot ID.
    
    Returns:
        Default namespace string.
    """
    # Sanitize robot_id to be a valid namespace
    # Replace invalid characters with underscores
    namespace = re.sub(r'[^a-zA-Z0-9_]', '_', robot_id)
    
    # Ensure it starts with a letter
    if namespace and not namespace[0].isalpha():
        namespace = f"robot_{namespace}"
    
    return namespace


def get_default_frame_prefix(robot_id: str) -> str:
    """
    Get the default frame prefix for a robot ID.
    
    This follows ROS conventions where the frame prefix is the same as the namespace.
    
    Args:
        robot_id: The robot ID.
    
    Returns:
        Default frame prefix string (with trailing slash).
    """
    namespace = get_default_namespace_for_robot(robot_id)
    return f"{namespace}/"
