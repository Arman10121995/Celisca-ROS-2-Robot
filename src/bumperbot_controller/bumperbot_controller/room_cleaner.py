#!/usr/bin/env python3
"""
Room Cleaner Module

This module contains the cleaning-specific logic extracted from room_vacuum_controller.py.
It can be used as a standalone cleaner or integrated with a mapper.

Usage:
    from room_cleaner import RoomCleaner
    cleaner = RoomCleaner(map_data, config)
    cleaner.plan_cleaning()
    # ... use cleaner methods
"""

import math
from collections import deque
from typing import Dict, List, Set, Tuple, Optional
from dataclasses import dataclass, field
from enum import Enum
import rclpy
from rclpy.node import Node


class CleanerState(Enum):
    """States for the cleaning process."""
    CLEAN_PLAN = "CLEAN_PLAN"
    CLEAN_APPROACH_LANE = "CLEAN_APPROACH_LANE"
    CLEAN_SWEEP_LANE = "CLEAN_SWEEP_LANE"
    CLEAN_DONE = "CLEAN_DONE"


@dataclass
class CleanerConfig:
    """Configuration for the room cleaner."""
    cleaning_grid_spacing: float = 0.40
    map_resolution: float = 0.15
    robot_obstacle_clearance: float = 0.30
    cleaning_goal_dist: float = 0.25
    lane_width: float = 0.40
    
    # Derived settings
    cleaning_step_cells: int = 1
    rejected_goal_radius_cells: int = 1
    
    def __post_init__(self):
        self.cleaning_step_cells = max(1, int(round(self.cleaning_grid_spacing / self.map_resolution)))
        self.rejected_goal_radius_cells = max(1, int(round(0.20 / self.map_resolution)))


class RoomCleaner:
    """
    Handles the cleaning logic for a room vacuum robot.
    
    This class manages:
    - Cleaning goal generation
    - Cleaning lane planning
    - Cleaning path following
    - Tracking of cleaned areas
    """
    
    def __init__(self, config: CleanerConfig = None):
        """
        Initialize the room cleaner.
        
        Args:
            config: CleanerConfig instance with cleaning parameters
        """
        self.config = config or CleanerConfig()
        
        # Cleaning state
        self.state = CleanerState.CLEAN_PLAN
        self.cleaning_goals: List[Tuple[int, int]] = []
        self.current_clean_goal_index: int = 0
        self.current_cleaning_lane_cells: List[Tuple[int, int]] = []
        self.current_lane_start_cell: Optional[Tuple[int, int]] = None
        self.current_lane_endpoint_cell: Optional[Tuple[int, int]] = None
        self.current_goal_cell: Optional[Tuple[int, int]] = None
        self.cleaned_cells: Set[Tuple[int, int]] = set()
        self.temporarily_skipped_cleaning_cells: Set[Tuple[int, int]] = set()
        self.rejected_cleaning_cells: Set[Tuple[int, int]] = set()
        
        # Map data (to be provided by mapper)
        self.free_counts: Dict[Tuple[int, int], int] = {}
        self.occupied_counts: Dict[Tuple[int, int], int] = {}
        self.temporary_obstacle_counts: Dict[Tuple[int, int], int] = {}
        self.map_version: int = 0
        self.last_cleaning_goal_version: int = -1
        
        # Callbacks for integration with mapper
        self.is_cell_occupied_callback = None
        self.is_cell_free_callback = None
        self.is_cell_blocked_callback = None
        self.plan_path_to_cell_callback = None
        self.reached_cell_callback = None
    
    def set_map_data(self, free_counts, occupied_counts, temporary_obstacle_counts=None):
        """Set the map data from the mapper."""
        self.free_counts = free_counts
        self.occupied_counts = occupied_counts
        if temporary_obstacle_counts:
            self.temporary_obstacle_counts = temporary_obstacle_counts
        self.map_version += 1
    
    def set_callbacks(self, is_occupied, is_free, is_blocked, plan_path, reached_cell):
        """Set callback functions for integration with mapper."""
        self.is_cell_occupied_callback = is_occupied
        self.is_cell_free_callback = is_free
        self.is_cell_blocked_callback = is_blocked
        self.plan_path_to_cell_callback = plan_path
        self.reached_cell_callback = reached_cell
    
    def is_cell_occupied(self, cell):
        """Check if a cell is occupied."""
        if self.is_cell_occupied_callback:
            return self.is_cell_occupied_callback(cell)
        return cell in self.occupied_counts and self.occupied_counts[cell] > 0
    
    def is_cell_free(self, cell):
        """Check if a cell is free."""
        if self.is_cell_free_callback:
            return self.is_cell_free_callback(cell)
        return cell in self.free_counts and self.free_counts[cell] > 0
    
    def is_cell_blocked(self, cell):
        """Check if a cell is blocked."""
        if self.is_cell_blocked_callback:
            return self.is_cell_blocked_callback(cell)
        return self.is_cell_occupied(cell)
    
    def plan_path_to_cell(self, goal):
        """Plan a path to a cell."""
        if self.plan_path_to_cell_callback:
            return self.plan_path_to_cell_callback(goal)
        return False
    
    def reached_cell(self, cell, tolerance):
        """Check if a cell has been reached."""
        if self.reached_cell_callback:
            return self.reached_cell_callback(cell, tolerance)
        return False
    
    def build_cleaning_goals(self):
        """
        Build a list of cleaning goals from free cells.
        
        Returns:
            List of (x, y) cell coordinates to clean
        """
        cleaning_goals = []
        added_cells = set()
        
        # Get all free cells
        free_cells = [cell for cell, count in self.free_counts.items() if count > 0]
        
        # Sort cells for systematic cleaning (left to right, bottom to top)
        free_cells.sort(key=lambda cell: (cell[1], cell[0]))
        
        # Add cells spaced by cleaning grid
        for cell in free_cells:
            cx, cy = cell
            # Check if this cell or nearby cells are already in goals
            nearby = False
            for gx, gy in cleaning_goals:
                if abs(cx - gx) < self.config.cleaning_step_cells and abs(cy - gy) < self.config.cleaning_step_cells:
                    nearby = True
                    break
            if not nearby:
                cleaning_goals.append(cell)
        
        return cleaning_goals
    
    def mark_history_cleaned(self):
        """Mark cells from history as cleaned (to be implemented with history)."""
        # This would be implemented with access to robot's history
        pass
    
    def is_cell_cleaned(self, cell):
        """Check if a cell has been cleaned."""
        return cell in self.cleaned_cells
    
    def mark_cleaned_area(self, center_cell, radius=1):
        """Mark an area around a cell as cleaned."""
        cx, cy = center_cell
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                cell = (cx + dx, cy + dy)
                if cell in self.cleaning_goals or cell in self.free_counts:
                    self.cleaned_cells.add(cell)
    
    def plan_cleaning(self):
        """Plan the cleaning process."""
        self.temporarily_skipped_cleaning_cells.clear()
        self.mark_history_cleaned()
        self.cleaning_goals = self.build_cleaning_goals()
        return self.plan_next_cleaning_lane("cleaning started")
    
    def refresh_cleaning_goals_if_needed(self, map_changed, reason):
        """Refresh cleaning goals if map has changed."""
        if not map_changed and self.last_cleaning_goal_version == self.map_version:
            return None
        return self.refresh_cleaning_goals(reason)
    
    def refresh_cleaning_goals(self, reason):
        """Refresh the list of cleaning goals."""
        old_remaining = set(self.cleaning_goals[self.current_clean_goal_index:])
        self.temporarily_skipped_cleaning_cells.clear()
        self.mark_history_cleaned()
        new_goals = self.build_cleaning_goals()
        new_goal_set = set(new_goals)
        added_count = len(new_goal_set - old_remaining)
        removed_count = len(old_remaining - new_goal_set)
        self.cleaning_goals = new_goals
        note = self.plan_next_cleaning_lane(reason)
        return f"cleaning_goals_refreshed_added_{added_count}_removed_{removed_count}; {note}"
    
    def plan_next_cleaning_lane(self, reason):
        """Plan the next cleaning lane."""
        self.clear_cleaning_lane_state()
        self.current_path = []
        self.current_path_direct = False
        self.current_clean_goal_index = 0
        self.last_cleaning_goal_version = self.map_version
        
        if not self.cleaning_goals:
            self.current_goal_cell = None
            self.state = CleanerState.CLEAN_DONE
            return "cleaning_no_goals"
        
        while self.current_clean_goal_index < len(self.cleaning_goals):
            start_cell = self.cleaning_goals[self.current_clean_goal_index]
            if (
                self.is_cell_cleaned(start_cell)
                or self.is_rejected_goal(start_cell)
                or start_cell in self.temporarily_skipped_cleaning_cells
            ):
                self.current_clean_goal_index += 1
                continue
            
            lane_cells = self.get_cleaning_lane_cells(self.current_clean_goal_index)
            if not lane_cells:
                self.current_clean_goal_index += 1
                continue
            
            self.current_cleaning_lane_cells = lane_cells
            self.current_lane_start_cell = lane_cells[0]
            self.current_lane_endpoint_cell = lane_cells[-1]
            self.current_goal_cell = self.current_lane_start_cell
            
            if self.current_cleaning_lane_start_reached():
                return self.start_cleaning_lane_sweep()
            
            if self.plan_path_to_cell(self.current_lane_start_cell):
                self.current_goal_cell = self.current_lane_start_cell
                self.state = CleanerState.CLEAN_APPROACH_LANE
                return f"cleaning_approach_lane_{self.current_lane_start_cell}_to_{self.current_lane_endpoint_cell}"
            
            self.skip_cleaning_lane_cells()
            self.current_clean_goal_index += len(lane_cells)
        
        self.clear_cleaning_lane_state()
        self.current_goal_cell = None
        self.current_path = []
        self.state = CleanerState.CLEAN_DONE
        return "cleaning_lanes_unreachable"
    
    def clear_cleaning_lane_state(self):
        """Clear the current cleaning lane state."""
        self.current_cleaning_lane_cells = []
        self.current_lane_start_cell = None
        self.current_lane_endpoint_cell = None
    
    def get_cleaning_lane_cells(self, start_index):
        """Get cells for a cleaning lane starting at index."""
        if start_index >= len(self.cleaning_goals):
            return []
        
        start_cell = self.cleaning_goals[start_index]
        lane_cells = [start_cell]
        max_gap_cells = max(self.config.cleaning_step_cells + 1, 
                          int(math.ceil(self.config.cleaning_step_cells * 1.5)))
        
        for cell in self.cleaning_goals[start_index + 1:]:
            previous = lane_cells[-1]
            if cell[1] != start_cell[1]:
                break
            if abs(cell[0] - previous[0]) > max_gap_cells:
                break
            if (
                self.is_cell_cleaned(cell)
                or self.is_rejected_goal(cell)
                or cell in self.temporarily_skipped_cleaning_cells
            ):
                break
            lane_cells.append(cell)
        
        return lane_cells
    
    def current_cleaning_lane_start_reached(self):
        """Check if the start of the current cleaning lane has been reached."""
        if self.current_lane_start_cell is None:
            return False
        return self.reached_cell(self.current_lane_start_cell, self.config.cleaning_goal_dist)
    
    def current_cleaning_lane_finished(self):
        """Check if the current cleaning lane is finished."""
        if self.current_lane_endpoint_cell is None:
            return False
        if self.reached_cell(self.current_lane_endpoint_cell, self.config.cleaning_goal_dist):
            return True
        return all(self.is_cell_cleaned(cell) for cell in self.current_cleaning_lane_cells)
    
    def start_cleaning_lane_sweep(self):
        """Start sweeping the current cleaning lane."""
        self.mark_cleaned_area(self.current_lane_start_cell)
        
        remaining_lane_cells = [
            cell for cell in self.current_cleaning_lane_cells
            if not self.is_cell_cleaned(cell) and not self.is_cell_blocked(cell)
        ]
        if not remaining_lane_cells:
            return self.finish_current_cleaning_lane()
        
        for endpoint in reversed(remaining_lane_cells):
            # For now, just mark the endpoint (path planning would be done by mapper)
            self.current_lane_endpoint_cell = endpoint
            self.current_goal_cell = endpoint
            self.state = CleanerState.CLEAN_SWEEP_LANE
            return f"cleaning_sweep_lane_to_{endpoint}"
        
        self.skip_cleaning_lane_cells()
        self.current_clean_goal_index += len(self.current_cleaning_lane_cells)
        return self.plan_next_cleaning_lane("skipped cleaning lane without clear straight sweep")
    
    def finish_current_cleaning_lane(self):
        """Finish the current cleaning lane."""
        # Mark current position as cleaned (if we have a position callback)
        self.current_clean_goal_index += len(self.current_cleaning_lane_cells)
        self.cleaning_goals = self.build_cleaning_goals()
        return self.plan_next_cleaning_lane("finished cleaning lane")
    
    def skip_cleaning_lane_cells(self):
        """Skip the cells in the current cleaning lane."""
        for cell in self.current_cleaning_lane_cells:
            self.temporarily_skipped_cleaning_cells.add(cell)
    
    def is_rejected_goal(self, cell):
        """Check if a cell is in the rejected goals set."""
        return cell in self.rejected_cleaning_cells
    
    def reject_goal_cell(self, goal, reason):
        """Reject a goal cell and add nearby cells to rejected set."""
        radius = self.config.rejected_goal_radius_cells
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                self.rejected_cleaning_cells.add((goal[0] + dx, goal[1] + dy))
    
    def remaining_cleaning_goals(self):
        """Count remaining cleaning goals."""
        return sum(1 for cell in self.cleaning_goals[self.current_clean_goal_index:] 
                   if not self.is_cell_cleaned(cell))


if __name__ == '__main__':
    print("Room Cleaner module - import to use")
