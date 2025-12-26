# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

"""Custom termination functions for MyCobot 280 tasks."""

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg


def tray_reached_goal(
    env: ManagerBasedRLEnv, 
    asset_cfg: SceneEntityCfg,
    target_x: float = 0.32,  # Target CENTER position (not pivot)
    threshold: float = 0.05,
    tray_pivot_offset_x: float = 0.05  # Half of tray size (10cm/2 = 5cm)
) -> torch.Tensor:
    """Check if tray CENTER has reached the target x-position.
    
    This termination function checks if the tray CENTER (not pivot/spawn point) 
    has been pushed forward to the target position. 
    
    Args:
        env: The environment.
        asset_cfg: Scene entity configuration for the tray.
        target_x: Target x-position for the TRAY CENTER (default: 0.32 = 10cm forward from 0.22).
        threshold: Distance threshold to consider goal reached (default: 0.05 = 5cm).
        tray_pivot_offset_x: Offset from pivot to center (half tray size, default: 0.05m).
    
    Returns:
        Boolean tensor indicating whether tray center reached the goal for each environment.
    """
    # Get the tray asset
    tray = env.scene[asset_cfg.name]
    
    # Get current tray PIVOT position (world frame)
    tray_pivot_pos_w = tray.data.root_pos_w[:, :3]  # Shape: (num_envs, 3)
    
    # Calculate tray CENTER position (pivot + offset)
    tray_center_x = tray_pivot_pos_w[:, 0] + tray_pivot_offset_x
    
    # Check if tray CENTER x-position is within threshold of target
    current_center_x = tray_center_x[0].item()  # Get center x-position for first environment
    current_pivot_x = tray_pivot_pos_w[0, 0].item()  # Get pivot x-position
    distance_to_goal = torch.abs(tray_center_x - target_x)
    goal_reached = distance_to_goal < threshold
    
    # Print current position and progress (every 30 steps to avoid spam)
    if env.episode_length_buf[0] % 30 == 0:
        print(f"[TRAY] Pivot x={current_pivot_x:.4f} | Center x={current_center_x:.4f} | Target={target_x:.4f} | Distance={distance_to_goal[0].item():.4f}m | Goal={'✓ REACHED!' if goal_reached[0] else '✗'}")
    
    return goal_reached