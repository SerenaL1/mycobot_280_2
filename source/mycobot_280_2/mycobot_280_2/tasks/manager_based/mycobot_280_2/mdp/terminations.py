# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

"""Custom termination functions for MyCobot 280 tasks."""

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg


def tray_reached_goal(
    env: ManagerBasedRLEnv, 
    asset_cfg: SceneEntityCfg,
    target_x: float = 0.3,
    threshold: float = 0.05
) -> torch.Tensor:
    """Check if tray has reached the target x-position.
    
    This termination function checks if the tray has been pushed forward
    to the target position. Used for demo recording success condition.
    
    Args:
        env: The environment.
        asset_cfg: Scene entity configuration for the tray.
        target_x: Target x-position for the tray (default: 0.35 = 10cm forward from 0.25).
        threshold: Distance threshold to consider goal reached (default: 0.05 = 5cm).
    
    Returns:
        Boolean tensor indicating whether tray reached the goal for each environment.
    """
    # Get the tray asset
    tray = env.scene[asset_cfg.name]
    
    # Get current tray position (world frame)
    tray_pos_w = tray.data.root_pos_w[:, :3]  # Shape: (num_envs, 3)
    
    # Check if tray x-position is within threshold of target
    current_x = tray_pos_w[0, 0].item()  # Get x-position for first environment
    distance_to_goal = torch.abs(tray_pos_w[:, 0] - target_x)
    goal_reached = distance_to_goal < threshold
    
    # Print current position and progress (every 30 steps to avoid spam)
    if env.episode_length_buf[0] % 30 == 0:
        print(f"[TRAY] x={current_x:.4f} | Target={target_x:.4f} | Distance={distance_to_goal[0].item():.4f}m | Goal={'✓ REACHED!' if goal_reached[0] else '✗'}")
    
    return goal_reached