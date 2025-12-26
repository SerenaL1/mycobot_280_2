# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

import isaaclab.sim as sim_utils
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.utils import configclass

from .. import mdp  # Go up one level to mycobot_280_2/
from ..mycobot_280_2_env_cfg import Mycobot2802EnvCfg  # Import from parent
from ..mycobot_280_2 import MYCOBOT_280_CFG  # Import from parent

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip


@configclass
class EventCfg:
    """Configuration for events - DETERMINISTIC resets for clinical reliability."""

    # Reset robot to home position (deterministic)
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "position_range": (1.0, 1.0),  # Exactly 1.0 = deterministic
            "velocity_range": (0.0, 0.0),
        },
    )

    # Reset tray to exact position (deterministic - NO randomization)
    reset_tray_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("tray"),
            "pose_range": {},  # No change - uses init_state position
            "velocity_range": {},
        },
    )

    # Reset all 9 blocks to EXACT positions (deterministic - NO randomization)
    reset_block_0 = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("block_0"),
            "pose_range": {},  # No change - uses init_state position
            "velocity_range": {},
        },
    )

    reset_block_1 = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("block_1"),
            "pose_range": {},
            "velocity_range": {},
        },
    )

    reset_block_2 = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("block_2"),
            "pose_range": {},
            "velocity_range": {},
        },
    )

    reset_block_3 = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("block_3"),
            "pose_range": {},
            "velocity_range": {},
        },
    )

    reset_block_4 = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("block_4"),
            "pose_range": {},
            "velocity_range": {},
        },
    )

    reset_block_5 = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("block_5"),
            "pose_range": {},
            "velocity_range": {},
        },
    )

    reset_block_6 = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("block_6"),
            "pose_range": {},
            "velocity_range": {},
        },
    )

    reset_block_7 = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("block_7"),
            "pose_range": {},
            "velocity_range": {},
        },
    )

    reset_block_8 = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("block_8"),
            "pose_range": {},
            "velocity_range": {},
        },
    )


@configclass
class MycobotEnvCfg(Mycobot2802EnvCfg):
    """MyCobot 280 environment with joint position control."""

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set events
        self.events = EventCfg()

        # Set MyCobot as robot
        self.scene.robot = MYCOBOT_280_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for MyCobot (joint position control)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
                         "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"],
            scale=0.5,
            use_default_offset=True
        )

        # Frame transformer for end-effector tracking
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/g_base",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/joint6_flange",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.0],
                    ),
                ),
            ],
        )