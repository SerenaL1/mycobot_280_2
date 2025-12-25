# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.utils import configclass

from .. import mdp  # ← Changed: Go up one level to mycobot_280_2/
from ..mycobot_280_2_env_cfg import Mycobot2802EnvCfg  # ← Changed: Import from parent
from ..mycobot_280_2 import MYCOBOT_280_CFG  # ← Changed: Import from parent

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip


@configclass
class EventCfg:
    """Configuration for events."""

    # Reset robot to home position
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "position_range": (0.9, 1.1),
            "velocity_range": (0.0, 0.0),
        },
    )

    # Reset block position (with small randomization)
    reset_block_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("kohs_block"),
            "pose_range": {
                "x": (0.33, 0.37),
                "y": (-0.02, 0.02),
                "z": (0.7325, 0.7325),
            },
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

        # Rigid body properties for the Kohs block
        block_properties = RigidBodyPropertiesCfg(
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
            max_angular_velocity=1000.0,
            max_linear_velocity=1000.0,
            max_depenetration_velocity=5.0,
            disable_gravity=False,
        )

        # Set the Kohs block
        self.scene.kohs_block = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/KohsBlock",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=(0.35, 0.0, 0.7325),
                rot=(1.0, 0.0, 0.0, 0.0)
            ),
            spawn=sim_utils.CuboidCfg(
                size=(0.025, 0.025, 0.025),
                rigid_props=block_properties,
                mass_props=sim_utils.MassPropertiesCfg(mass=0.015),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.1, 0.1)),
                physics_material=sim_utils.RigidBodyMaterialCfg(
                    static_friction=0.5,
                    dynamic_friction=0.4,
                    restitution=0.1,
                ),
            ),
        )

        # Frame transformer for end-effector tracking
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
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