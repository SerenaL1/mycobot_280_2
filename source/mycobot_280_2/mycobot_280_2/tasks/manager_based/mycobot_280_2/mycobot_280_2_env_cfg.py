# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg  # ADDED RigidObjectCfg
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
#from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
#from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from . import mdp
from .mycobot_280_2 import MYCOBOT_280_CFG


##
# Scene definition
##


@configclass
class Mycobot2802SceneCfg(InteractiveSceneCfg):
    """Configuration for a myCobot 280 manipulation scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # robot
    robot: ArticulationCfg = MYCOBOT_280_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # Table
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.CuboidCfg(
            size=(0.8, 0.6, 0.7),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            mass_props=sim_utils.MassPropertiesCfg(mass=50.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.7, 0.6)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.4, 0.0, 0.35)),
    )

    # Tray for blocks
    tray = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Tray",
        spawn=sim_utils.CuboidCfg(
            size=(0.12, 0.12, 0.02),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.2),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.2, 0.2)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.35, 0.0, 0.71)),
    )

    # Kohs Block
    kohs_block = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/KohsBlock",
        spawn=sim_utils.CuboidCfg(
            size=(0.025, 0.025, 0.025),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.015),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.1, 0.1)),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.5,
                dynamic_friction=0.4,
                restitution=0.1,
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.35, 0.0, 0.7325),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )

    # Lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )

    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=600.0),
        init_state=AssetBaseCfg.InitialStateCfg(rot=(0.738, 0.477, 0.477, 0.0)),
    )


##
# MDP settings
##

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # Joint position control for the 6 DOF arm
    arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", 
                     "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"],
        scale=0.5,
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # Robot joint positions and velocities
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot")})
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("robot")})
        
        # End effector position
        ee_pos = ObsTerm(
            func=mdp.root_pos_w,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=["joint6_flange"])},
        )
        
        # Block position
        block_pos = ObsTerm(
            func=mdp.root_pos_w,
            params={"asset_cfg": SceneEntityCfg("kohs_block")},
        )

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # Observation groups
    policy: PolicyCfg = PolicyCfg()


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


##
# Environment configuration
##

@configclass
class Mycobot2802EnvCfg(ManagerBasedEnvCfg):  # CHANGED: No RL!
    """Configuration for the myCobot 280 manipulation environment."""

    # Scene settings
    scene: Mycobot2802SceneCfg = Mycobot2802SceneCfg(num_envs=1, env_spacing=2.0)  # CHANGED: 1 env for teleoperation
    
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    
    # NO rewards or terminations - removed!

    def __post_init__(self) -> None:
        """Post initialization."""
        # General settings
        self.decimation = 2
        # No episode_length_s needed - removed!
        
        # Viewer settings
        self.viewer.eye = (1.5, 1.5, 1.0)
        self.viewer.lookat = (0.4, 0.0, 0.5)
        
        # Simulation settings
        self.sim.dt = 1.0 / 120.0
        self.sim.render_interval = self.decimation
        self.sim.physics_material = sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        )