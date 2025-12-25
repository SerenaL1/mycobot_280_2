# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from . import mdp

##
# Scene definition
##


@configclass
class Mycobot2802SceneCfg(InteractiveSceneCfg):
    """Configuration for the MyCobot 280 scene with robot and objects."""

    # robots: will be populated by child env cfg
    robot: ArticulationCfg = MISSING
    # end-effector sensor: will be populated by child env cfg
    ee_frame: FrameTransformerCfg = MISSING

    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -1.05]),
        spawn=GroundPlaneCfg(),
    )

    # Table
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, 0], rot=[0.707, 0, 0, 0.707]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    )

    # Tray - hollow tray with walls to hold blocks (10x10cm, 2cm tall)
    # Positioned CLOSE to robot arm so it can push it forward
    # Robot base is at ~(0, 0, 0.7), so tray at (0.25, 0.0, 0.72) is within reach
    tray = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Tray",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.25, 0.0, 0.72)),
        spawn=sim_utils.MultiShapeCfg(
            shapes=[
                # Base (bottom of tray) - 10x10cm x 0.5cm thick
                sim_utils.CuboidCfg(
                    size=(0.10, 0.10, 0.005),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
                    physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.6),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                ),
                # Front wall (along Y axis, at -X) - towards robot
                sim_utils.CuboidCfg(
                    size=(0.005, 0.10, 0.02),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
                    physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.6),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                    offset=sim_utils.OffsetCfg(pos=(-0.0475, 0.0, 0.0125)),
                ),
                # Back wall (along Y axis, at +X) - away from robot
                sim_utils.CuboidCfg(
                    size=(0.005, 0.10, 0.02),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
                    physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.6),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                    offset=sim_utils.OffsetCfg(pos=(0.0475, 0.0, 0.0125)),
                ),
                # Left wall (along X axis, at -Y)
                sim_utils.CuboidCfg(
                    size=(0.09, 0.005, 0.02),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
                    physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.6),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                    offset=sim_utils.OffsetCfg(pos=(0.0, -0.0475, 0.0125)),
                ),
                # Right wall (along X axis, at +Y)
                sim_utils.CuboidCfg(
                    size=(0.09, 0.005, 0.02),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
                    physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.6),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                    offset=sim_utils.OffsetCfg(pos=(0.0, 0.0475, 0.0125)),
                ),
            ],
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.5),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
    )

    # Kohs blocks - 9 red 2.5cm cubes arranged in 3x3 grid INSIDE the tray
    # Tray center is at (0.25, 0.0, 0.72)
    # Blocks sit on tray base at z = 0.72 + 0.005/2 + 0.025/2 = 0.735
    # Blocks are spaced 2.5cm apart (0.025m) in a 3x3 grid
    block_0 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Block_0",
        spawn=sim_utils.CuboidCfg(
            size=(0.025, 0.025, 0.025),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.22, -0.025, 0.735)),
    )
    
    block_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Block_1",
        spawn=sim_utils.CuboidCfg(
            size=(0.025, 0.025, 0.025),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.25, -0.025, 0.735)),
    )
    
    block_2 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Block_2",
        spawn=sim_utils.CuboidCfg(
            size=(0.025, 0.025, 0.025),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.28, -0.025, 0.735)),
    )
    
    block_3 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Block_3",
        spawn=sim_utils.CuboidCfg(
            size=(0.025, 0.025, 0.025),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.22, 0.0, 0.735)),
    )
    
    block_4 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Block_4",
        spawn=sim_utils.CuboidCfg(
            size=(0.025, 0.025, 0.025),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.25, 0.0, 0.735)),
    )
    
    block_5 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Block_5",
        spawn=sim_utils.CuboidCfg(
            size=(0.025, 0.025, 0.025),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.28, 0.0, 0.735)),
    )
    
    block_6 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Block_6",
        spawn=sim_utils.CuboidCfg(
            size=(0.025, 0.025, 0.025),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.22, 0.025, 0.735)),
    )
    
    block_7 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Block_7",
        spawn=sim_utils.CuboidCfg(
            size=(0.025, 0.025, 0.025),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.25, 0.025, 0.735)),
    )
    
    block_8 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Block_8",
        spawn=sim_utils.CuboidCfg(
            size=(0.025, 0.025, 0.025),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.28, 0.025, 0.735)),
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


##
# MDP settings
##


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # Will be set by child env cfg
    arm_action: mdp.JointPositionActionCfg = MISSING


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # Robot joint state
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)

        # Last action
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # Observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class Mycobot2802EnvCfg(ManagerBasedRLEnvCfg):
    """Base configuration for the MyCobot 280 manipulation environment."""

    # Scene settings
    scene: Mycobot2802SceneCfg = Mycobot2802SceneCfg(num_envs=1, env_spacing=2.0, replicate_physics=False)

    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    # No MDP managers for base config
    commands = None
    rewards = None
    events = None
    curriculum = None

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 20.0

        # viewer settings
        self.viewer.eye = (1.5, 1.5, 1.0)
        self.viewer.lookat = (0.4, 0.0, 0.5)

        # simulation settings
        self.sim.dt = 1.0 / 120.0
        self.sim.render_interval = self.decimation

        # physics settings
        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625