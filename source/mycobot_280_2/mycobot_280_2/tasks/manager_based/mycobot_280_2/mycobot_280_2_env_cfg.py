# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from . import mdp
import os

##
# Scene configuration - RELATIVE POSITIONING
##
TRAY_USD_PATH = os.path.join(os.path.dirname(__file__), "assets", "tray.usd")

# Tray configuration - CHANGE THESE THREE VALUES TO MOVE EVERYTHING!
TRAY_CENTER_X = 0.22
TRAY_CENTER_Y = 0.0
TRAY_CENTER_Z = 0.005  # Relative to table surface (z=0.7 is table top)
TRAY_BOTTOM_THICKNESS = 0.3 * 0.0254  # 0.3 inches from your Fusion model
TRAY_SIZE = 0.10  # 10cm x 10cm

# Block configuration
BLOCK_SIZE = 1.0 * 0.0254  # 1 inch = 0.0254m
BLOCK_GAP = 0.3 * 0.0254  # 0.3 inch gap between blocks
BLOCK_SPACING = BLOCK_SIZE + BLOCK_GAP  # 1.3 inches center-to-center
BLOCK_MASS = 0.02  # 20g

# Calculate block z-position: sitting ON tray bottom
# Just: tray_z + tray_bottom_thickness + half_block_height
BLOCK_Z = TRAY_CENTER_Z + TRAY_BOTTOM_THICKNESS + BLOCK_SIZE / 2

# Calculate 3x3 grid positions centered on tray
# Grid offsets: -1, 0, +1 spacing from center
BLOCK_POSITIONS = [
    # Row 1 (back row, y = -BLOCK_SPACING)
    (TRAY_CENTER_X - BLOCK_SPACING, TRAY_CENTER_Y - BLOCK_SPACING, BLOCK_Z),  # block_0
    (TRAY_CENTER_X,                  TRAY_CENTER_Y - BLOCK_SPACING, BLOCK_Z),  # block_1
    (TRAY_CENTER_X + BLOCK_SPACING, TRAY_CENTER_Y - BLOCK_SPACING, BLOCK_Z),  # block_2
    # Row 2 (middle row, y = 0)
    (TRAY_CENTER_X - BLOCK_SPACING, TRAY_CENTER_Y,                  BLOCK_Z),  # block_3
    (TRAY_CENTER_X,                  TRAY_CENTER_Y,                  BLOCK_Z),  # block_4
    (TRAY_CENTER_X + BLOCK_SPACING, TRAY_CENTER_Y,                  BLOCK_Z),  # block_5
    # Row 3 (front row, y = +BLOCK_SPACING)
    (TRAY_CENTER_X - BLOCK_SPACING, TRAY_CENTER_Y + BLOCK_SPACING, BLOCK_Z),  # block_6
    (TRAY_CENTER_X,                  TRAY_CENTER_Y + BLOCK_SPACING, BLOCK_Z),  # block_7
    (TRAY_CENTER_X + BLOCK_SPACING, TRAY_CENTER_Y + BLOCK_SPACING, BLOCK_Z),  # block_8
]

##
# Helper function to create blocks
##
def create_block_cfg(block_id: int, position: tuple) -> RigidObjectCfg:
    """Create a block with consistent physics settings."""
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Block_{block_id}",
        spawn=sim_utils.CuboidCfg(
            size=(BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=False,
                disable_gravity=False,
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=8,
                linear_damping=0.5,
                angular_damping=0.5,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=BLOCK_MASS),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.8,
                dynamic_friction=0.7,
                restitution=0.0,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=position),
    )


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

    # Tray - Use your custom USD file 
    # Make sure you have tray.usd in the assets folder
    tray = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Tray",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(TRAY_CENTER_X, TRAY_CENTER_Y, TRAY_CENTER_Z)
        ),
        spawn=UsdFileCfg(
            usd_path=TRAY_USD_PATH,
            scale=(0.0254, 0.0254, 0.0254),  # 4 inches converted to meters
        ),
    )

    # All 9 blocks using helper function - positions calculated automatically
    block_0 = create_block_cfg(0, BLOCK_POSITIONS[0])
    block_1 = create_block_cfg(1, BLOCK_POSITIONS[1])
    block_2 = create_block_cfg(2, BLOCK_POSITIONS[2])
    block_3 = create_block_cfg(3, BLOCK_POSITIONS[3])
    block_4 = create_block_cfg(4, BLOCK_POSITIONS[4])
    block_5 = create_block_cfg(5, BLOCK_POSITIONS[5])
    block_6 = create_block_cfg(6, BLOCK_POSITIONS[6])
    block_7 = create_block_cfg(7, BLOCK_POSITIONS[7])
    block_8 = create_block_cfg(8, BLOCK_POSITIONS[8])

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

    arm_action: mdp.JointPositionActionCfg = MISSING


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    
    # SUCCESS: Tray pushed forward by 10cm (from x=0.25 to x=0.35)
    # Demo is saved when tray reaches target position for 10 consecutive steps
    success = DoneTerm(
        func=mdp.tray_reached_goal,
        params={
            "asset_cfg": SceneEntityCfg("tray"),
            "target_x": 0.3,  # 10cm forward from starting position (0.25)
            "threshold": 0.05,  # Within 5cm counts as success
        }
    )


@configclass
class Mycobot2802EnvCfg(ManagerBasedRLEnvCfg):
    """Base configuration for the MyCobot 280 manipulation environment."""

    scene: Mycobot2802SceneCfg = Mycobot2802SceneCfg(num_envs=1, env_spacing=2.0, replicate_physics=False)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    commands = None
    rewards = None
    events = None
    curriculum = None

    def __post_init__(self):
        """Post initialization."""
        self.decimation = 2
        self.episode_length_s = 20.0

        # Viewer looks at tray
        self.viewer.eye = (1.5, 1.5, 1.0)
        self.viewer.lookat = (TRAY_CENTER_X, TRAY_CENTER_Y, TRAY_CENTER_Z)

        self.sim.dt = 1.0 / 120.0
        self.sim.render_interval = self.decimation

        # Enhanced physics for stability
        self.sim.physx.bounce_threshold_velocity = 0.05
        self.sim.physx.solver_position_iteration_count = 16
        self.sim.physx.solver_velocity_iteration_count = 8
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625