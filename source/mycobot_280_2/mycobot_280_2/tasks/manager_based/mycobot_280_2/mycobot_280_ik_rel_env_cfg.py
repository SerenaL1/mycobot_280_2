from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.utils import configclass

from .mycobot_280_2_env_cfg import Mycobot2802EnvCfg, Mycobot2802SceneCfg

@configclass
class Mycobot2802SceneIKCfg(Mycobot2802SceneCfg):
    """Scene with frame transformer for IK."""
    
    # Add end-effector frame tracker
    ee_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link",
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                name="end_effector",
                prim_path="{ENV_REGEX_NS}/Robot/joint6_flange",  # Your end effector
            ),
        ],
    )


@configclass
class ActionsCfgIK:
    """Action specifications using IK control."""
    
    arm_action = DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
                     "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"],
        body_name="joint6_flange",  # End effector link name
        controller=DifferentialIKControllerCfg(
            command_type="pose",
            use_relative_mode=True,  # CRITICAL: Must be True for teleop
            ik_method="dls",
        ),
        scale=0.5,
    )


@configclass
class Mycobot2802IKRelEnvCfg(Mycobot2802EnvCfg):
    """MyCobot environment with IK control for teleoperation."""
    
    def __post_init__(self):
        super().__post_init__()
        
        # Replace scene with IK-enabled scene
        self.scene = Mycobot2802SceneIKCfg(num_envs=1, env_spacing=2.0)
        
        # Replace actions with IK control
        self.actions = ActionsCfgIK()