# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.devices.device_base import DeviceBase, DevicesCfg
from isaaclab.devices.keyboard import Se3KeyboardCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass

from . import mycobot_280_joint_pos_env_cfg  # Same folder


@configclass
class MycobotIKRelEnvCfg(mycobot_280_joint_pos_env_cfg.MycobotEnvCfg):
    """MyCobot 280 environment with IK control for teleoperation."""
    
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set actions for IK control (replaces joint position control)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
                         "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"],
            body_name="joint6_flange",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="dls",
            ),
            scale=0.5,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(
                pos=[0.0, 0.0, 0.0]
            ),
        )

        # Configure teleop devices (keyboard control)
        self.teleop_devices = DevicesCfg(
            devices={
                "keyboard": Se3KeyboardCfg(
                    pos_sensitivity=0.05,
                    rot_sensitivity=0.05,
                    sim_device=self.sim.device,
                ),
            }
        )