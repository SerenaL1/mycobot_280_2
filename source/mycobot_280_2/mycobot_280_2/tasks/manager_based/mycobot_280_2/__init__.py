# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

##
# Register Gym environments
##

gym.register(
    id="Template-Mycobot-280-IK-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "mycobot_280_2.tasks.manager_based.mycobot_280_2.config.mycobot_280_ik_rel_env_cfg:MycobotIKRelEnvCfg",
    },
)

gym.register(
    id="Template-Mycobot-280-Joint-v0",
    entry_point="isaaclab.envs:ManagerBasedEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "mycobot_280_2.tasks.manager_based.mycobot_280_2.config.mycobot_280_joint_pos_env_cfg:MycobotEnvCfg",
    },
)