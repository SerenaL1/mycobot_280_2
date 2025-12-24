from pathlib import Path
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

TEMPLATE_ASSETS_DATA_DIR = Path(__file__).resolve().parent

##
# Configuration
##

MYCOBOT_280_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{TEMPLATE_ASSETS_DATA_DIR}/assets/mycobot_280_3.usd",  # ← Fixed filename
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
            # ← Fixed joint names to match URDF
            "joint2_to_joint1": 0.0,
            "joint3_to_joint2": -1.57,
            "joint4_to_joint3": 1.57,
            "joint5_to_joint4": 0.0,
            "joint6_to_joint5": 0.0,
            "joint6output_to_joint6": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        # Total mass: 0.85 kg
        # Joint 1-2: heaviest (base motors) - 0.25kg + 0.15kg = 0.4kg
        # Joint 3-4: middle weight - 0.15kg + 0.12kg = 0.27kg
        # Joint 5-6: lightest (wrist/gripper) - 0.08kg + 0.06kg = 0.14kg
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint.*"],  # ← This will match all joints
            effort_limit_sim=1.5,
            velocity_limit_sim=3.0,
            stiffness={
                "joint2_to_joint1": 150.0,      # Base rotation - moves all mass
                "joint3_to_joint2": 120.0,      # Shoulder lift - moves arm
                "joint4_to_joint3": 100.0,      # Elbow - moves forearm
                "joint5_to_joint4": 70.0,       # Wrist pitch - less mass
                "joint6_to_joint5": 50.0,       # Wrist roll - minimal mass
                "joint6output_to_joint6": 40.0, # End effector rotation
            },
            damping={
                "joint2_to_joint1": 60.0,
                "joint3_to_joint2": 50.0,
                "joint4_to_joint3": 40.0,
                "joint5_to_joint4": 30.0,
                "joint6_to_joint5": 20.0,
                "joint6output_to_joint6": 15.0,
            },
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of myCobot 280 robot arm."""