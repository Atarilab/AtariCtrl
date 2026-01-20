from atarictrl.config import cfg_registry
from atarictrl.controller.ctrl_cfgs import (
    JoystickCtrlCfg,  # noqa: F401
    KeyboardCtrlCfg,  # noqa: F401
    UnitreeCtrlCfg,  # noqa: F401
)
from atarictrl.pipeline.pipeline_cfgs import (
    RlPipelineCfg,  # noqa: F401
)
from atarictrl.tools.debug_log import DebugCfg
from atarictrl.tools.tool_cfgs import ViconOdometryCfg

from .env.g1_dummy_env_cfg import G1DummyEnvCfg  # noqa: F401
from .env.g1_mujuco_env_cfg import G1_23MujocoEnvCfg, G1MujocoBoxEnvCfg
from .env.g1_real_env_cfg import G1RealEnvCfg  # noqa: F401
from .policy.g1_beyondmimic_policy_cfg import  G1BeyondMimicObjectPolicyCfg  # noqa: F401
from .policy.g1_unitree_policy_cfg import G1UnitreePolicyCfg  # noqa: F401

from .g1_cfg import g1
from .env.g1_real_env_cfg import G1RealEnvCfg, G1UnitreeCfg  # noqa: F401


# ======================== Custom Configs ======================== #
"""
Add your custom config here.
"""


@cfg_registry.register
class g1_dev(RlPipelineCfg):
    robot: str = "g1"
    env: G1_23MujocoEnvCfg = G1_23MujocoEnvCfg()

    ctrl: list[KeyboardCtrlCfg] = [
        KeyboardCtrlCfg(),
    ]

    policy: G1UnitreePolicyCfg = G1UnitreePolicyCfg()
    

@cfg_registry.register
class g1_reach_real(RlPipelineCfg):
    """
    ReachPolicy configuration for G1 robot.
    Upper body control with 3D hand position commands via joystick.
    Base is fixed, only upper body actions are output.
    """
    robot: str = "g1"
    env: G1RealEnvCfg = G1RealEnvCfg(
        odometry_type="DUMMY"
    )
    ctrl: list[JoystickCtrlCfg | KeyboardCtrlCfg] = [
        JoystickCtrlCfg(),
    ]
    do_safety_check: bool = True
    

@cfg_registry.register
class g1_dummy_vicon_test(RlPipelineCfg):
    """
    Dummy environment configuration for testing Vicon odometry.
    This config uses DummyEnv with VICON odometry type to test ViconOdometry
    without requiring a real robot or simulation.
    """
    robot: str = "g1"
    env: G1DummyEnvCfg = G1DummyEnvCfg(
        odometry_type="VICON",
        vicon_cfg=ViconOdometryCfg(
            zmq_address="tcp://localhost:5555",
            topic_filter="",
            segments=[("G1", "G1")],
            robot_segment=("G1", "G1"),
            pos_offset=[0.0, 0.0, 0.0],
            zero_align=True,
        ),
    )

    ctrl: list[KeyboardCtrlCfg] = [
        KeyboardCtrlCfg(),
    ]

    policy: G1UnitreePolicyCfg = G1UnitreePolicyCfg()
    
@cfg_registry.register
class g1_real_vicon(g1):
    """
    Unitree G1 robot, Unitree Policy, Sim2Real.
    To extend the sim2sim config to sim2real, just need to change the env to real env.
    """

    # env: G1DummyEnvCfg = G1DummyEnvCfg()
    env: G1RealEnvCfg = G1RealEnvCfg(
        # env_type="UnitreeEnv",  # For unitree_sdk2py
        env_type="UnitreeCppEnv",  # For unitree_cpp, check README for more details
        unitree=G1UnitreeCfg(
            net_if="enx3c4937046061",  # note: change to your network interface
        ),
        odometry_type="VICON",
        vicon_cfg=ViconOdometryCfg(
            zmq_address="tcp://localhost:5555",
            topic_filter="",
            segments=[("G1", "G1")],
            robot_segment=("G1", "G1"),
            pos_offset=[0.0, 0.0, 0.0],
            zero_align=True,
        ),
    )

    ctrl: list[UnitreeCtrlCfg] = [
        UnitreeCtrlCfg(),
    ]

    do_safety_check: bool = True  # enable safety check for real robot


@cfg_registry.register
class g1_beyondmimic_object(RlPipelineCfg):
    """
    BeyondMimic Policy, support both with and without state estimator.
    """

    robot: str = "g1"
    # env: G1MujocoCylinderEnvCfg = G1MujocoCylinderEnvCfg()
    env: G1MujocoBoxEnvCfg = G1MujocoBoxEnvCfg()
    ctrl: list[KeyboardCtrlCfg | JoystickCtrlCfg] = [
        KeyboardCtrlCfg(),
        JoystickCtrlCfg(),
    ]

    policy: G1BeyondMimicObjectPolicyCfg = G1BeyondMimicObjectPolicyCfg(
        # policy_name="sub3_largebox_001_original",
        # policy_name="sub8_largebox_031_original2",
        policy_name="sub8_largebox_028_original",
        # policy_name="sub8_largebox_031_original",
        # policy_name="sub7_largebox_001_original",
        # policy_name="sub16_largebox_001_original",
        # policy_name="cylinder_sbmpc",
        # policy_name="policy_sbmpc_pickupup_smallbox",
        without_state_estimator=True,
        use_modelmeta_config=True,  # use robot dof config from modelmeta
        use_motion_from_model=True,  # use motion from onnx model
        action_beta=1.0,
        max_timestep=-1,
        override_robot_anchor_pos=False,
        # start_timestep=60,
    )
    debug: DebugCfg = DebugCfg(log_obs=True)
