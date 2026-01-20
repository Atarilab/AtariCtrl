from atarictrl.config import cfg_registry
from atarictrl.controller.ctrl_cfgs import (
    JoystickCtrlCfg,  # noqa: F401
    KeyboardCtrlCfg,  # noqa: F401
    UnitreeCtrlCfg,  # noqa: F401
)
from atarictrl.pipeline.pipeline_cfgs import (
    DebugCfg,  # noqa: F401
)
from atarictrl.tools.tool_cfgs import ViconOdometryCfg

from .env.g1_mujuco_env_cfg import G1MujocoEnvCfg, G1MujocoBoxEnvCfg  # noqa: F401
from .env.g1_real_env_cfg import G1RealEnvCfg, G1UnitreeCfg  # noqa: F401
from .pipeline.g1_locomimic_pipeline_cfg import G1RlLocoMimicPipelineCfg  # noqa: F401
from .policy.g1_amo_policy_cfg import G1AmoPolicyCfg  # noqa: F401
from .policy.g1_beyondmimic_policy_cfg import G1BeyondMimicPolicyCfg  # noqa: F401
from .policy.g1_beyondmimic_policy_cfg import G1BeyondMimicObjectPolicyCfg  # noqa: F401



# ================= LocoMotion + MotionMimic Policy Switch Configs ================= #


@cfg_registry.register
class g1_locomimic_beyondmimic(G1RlLocoMimicPipelineCfg):
    """
    Smooth switch between multiple BeyondMimic policies, Sim2Sim.
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg()
    ctrl: list[KeyboardCtrlCfg | JoystickCtrlCfg] = [
        KeyboardCtrlCfg(
            triggers={
                "i": "[SIM_REBORN]",
                "o": "[SHUTDOWN]",
                "]": "[POLICY_LOCO]",
                "[": "[POLICY_MIMIC]",
                ";": "[POLICY_SWITCH],NEXT",
                "'": "[POLICY_SWITCH],LAST",
            }
        ),
        JoystickCtrlCfg(
            combination_init_buttons=[],
            triggers={
                "A": "[SHUTDOWN]",
                "Back": "[POLICY_LOCO]",
                "Start": "[POLICY_MIMIC]",
                "RB": "[POLICY_SWITCH],NEXT",
                "LB": "[POLICY_SWITCH],LAST",
            },
        ),
    ]

    loco_policy: G1AmoPolicyCfg = G1AmoPolicyCfg()
    # loco_policy: G1AsapLocoPolicyCfg = G1AsapLocoPolicyCfg()
    # loco_policy: G1UnitreePolicyCfg = G1UnitreePolicyCfg()
    # loco_policy: G1UnitreeWoGaitPolicyCfg = G1UnitreeWoGaitPolicyCfg()
    """Any LocoMotion policy, as init"""

    mimic_policies: list[G1BeyondMimicPolicyCfg] = [
        G1BeyondMimicPolicyCfg(policy_name="Dance_wose", without_state_estimator=True),
        G1BeyondMimicPolicyCfg(
        policy_name="victor",
        without_state_estimator=True,
        use_modelmeta_config=True,  # use robot dof config from modelmeta
        use_motion_from_model=True,  # use motion from onnx model
        max_timestep=250,
    ),
        G1BeyondMimicPolicyCfg(policy_name="Violin", without_state_estimator=False, max_timestep=500),
        G1BeyondMimicPolicyCfg(policy_name="Waltz", without_state_estimator=False, max_timestep=850),
    ]


# ================= LocoMimic Policy Switch Sim2real Configs ================= #


@cfg_registry.register
class g1_locomimic_beyondmimic_real(g1_locomimic_beyondmimic):
    """
    Locomotion + Beyondmimic, Sim2Real.
    Warning: Make sure the policy is stable for real robot before using it.
    """

    env: G1RealEnvCfg = G1RealEnvCfg(
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
        UnitreeCtrlCfg(
            combination_init_buttons=[],
            triggers={
                "A": "[SHUTDOWN]",
                "Select": "[POLICY_LOCO]",
                "Start": "[POLICY_MIMIC]",
                "R1": "[POLICY_SWITCH],NEXT",
                "L1": "[POLICY_SWITCH],LAST",
            },
        ),
    ]

    do_safety_check: bool = True  # enable safety check for real robot


@cfg_registry.register
class g1_locomimic_beyondmimic_object(g1_locomimic_beyondmimic):
    """
    Smooth switch between multiple BeyondMimic policies, Sim2Sim.
    """

    env: G1MujocoBoxEnvCfg = G1MujocoBoxEnvCfg()
    # env: G1MujocoCylinderEnvCfg = G1MujocoCylinderEnvCfg()
    mimic_policies: list[G1BeyondMimicObjectPolicyCfg] = [
        G1BeyondMimicObjectPolicyCfg(
            # policy_name="sub3_largebox_005_original2",
            # policy_name="sub8_largebox_031_original2",
            policy_name="ik/3",
            # policy_name="sub3_largebox_005_original",
            # policy_name="policy_sbmpc_pickupup_smallbox",
            without_state_estimator=True,
            use_modelmeta_config=True,  # use robot dof config from modelmeta
            use_motion_from_model=True,  # use motion from onnx model
            action_beta=1.0,
            max_timestep=-1,
            override_robot_anchor_pos=False,
            # start_timestep=60,
        )
    ]
    debug: DebugCfg = DebugCfg(log_obs=True)

@cfg_registry.register
class g1_locomimic_beyondmimic_object_vicon(g1_locomimic_beyondmimic_object):
    env: G1RealEnvCfg = G1RealEnvCfg(
        env_type="UnitreeCppEnv",  # For unitree_cpp, check README for more details
        unitree=G1UnitreeCfg(
            net_if="enx3c4937046061",  # note: change to your network interface
        ),
        odometry_type="VICON",
        vicon_cfg=ViconOdometryCfg(
            zmq_address="tcp://localhost:5555",
            topic_filter="",
            segments=[("G1", "G1"), ("yellowbox", "yellowbox")],
            robot_segment=("G1", "G1"),
            pos_offset=[0.0, 0.0, 0.0],
            zero_align=False,
        ),
    )

    debug: DebugCfg = DebugCfg(log_obs=True)