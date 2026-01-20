from typing import Literal

from atarictrl.environment.env_cfgs import RobotEnvCfg
from atarictrl.tools.tool_cfgs import ViconOdometryCfg, ZedOdometryCfg  # noqa: F401

# from atarictrl.tools.tool_cfgs import ForwardKinematicCfg
from .g1_env_cfg import G1EnvCfg


class G1DummyEnvCfg(G1EnvCfg, RobotEnvCfg):
    env_type: str = RobotEnvCfg.model_fields["env_type"].default
    # ====== ENV CONFIGURATION ======
    odometry_type: Literal["NONE", "DUMMY", "ZED", "VICON"] = "DUMMY"
    # zed_cfg: ZedOdometryCfg | None = ZedOdometryCfg(
    #     server_ip="127.0.0.1",
    #     pos_offset=[0.0, 0.0, 0.8],
    #     zero_align=True,
    # )
    # vicon_cfg: ViconOdometryCfg | None = ViconOdometryCfg(
    #     zmq_address="tcp://localhost:5555",
    #     topic_filter="",
    #     segments=[("G1", "G1")],
    #     robot_segment=("G1", "G1"),
    #     zero_align=True,
    # )

    # forward_kinematic: ForwardKinematicCfg | None = None
