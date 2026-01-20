from typing import Literal

from pydantic import model_validator

from atarictrl.config import Config
from atarictrl.tools.tool_cfgs import DoFConfig, ForwardKinematicCfg, ViconOdometryCfg, ZedOdometryCfg


class EnvCfg(Config):
    env_type: str  # name of the environment class
    is_sim: bool = False

    urdf: str | None = None
    xml: str
    body_names: list[str] | None = None

    dof: DoFConfig

    forward_kinematic: ForwardKinematicCfg | None = None
    update_with_fk: bool = False
    """Whether to update info from fk"""
    torso_name: str = "torso_link"
    """Name of the torso link, used in fk info extraction"""

    born_place_align: bool = True
    """Whether to align the born place to zero position and heading"""


class MujocoEnvCfg(EnvCfg):
    env_type: str = "MujocoEnv"
    is_sim: bool = True
    # ====== ENV CONFIGURATION ======
    sim_duration: float = 60.0
    sim_dt: float = 0.001
    sim_decimation: int = 20

    visualize_extras: bool = True  # TODO: remove
    
    fix_base: bool = False
    """Whether to fix the robot base position and orientation"""
    fixed_base_pos: list[float] | None = None
    """Fixed base position [x, y, z]. If None, uses initial position when fix_base is enabled"""
    fixed_base_quat: list[float] | None = None
    """Fixed base orientation quaternion [x, y, z, w]. If None, uses initial orientation when fix_base is enabled"""


class MujocoBoxEnvCfg(MujocoEnvCfg):
    """Configuration for MuJoCo environment with box state tracking."""
    env_type: str = "MujocoBoxEnv"
    
    box_body_name: str = "box"
    """Name of the box body in the XML"""
    box_joint_name: str = "box_free_joint"
    """Name of the box free joint in the XML"""


class RobotEnvCfg(EnvCfg):
    env_type: str = "DummyEnv"
    is_sim: bool = False
    # ====== ENV CONFIGURATION ======
    act: bool = True

    odometry_type: Literal["NONE", "DUMMY", "ZED", "VICON"] = "NONE"
    zed_cfg: ZedOdometryCfg | None = None
    """ZED odometry config, if odometry_type is "ZED", this must be set"""
    vicon_cfg: ViconOdometryCfg | None = None
    """Vicon odometry config, if odometry_type is "VICON", this must be set"""

    @model_validator(mode="after")
    def check_odometry_config(self):
        if self.odometry_type == "ZED" and self.zed_cfg is None:
            raise ValueError("zed_cfg must be set if odometry_type is 'ZED'")
        if self.odometry_type == "VICON" and self.vicon_cfg is None:
            raise ValueError("vicon_cfg must be set if odometry_type is 'VICON'")
        return self


class UnitreeEnvCfg(RobotEnvCfg):
    """
    Configuration for Unitree Robot environment.
    """

    class UnitreeCfg(Config):
        """Unitree SDK configuration"""

        net_if: str = "enx3c4937046061"
        """network interface to communicate with the robot"""

        robot: Literal["g1"]
        msg_type: Literal["hg", "go"]
        control_mode: str = "position"
        hand_type: Literal["Dex-3", "Inspire", "NONE"] = "NONE"

        lowcmd_topic: str = "rt/lowcmd"
        lowstate_topic: str = "rt/lowstate"

        enable_odometry: bool = False
        sport_state_topic: str = "rt/odommodestate"

        control_dt: float = 0.02
        """control command dt"""

    env_type: str = "UnitreeEnv"  # For unitree_sdk2py
    # env_type: str = "UnitreeCppEnv" # For unitree_cpp
    """UnitreeEnv for unitree_sdk2py, UnitreeCppEnv for unitree_cpp, check README for more details"""

    unitree: UnitreeCfg

    odometry_type: Literal["NONE", "DUMMY", "UNITREE", "ZED", "VICON"] = "DUMMY"  # pyright: ignore[reportIncompatibleVariableOverride]

    joint2motor_idx: list[int] | None = None
    """Mapping from env dof to motor index, None for direct mapping"""
    weak_motor: list[int] = []

    hand_retarget: None = None  # TODO

    @model_validator(mode="after")
    def check_joint2motor_idx(self):
        if self.joint2motor_idx is not None and len(self.joint2motor_idx) != self.dof.num_dofs:
            raise ValueError("joint2motor_idx length must match dof.num_dofs")
        return self
