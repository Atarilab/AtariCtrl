from atarictrl.config import ASSETS_DIR
from atarictrl.environment.env_cfgs import MujocoEnvCfg, MujocoBoxEnvCfg
from atarictrl.tools.tool_cfgs import DoFConfig
from .g1_env_cfg import G1_12EnvCfg, G1_23EnvCfg, G1EnvCfg, G1_29DoF


class G1MujocoEnvCfg(G1EnvCfg, MujocoEnvCfg):
    env_type: str = MujocoEnvCfg.model_fields["env_type"].default
    is_sim: bool = MujocoEnvCfg.model_fields["is_sim"].default
    # ====== ENV CONFIGURATION ======

    update_with_fk: bool = True


class G1_23MujocoEnvCfg(G1_23EnvCfg, MujocoEnvCfg):
    env_type: str = MujocoEnvCfg.model_fields["env_type"].default
    is_sim: bool = MujocoEnvCfg.model_fields["is_sim"].default
    # ====== ENV CONFIGURATION ======
    update_with_fk: bool = True


class G1_12MujocoEnvCfg(G1_12EnvCfg, MujocoEnvCfg):
    env_type: str = MujocoEnvCfg.model_fields["env_type"].default
    is_sim: bool = MujocoEnvCfg.model_fields["is_sim"].default
    # ====== ENV CONFIGURATION ======
    update_with_fk: bool = False


class G1MujocoBoxEnvCfg(G1EnvCfg, MujocoBoxEnvCfg):
    """MuJoCo environment configuration with a box in the scene.
    
    This configuration uses MujocoBoxEnvCfg to enable separate box state tracking.
    It inherits from G1EnvCfg for robot configuration and MujocoBoxEnvCfg for box tracking.
    """
    env_type: str = MujocoBoxEnvCfg.model_fields["env_type"].default
    is_sim: bool = MujocoBoxEnvCfg.model_fields["is_sim"].default
    xml: str = (ASSETS_DIR / "robots/g1/scene_g1_box.xml").as_posix()
    
    update_with_fk: bool = True
    
class G1MujocoCylinderEnvCfg(G1EnvCfg, MujocoBoxEnvCfg):
    """MuJoCo environment configuration with a cylinder in the scene.
    
    This configuration uses MujocoBoxEnvCfg to enable separate box state tracking.
    It inherits from G1EnvCfg for robot configuration and MujocoBoxEnvCfg for box tracking.
    """
    env_type: str = MujocoBoxEnvCfg.model_fields["env_type"].default
    is_sim: bool = MujocoBoxEnvCfg.model_fields["is_sim"].default
    xml: str = (ASSETS_DIR / "robots/g1/scene_g1_cylinder.xml").as_posix()
    
    update_with_fk: bool = True
    
class G1MujocoTableBoxEnvCfg(G1EnvCfg, MujocoBoxEnvCfg):
    """MuJoCo environment configuration with a box on a table in the scene.
    
    This configuration uses MujocoBoxEnvCfg to enable separate box state tracking.
    It inherits from G1EnvCfg for robot configuration and MujocoBoxEnvCfg for box tracking.
    """
    env_type: str = MujocoBoxEnvCfg.model_fields["env_type"].default
    is_sim: bool = MujocoBoxEnvCfg.model_fields["is_sim"].default
    xml: str = (ASSETS_DIR / "robots/g1/scene_g1_box_table.xml").as_posix()
    
    update_with_fk: bool = True
