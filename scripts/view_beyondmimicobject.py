import time

import mujoco

from atarictrl.config.g1.ctrl.g1_beyondmimic_ctrl_cfg import G1BeyondmimicCtrlCfg
from atarictrl.config.g1.env.g1_mujuco_env_cfg import G1MujocoBoxEnvCfg
from atarictrl.config.g1.policy.g1_beyondmimic_policy_cfg import G1BeyondMimicDoF
from atarictrl.controller.beyondmimic_ctrl import BeyondMimicCtrl
from atarictrl.environment.mujoco_box_env import MujocoBoxEnv
from atarictrl.tools.tool_cfgs import ForwardKinematicCfg

bm_dof = G1BeyondMimicDoF()

fk_cfg = ForwardKinematicCfg(
    xml_path=G1MujocoBoxEnvCfg.model_fields["xml"].default,
    debug_viz=False,  # Use environment's viewer instead
    kinematic_joint_names=bm_dof.joint_names,
)
env_cfg = G1MujocoBoxEnvCfg(forward_kinematic=fk_cfg)

env = MujocoBoxEnv(cfg_env=env_cfg)

ctrl_cfg = G1BeyondmimicCtrlCfg(
    motion_name="motion",
)
ctrl_cfg.motion_cfg.anchor_body_name = "pelvis"

ctrl = BeyondMimicCtrl(
    cfg_ctrl=ctrl_cfg,
    env=env,
)
ctrl.playing = True


for i in range(1000):
    joint_pos = ctrl.joint_pos
    base_pos = ctrl.anchor_pos_w
    base_quat = ctrl.anchor_quat_w
    
    # Set robot pose
    env.data.qpos[env.robot_base_qposadr : env.robot_base_qposadr + 3] = base_pos
    # Convert quaternion from (x, y, z, w) to MuJoCo format (w, x, y, z)
    base_quat_mj = base_quat[[3, 0, 1, 2]]
    env.data.qpos[env.robot_base_qposadr + 3 : env.robot_base_qposadr + 7] = base_quat_mj
    env.data.qpos[env.robot_dof_qpos_indices] = joint_pos
    # Set box position if available
    if ctrl.object_pos_w is not None and ctrl.object_quat_w is not None:
        env.set_box_state(box_pos=ctrl.object_pos_w, box_quat=ctrl.object_quat_w)
    
    # Forward kinematics
    mujoco.mj_forward(env.model, env.data)  # pyright: ignore[reportAttributeAccessIssue]
    
    # Render
    if env.viewer.is_alive:
        env.viewer.render()
    
    ctrl.post_step_callback()
    print(f"Step {i}")
    time.sleep(0.02)