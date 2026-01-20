import logging
import time

import mujoco
import mujoco_viewer
import numpy as np

from atarictrl.environment import Environment, env_registry
from atarictrl.environment.env_cfgs import MujocoEnvCfg
from atarictrl.environment.utils.mujoco_viz import MujocoVisualizer
from atarictrl.utils.util_func import quat_rotate_inverse_np, quatToEuler

logger = logging.getLogger(__name__)


@env_registry.register
class MujocoEnv(Environment):
    cfg_env: MujocoEnvCfg

    def __init__(self, cfg_env: MujocoEnvCfg, device="cpu"):
        super().__init__(cfg_env=cfg_env, device=device)

        self.sim_duration = cfg_env.sim_duration
        self.sim_dt = cfg_env.sim_dt
        self.sim_decimation = cfg_env.sim_decimation
        self.control_dt = self.sim_dt * self.sim_decimation

        self.model = mujoco.MjModel.from_xml_path(cfg_env.xml)  # pyright: ignore[reportAttributeAccessIssue]
        self.model.opt.timestep = self.sim_dt
        self.data = mujoco.MjData(self.model)  # pyright: ignore[reportAttributeAccessIssue]
        # mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_step(self.model, self.data)  # pyright: ignore[reportAttributeAccessIssue]
        
        # Dual Actuator Setup
        if self.model.nu == 2 * self.num_dofs:
            logger.info("Dual actuator mode detected.")
            self.dual_actuator_mode = True
            self.motor_indices = np.arange(self.num_dofs)
            self.pos_indices = np.arange(self.num_dofs, 2 * self.num_dofs)
            
            # Store default gains for position actuators
            self.default_pos_gains = self.model.actuator_gainprm[self.pos_indices, :].copy()
            self.default_pos_bias = self.model.actuator_biasprm[self.pos_indices, :].copy()
            
            self.set_control_mode("motor") # Default to motor
        else:
            self.dual_actuator_mode = False
            self.control_mode = "motor"

        self.viewer = mujoco_viewer.MujocoViewer(
            self.model,
            self.data,
            width=1200,
            height=900,
            hide_menus=True,
            diable_key_callbacks=True,
        )
        self.viewer.cam.distance = 3.0
        self.viewer.cam.elevation = -10.0
        self.viewer.cam.azimuth = 180.0
        # self.viewer._paused = True

        if cfg_env.visualize_extras:
            self.visualizer = MujocoVisualizer(self.viewer)
        else:
            self.visualizer = None

        self.last_time = time.time()

        self.update()  # get initial state
        
        # Find robot's floating base joint for fix_base functionality
        self.robot_base_joint_name = "floating_base_joint"
        self.robot_base_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.robot_base_joint_name)  # pyright: ignore[reportAttributeAccessIssue]
        if self.robot_base_joint_id >= 0:
            # Get robot base joint addresses
            self.robot_base_qposadr = self.model.jnt_qposadr[self.robot_base_joint_id]  # pyright: ignore[reportAttributeAccessIssue]
            self.robot_base_dofadr = self.model.jnt_dofadr[self.robot_base_joint_id]  # pyright: ignore[reportAttributeAccessIssue]
        else:
            # Fall back to hardcoded indices if joint not found (for backward compatibility)
            self.robot_base_qposadr = 0
            self.robot_base_dofadr = 0
        
        # Fix base if necessary (e.g. for bimanual manipulation)
        self.fix_base = cfg_env.fix_base
        if self.fix_base:
            # Store fixed base position and orientation
            if cfg_env.fixed_base_pos is not None:
                self._fixed_base_pos = np.array(cfg_env.fixed_base_pos, dtype=np.float32)
            else:
                # Use initial position
                self._fixed_base_pos = self._base_pos.copy() if self._base_pos is not None else np.zeros(3, dtype=np.float32)
            
            if cfg_env.fixed_base_quat is not None:
                self._fixed_base_quat = np.array(cfg_env.fixed_base_quat, dtype=np.float32)
            else:
                # Use initial orientation
                self._fixed_base_quat = self._base_quat.copy() if self._base_quat is not None else np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
            
            # Convert quaternion from (x, y, z, w) to MuJoCo format (w, x, y, z)
            self._fixed_base_quat_mj = self._fixed_base_quat[[3, 0, 1, 2]]
            
            # Apply fixed base immediately if enabled
            if self._fixed_base_pos is not None:
                self.data.qpos[self.robot_base_qposadr : self.robot_base_qposadr + 3] = self._fixed_base_pos
                self.data.qpos[self.robot_base_qposadr + 3 : self.robot_base_qposadr + 7] = self._fixed_base_quat_mj
                self.data.qvel[self.robot_base_dofadr : self.robot_base_dofadr + 3] = 0.0
                self.data.qvel[self.robot_base_dofadr + 3 : self.robot_base_dofadr + 6] = 0.0
                mujoco.mj_forward(self.model, self.data)  # pyright: ignore[reportAttributeAccessIssue]
                self.update()  # Update state after fixing base
        else:
            self._fixed_base_pos = None
            self._fixed_base_quat = None
            self._fixed_base_quat_mj = None

    def reborn(self, init_qpos=None):
        if init_qpos is not None:
            self.data.qpos[0:7] = init_qpos
            self.data.qvel[:] = 0.0
            self.data.ctrl[:] = 0.0
        else:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)  # pyright: ignore[reportAttributeAccessIssue]
        mujoco.mj_forward(self.model, self.data)  # pyright: ignore[reportAttributeAccessIssue]

    def reset(self):
        if self.born_place_align:  # TODO: merge
            self.born_place_align = False  # disable during reset
            self.update()
            self.born_place_align = True  # enable after reset
            self.set_born_place()
            self.update()

    def set_gains(self, stiffness, damping):
        assert len(stiffness) == self.num_dofs and len(damping) == self.num_dofs
        self.stiffness = np.asarray(stiffness)
        self.damping = np.asarray(damping)

    def self_check(self):
        pass

    def set_born_place(self, quat: np.ndarray | None = None, pos: np.ndarray | None = None):
        quat_ = self.base_quat if quat is None else quat
        pos_ = self.base_pos if pos is None else pos
        super().set_born_place(quat_, pos_)

    def set_init_dof_pos(self, dof_pos: np.ndarray):
        """Set initial DOF positions in the simulation.
        
        Args:
            dof_pos: Array of DOF positions (length should match num_dofs)
        """
        assert len(dof_pos) == self.num_dofs, f"dof_pos length ({len(dof_pos)}) should match num_dofs ({self.num_dofs})"
        # Set DOF positions (last num_dofs elements of qpos)
        self.data.qpos[-self.num_dofs :] = np.asarray(dof_pos, dtype=np.float32)
        # Zero out DOF velocities
        self.data.qvel[-self.num_dofs :] = 0.0
        # Forward kinematics to update dependent quantities
        mujoco.mj_forward(self.model, self.data)  # pyright: ignore[reportAttributeAccessIssue]
        # Update state
        self.update()

    def update(self, simple=False):  # TODO: clean sensors in xml
        """simple: only update dof pos & vel"""
        dof_pos = self.data.qpos.astype(np.float32)[-self.num_dofs :]
        dof_vel = self.data.qvel.astype(np.float32)[-self.num_dofs :]

        self._dof_pos = dof_pos.copy()
        self._dof_vel = dof_vel.copy()

        if simple:
            return

        quat = self.data.qpos.astype(np.float32)[3:7][[1, 2, 3, 0]]
        ang_vel = self.data.qvel.astype(np.float32)[3:6]
        base_pos = self.data.qpos.astype(np.float32)[:3]
        lin_vel = self.data.qvel.astype(np.float32)[0:3]

        if self.born_place_align:
            quat, base_pos = self.base_align.align_transform(quat, base_pos)

        lin_vel = quat_rotate_inverse_np(quat, lin_vel)
        rpy = quatToEuler(quat)

        self._base_rpy = rpy.copy()
        self._base_quat = quat.copy()
        self._base_ang_vel = ang_vel.copy()

        self._base_pos = base_pos.copy()
        self._base_lin_vel = lin_vel.copy()

        if self.update_with_fk:
            fk_info = self.fk()
            self._fk_info = fk_info.copy()
            self._torso_ang_vel = fk_info[self._torso_name]["ang_vel"]
            self._torso_quat = fk_info[self._torso_name]["quat"]
            self._torso_pos = fk_info[self._torso_name]["pos"]

    def step(self, pd_target, hand_pose=None):
        assert len(pd_target) == self.num_dofs, "pd_target len should be num_dofs of env"

        if hand_pose is not None:
            logger.info("Hand pose-->", hand_pose)

        self.viewer.cam.lookat = self.data.qpos.astype(np.float32)[:3]
        if self.viewer.is_alive:
            self.viewer.render()

        for _ in range(self.sim_decimation):
            torque = (pd_target - self.dof_pos) * self.stiffness - self.dof_vel * self.damping
            torque = np.clip(torque, -self.torque_limits, self.torque_limits)
            self.data.ctrl[self.motor_indices] = torque
            mujoco.mj_step(self.model, self.data)  # pyright: ignore[reportAttributeAccessIssue]
            
            # Fix base if enabled
            if self.fix_base and self._fixed_base_pos is not None:
                # Reset base position and orientation
                self.data.qpos[self.robot_base_qposadr : self.robot_base_qposadr + 3] = self._fixed_base_pos
                self.data.qpos[self.robot_base_qposadr + 3 : self.robot_base_qposadr + 7] = self._fixed_base_quat_mj
                # Reset base velocities to zero
                self.data.qvel[self.robot_base_dofadr : self.robot_base_dofadr + 3] = 0.0
                self.data.qvel[self.robot_base_dofadr + 3 : self.robot_base_dofadr + 6] = 0.0
                # Forward kinematics to update dependent quantities
                mujoco.mj_forward(self.model, self.data)  # pyright: ignore[reportAttributeAccessIssue]
            
            self.update(simple=True)
        self.update(simple=False)


    def set_control_mode(self, mode):
        if not self.dual_actuator_mode:
            return
            
        self.control_mode = mode
        self.model.actuator_gainprm[self.pos_indices, :] = 0
        self.model.actuator_biasprm[self.pos_indices, :] = 0
            
        logger.info(f"Switched control mode to: {mode}")
        
    def shutdown(self):
        self.viewer.close()


if __name__ == "__main__":
    from atarictrl.config.g1.env.g1_mujuco_env_cfg import G1MujocoEnvCfg

    mujoco_env = MujocoEnv(cfg_env=G1MujocoEnvCfg())
    mujoco_env.viewer._paused = False

    while True:
        # mujoco_env.update()
        mujoco_env.step(np.zeros(mujoco_env.num_dofs))
        time.sleep(0.02)
