import logging

import mujoco
import numpy as np

from atarictrl.environment import env_registry
from atarictrl.environment.mujoco_env import MujocoEnv
from atarictrl.environment.env_cfgs import MujocoBoxEnvCfg
from atarictrl.utils.util_func import quat_rotate_inverse_np, quatToEuler

logger = logging.getLogger(__name__)


@env_registry.register
class MujocoBoxEnv(MujocoEnv):
    """MuJoCo environment with box state tracking.
    
    Extends MujocoEnv to separately track box states (position, orientation, velocities)
    from robot states. The box must have a free joint.
    """
    
    def __init__(self, cfg_env: MujocoBoxEnvCfg, device="cpu"):
        # Initialize box state variables first (before super().__init__ calls update())
        self._box_pos: np.ndarray | None = None
        self._box_quat: np.ndarray | None = None
        self._box_lin_vel: np.ndarray | None = None
        self._box_ang_vel: np.ndarray | None = None
        # World frame velocities (before transformation to box frame)
        self._box_lin_vel_w: np.ndarray | None = None
        self._box_ang_vel_w: np.ndarray | None = None
        
        # Initialize box joint attributes as None (will be set after super().__init__)
        # This prevents AttributeError when _update_box_state() is called during super().__init__()
        self.box_joint_qposadr: int | None = None
        self.box_joint_dofadr: int | None = None
        
        # Initialize robot joint indices as None (will be set after super().__init__)
        # This prevents AttributeError when update() is called during super().__init__()
        self.robot_base_qposadr: int | None = None
        self.robot_base_dofadr: int | None = None
        self.robot_dof_qpos_indices: list[int] | None = None
        self.robot_dof_qvel_indices: list[int] | None = None
        
        # Call parent init (this will call update() which will try to call _update_box_state(),
        # but it will return early since box_joint_qposadr is None)
        super().__init__(cfg_env=cfg_env, device=device)
        
        # Now set up box tracking after parent initialization is complete
        self.box_body_name = cfg_env.box_body_name
        self.box_joint_name = cfg_env.box_joint_name
        
        # Find robot's floating base joint to ensure we're reading robot states correctly
        self.robot_base_joint_name = "floating_base_joint"
        self.robot_base_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.robot_base_joint_name)  # pyright: ignore[reportAttributeAccessIssue]
        if self.robot_base_joint_id < 0:
            raise ValueError(f"Robot base joint '{self.robot_base_joint_name}' not found in model")
        
        # Get robot base joint addresses
        self.robot_base_qposadr = self.model.jnt_qposadr[self.robot_base_joint_id]  # pyright: ignore[reportAttributeAccessIssue]
        self.robot_base_dofadr = self.model.jnt_dofadr[self.robot_base_joint_id]  # pyright: ignore[reportAttributeAccessIssue]
        
        # Find robot DOF joint addresses by name
        self.robot_dof_qpos_indices = []
        self.robot_dof_qvel_indices = []
        for joint_name in self.joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)  # pyright: ignore[reportAttributeAccessIssue]
            if joint_id < 0:
                raise ValueError(f"Robot joint '{joint_name}' not found in model")
            qposadr = self.model.jnt_qposadr[joint_id]  # pyright: ignore[reportAttributeAccessIssue]
            dofadr = self.model.jnt_dofadr[joint_id]  # pyright: ignore[reportAttributeAccessIssue]
            joint_type = self.model.jnt_type[joint_id]  # pyright: ignore[reportAttributeAccessIssue]
            
            # Get the number of qpos and qvel for this joint
            if joint_type == mujoco.mjtJoint.mjJNT_FREE:  # pyright: ignore[reportAttributeAccessIssue]
                nqpos, nqvel = 7, 6
            elif joint_type == mujoco.mjtJoint.mjJNT_BALL:  # pyright: ignore[reportAttributeAccessIssue]
                nqpos, nqvel = 4, 3
            else:
                nqpos, nqvel = 1, 1
            
            self.robot_dof_qpos_indices.extend(range(qposadr, qposadr + nqpos))
            self.robot_dof_qvel_indices.extend(range(dofadr, dofadr + nqvel))
        
        # Find box body ID
        self.box_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.box_body_name)  # pyright: ignore[reportAttributeAccessIssue]
        if self.box_body_id < 0:
            raise ValueError(f"Box body '{self.box_body_name}' not found in model")
        
        # Find box joint ID and its indices in qpos/qvel
        self.box_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.box_joint_name)  # pyright: ignore[reportAttributeAccessIssue]
        if self.box_joint_id < 0:
            raise ValueError(f"Box joint '{self.box_joint_name}' not found in model")
        
        # Get joint address (indices in qpos/qvel)
        self.box_joint_qposadr = self.model.jnt_qposadr[self.box_joint_id]  # pyright: ignore[reportAttributeAccessIssue]
        self.box_joint_dofadr = self.model.jnt_dofadr[self.box_joint_id]  # pyright: ignore[reportAttributeAccessIssue]
        
        # Verify it's a free joint (should have 7 qpos and 6 qvel)
        joint_type = self.model.jnt_type[self.box_joint_id]  # pyright: ignore[reportAttributeAccessIssue]
        if joint_type != mujoco.mjtJoint.mjJNT_FREE:  # pyright: ignore[reportAttributeAccessIssue]
            raise ValueError(f"Box joint '{self.box_joint_name}' is not a free joint (type: {joint_type})")
        
        self.update()
    
    def _update_box_state(self):
        """Update box state from MuJoCo data."""
        # Return early if box joint attributes haven't been initialized yet
        if self.box_joint_qposadr is None or self.box_joint_dofadr is None:
            return
        
        # Get box position and orientation from qpos (free joint has 7 DOF: 3 pos + 4 quat)
        box_pos = self.data.qpos.astype(np.float32)[self.box_joint_qposadr : self.box_joint_qposadr + 3]
        box_quat_mj = self.data.qpos.astype(np.float32)[self.box_joint_qposadr + 3 : self.box_joint_qposadr + 7]
        # Convert from MuJoCo quaternion format (w, x, y, z) to (x, y, z, w)
        box_quat = box_quat_mj[[1, 2, 3, 0]]
        
        # Apply base alignment transformation if enabled (same as robot base)
        if self.born_place_align:
            box_quat, box_pos = self.base_align.align_transform(box_quat, box_pos)
        
        # Get box velocities from qvel (free joint has 6 DOF: 3 lin + 3 ang)
        box_lin_vel_w = self.data.qvel.astype(np.float32)[self.box_joint_dofadr : self.box_joint_dofadr + 3]
        box_ang_vel_w = self.data.qvel.astype(np.float32)[self.box_joint_dofadr + 3 : self.box_joint_dofadr + 6]
        
        # Store world frame velocities
        self._box_lin_vel_w = box_lin_vel_w.copy()
        self._box_ang_vel_w = box_ang_vel_w.copy()
        
        # Transform linear velocity to box frame (optional, similar to robot base)
        box_lin_vel = quat_rotate_inverse_np(box_quat, box_lin_vel_w)
        
        self._box_pos = box_pos.copy()
        self._box_quat = box_quat.copy()
        self._box_lin_vel = box_lin_vel.copy()
        self._box_ang_vel = box_ang_vel_w.copy()
    
    def update(self, simple=False):
        """Update both robot and box states using correct joint indices."""
        # If robot joint indices haven't been initialized yet, fall back to parent update
        if self.robot_dof_qpos_indices is None or self.robot_dof_qvel_indices is None:
            super().update(simple=simple)
            if not simple:
                self._update_box_state()
            return
        
        # Update DOF positions and velocities using correct indices
        dof_pos = self.data.qpos.astype(np.float32)[self.robot_dof_qpos_indices]
        dof_vel = self.data.qvel.astype(np.float32)[self.robot_dof_qvel_indices]
        
        self._dof_pos = dof_pos.copy()
        self._dof_vel = dof_vel.copy()
        
        if simple:
            # Still update box states even in simple mode (they're separate)
            self._update_box_state()
            return
        
        # Update robot base states using correct indices
        robot_quat_mj = self.data.qpos.astype(np.float32)[self.robot_base_qposadr + 3 : self.robot_base_qposadr + 7]
        robot_ang_vel = self.data.qvel.astype(np.float32)[self.robot_base_dofadr + 3 : self.robot_base_dofadr + 6]
        robot_base_pos = self.data.qpos.astype(np.float32)[self.robot_base_qposadr : self.robot_base_qposadr + 3]
        robot_lin_vel = self.data.qvel.astype(np.float32)[self.robot_base_dofadr : self.robot_base_dofadr + 3]
        
        # Convert from MuJoCo quaternion format (w, x, y, z) to (x, y, z, w)
        robot_quat = robot_quat_mj[[1, 2, 3, 0]]
        
        if self.born_place_align:
            robot_quat, robot_base_pos = self.base_align.align_transform(robot_quat, robot_base_pos)
        
        robot_lin_vel = quat_rotate_inverse_np(robot_quat, robot_lin_vel)
        rpy = quatToEuler(robot_quat)
        
        self._base_rpy = rpy.copy()
        self._base_quat = robot_quat.copy()
        self._base_ang_vel = robot_ang_vel.copy()
        self._base_pos = robot_base_pos.copy()
        self._base_lin_vel = robot_lin_vel.copy()
        
        if self.update_with_fk:
            fk_info = self.fk()
            self._fk_info = fk_info.copy()
            self._torso_ang_vel = fk_info[self._torso_name]["ang_vel"]
            self._torso_quat = fk_info[self._torso_name]["quat"]
            self._torso_pos = fk_info[self._torso_name]["pos"]
        
        # Update box states
        self._update_box_state()
    
    # === Box State Properties ===
    @property
    def box_pos(self):
        """Box position in world frame (x, y, z)."""
        return self._box_pos.copy() if self._box_pos is not None else None
    
    @property
    def box_quat(self):
        """Box orientation quaternion (x, y, z, w)."""
        return self._box_quat.copy() if self._box_quat is not None else None
    
    @property
    def box_lin_vel(self):
        """Box linear velocity in box frame (x, y, z)."""
        return self._box_lin_vel.copy() if self._box_lin_vel is not None else None
    
    @property
    def box_ang_vel(self):
        """Box angular velocity in box frame (x, y, z)."""
        return self._box_ang_vel.copy() if self._box_ang_vel is not None else None
    
    def get_box_data(self):
        """Get box state data as a dictionary."""
        return {
            "box_pos": self.box_pos,
            "box_quat": self.box_quat,
            "box_lin_vel": self.box_lin_vel,
            "box_ang_vel": self.box_ang_vel,
        }
    
    def set_box_state(self, box_pos: np.ndarray | None = None, box_quat: np.ndarray | None = None):
        """Set box position and orientation from external source (e.g., motion data).
        
        Args:
            box_pos: Box position in world frame (x, y, z)
            box_quat: Box orientation quaternion (x, y, z, w)
        """
        if self.box_joint_qposadr is None:
            logger.warning("Box joint not initialized, cannot set box state")
            return
        
        if box_pos is not None:
            assert box_pos.shape == (3,), f"box_pos must be of shape (3,), got {box_pos.shape}"
            self.data.qpos[self.box_joint_qposadr : self.box_joint_qposadr + 3] = box_pos
        
        if box_quat is not None:
            assert box_quat.shape == (4,), f"box_quat must be of shape (4,), got {box_quat.shape}"
            # Convert from (x, y, z, w) to MuJoCo format (w, x, y, z)
            box_quat_mj = box_quat[[3, 0, 1, 2]]
            self.data.qpos[self.box_joint_qposadr + 3 : self.box_joint_qposadr + 7] = box_quat_mj
        
        # Zero out velocities when setting position manually
        if box_pos is not None or box_quat is not None:
            self.data.qvel[self.box_joint_dofadr : self.box_joint_dofadr + 6] = 0.0
            # Forward kinematics to update dependent quantities
            mujoco.mj_forward(self.model, self.data)  # pyright: ignore[reportAttributeAccessIssue]
            # Update box state
            self._update_box_state()
    
    def set_init_dof_pos(self, dof_pos: np.ndarray):
        """Set initial DOF positions in the simulation.
        
        Args:
            dof_pos: Array of DOF positions (length should match num_dofs)
        """
        assert len(dof_pos) == self.num_dofs, f"dof_pos length ({len(dof_pos)}) should match num_dofs ({self.num_dofs})"
        
        # If robot joint indices haven't been initialized yet, fall back to parent method
        if self.robot_dof_qpos_indices is None or self.robot_dof_qvel_indices is None:
            super().set_init_dof_pos(dof_pos)
            return
        
        # Set DOF positions using correct indices (using numpy advanced indexing)
        dof_pos_array = np.asarray(dof_pos, dtype=np.float32)
        self.data.qpos[np.asarray(self.robot_dof_qpos_indices)] = dof_pos_array
        
        # Zero out DOF velocities using correct indices
        self.data.qvel[np.asarray(self.robot_dof_qvel_indices)] = 0.0
        
        # Forward kinematics to update dependent quantities
        mujoco.mj_forward(self.model, self.data)  # pyright: ignore[reportAttributeAccessIssue]
        # Update state
        self.update()

    def get_data(self):
        """Get both robot and box state data."""
        env_data = super().get_data()
        
        # Add box states to extras dictionary with "box/base_*_w" naming convention
        if self._box_pos is not None:
            env_data["box/base_pos_w"] = self.box_pos
            env_data["box/base_quat"] = self.box_quat
            # Use world frame velocities for consistency with naming (velocities are in world frame before alignment)
            if self._box_lin_vel_w is not None:
                env_data["box/base_lin_vel_w"] = self._box_lin_vel_w.copy()
            if self._box_ang_vel_w is not None:
                env_data["box/base_ang_vel_w"] = self._box_ang_vel_w.copy()
        
        return env_data

