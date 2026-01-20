import logging
import time

import mujoco
import numpy as np
from box import Box

import atarictrl.environment
import atarictrl.policy
from atarictrl.controller import CtrlManager
from atarictrl.environment import Environment
from atarictrl.pipeline import Pipeline, pipeline_registry
from atarictrl.pipeline.pipeline_cfgs import RlPipelineCfg
from atarictrl.policy import Policy, PolicyCfg
from atarictrl.tools.dof import DoFAdapter
from atarictrl.tools.tool_cfgs import DoFConfig
from atarictrl.utils.progress import ProgressBar
from atarictrl.utils.util_func import get_gravity_orientation, quat_mul

logger = logging.getLogger(__name__)


class PolicyWrapper:
    """A wrapper for Policy to handle observation and action adaptation."""

    def __init__(self, cfg_policy: PolicyCfg, env_dof_cfg: DoFConfig, device: str):
        self.env_dof_cfg = env_dof_cfg

        policy_type = cfg_policy.policy_type
        policy_name = policy_type
        if hasattr(cfg_policy, "policy_name"):
            policy_name += "@" + cfg_policy.policy_name  # type: ignore
        # while policy_name in self.policies.keys():
        #     policy_name += "_new"
        self.name = policy_name

        policy_class: type[Policy] = getattr(atarictrl.policy, policy_type)
        self.policy: Policy = policy_class(cfg_policy=cfg_policy, device=device)
        self.obs_adapter = DoFAdapter(env_dof_cfg.joint_names, self.policy.cfg_obs_dof.joint_names)
        self.actions_adapter = DoFAdapter(self.policy.cfg_action_dof.joint_names, env_dof_cfg.joint_names)
        
        self.use_motion_offset = False
        if hasattr(self.policy, "use_motion_offset"):
            if self.policy.use_motion_offset:
                self.use_motion_offset = True

    def get_observation(self, env_data: Box, ctrl_data: Box):
        env_data_adapted = env_data.copy()
        env_data_adapted.dof_pos = self.obs_adapter.fit(env_data_adapted.dof_pos)
        env_data_adapted.dof_vel = self.obs_adapter.fit(env_data_adapted.dof_vel)
        return self.policy.get_observation(env_data_adapted, ctrl_data)

    def get_action(self, obs):
        action = self.policy.get_action(obs)
        return self.actions_adapter.fit(action)

    def get_pd_target(self, obs):
        action = self.policy.get_action(obs)
        if hasattr(self.policy, "command"):
            pd_target = action + self.policy.command["joint_pos"]
        else:
            pd_target = action + self.policy.default_pos
        
        return self.actions_adapter.fit(pd_target, template=self.env_dof_cfg.default_pos)

    def get_init_dof_pos(self):
        return self.actions_adapter.fit(self.policy.get_init_dof_pos(), template=self.env_dof_cfg.default_pos)

    def __getattr__(self, name):
        """Fallback: delegate other func to the wrapped policy."""
        return getattr(self.policy, name)


@pipeline_registry.register
class RlPipeline(Pipeline):
    cfg: RlPipelineCfg

    def __init__(self, cfg: RlPipelineCfg):
        super().__init__(cfg=cfg)

        env_class: type[Environment] = getattr(atarictrl.environment, self.cfg.env.env_type)
        self.env: Environment = env_class(cfg_env=self.cfg.env, device=self.device)

        self.ctrl_manager = CtrlManager(cfg_ctrls=self.cfg.ctrl, env=self.env, device=self.device)

        self.policy = PolicyWrapper(
            cfg_policy=self.cfg.policy,
            env_dof_cfg=self.env.dof_cfg,
            device=self.device,
        )

        self.env.update_dof_cfg(override_cfg=self.policy.cfg_action_dof)
        self.visualizer = self.env.visualizer

        self.freq = self.cfg.policy.freq
        self.dt = 1.0 / self.freq

        self.self_check()
        self.reset()

    def self_check(self):
        self.env.self_check()
        for _ in range(2):
            self.step(dry_run=True)

    def reset(self):
        logger.info("Pipeline reset")
        self.timestep = 0

        self.env.reset()
        # self.env.reborn(init_qpos=[0.2, 0.2, 0.8] + [ 0.707, 0, 0, 0.707]) # FOR SIM DEBUG
        self.policy.reset()
        self.ctrl_manager.reset()
        
        # # Initialize robot and box from first frame of trajectory for BeyondMimicObjectPolicy with MujocoBoxEnv
        # if (hasattr(self.policy.policy, "use_motion_from_model") and 
        #     self.policy.policy.use_motion_from_model and
        #     hasattr(self.policy.policy, "command") and
        #     hasattr(self.env, "set_box_state") and
        #     hasattr(self.env, "robot_base_qposadr")):
        #     # Get first frame data by calling get_action with dummy observation
        #     # This populates self.policy.policy.command with the first frame
        #     if self.policy.policy.command is None:
        #         obs_shape = self.policy.policy.session.get_inputs()[0].shape
        #         dummy_obs = np.zeros(obs_shape[1], dtype=np.float32)
        #         self.policy.policy.get_action(dummy_obs)
            
        #     if self.policy.policy.command is not None:
        #         command = self.policy.policy.command
                
        #         # Get robot base pose from anchor body (first frame)
        #         if ("body_pos_w" in command and "body_quat_w" in command and 
        #             hasattr(self.policy.policy, "motion_anchor_body_index") and
        #             self.policy.policy.motion_anchor_body_index >= 0):
                    
        #             anchor_idx = self.policy.policy.motion_anchor_body_index
        #             body_pos_w = command["body_pos_w"]
        #             body_quat_w = command["body_quat_w"]
                    
        #             # body_pos_w and body_quat_w are expected to be 2D: (num_bodies, 3) and (num_bodies, 4)
        #             # Handle both 1D (flattened) and 2D arrays for robustness
        #             if body_pos_w.ndim == 1:
        #                 # If 1D, assume it's flattened and extract the anchor body's data
        #                 # This shouldn't happen normally, but handle it for safety
        #                 num_bodies = body_pos_w.shape[0] // 3
        #                 if num_bodies > anchor_idx:
        #                     start_idx = anchor_idx * 3
        #                     robot_base_pos = body_pos_w[start_idx:start_idx+3]
        #                 else:
        #                     robot_base_pos = body_pos_w[:3]
        #             else:
        #                 # 2D array: (num_bodies, 3)
        #                 robot_base_pos = body_pos_w[anchor_idx, :3]
                    
        #             if body_quat_w.ndim == 1:
        #                 # If 1D, assume it's flattened
        #                 num_bodies = body_quat_w.shape[0] // 4
        #                 if num_bodies > anchor_idx:
        #                     start_idx = anchor_idx * 4
        #                     robot_base_quat_mj = body_quat_w[start_idx:start_idx+4]
        #                 else:
        #                     robot_base_quat_mj = body_quat_w[:4]
        #             else:
        #                 # 2D array: (num_bodies, 4) with format [w, x, y, z]
        #                 robot_base_quat_mj = body_quat_w[anchor_idx, :4]
                    
        #             # Set robot base pose in MuJoCo
        #             self.env.data.qpos[self.env.robot_base_qposadr : self.env.robot_base_qposadr + 3] = robot_base_pos
        #             # robot_base_quat_mj is already in MuJoCo format [w, x, y, z]
        #             self.env.data.qpos[self.env.robot_base_qposadr + 3 : self.env.robot_base_qposadr + 7] = robot_base_quat_mj
        #             # Zero out base velocities
        #             self.env.data.qvel[self.env.robot_base_dofadr : self.env.robot_base_dofadr + 6] = 0.0
                
        #         # Get box pose from first frame
        #         if "object_pos_w" in command and "object_quat_w" in command:
        #             object_pos_w = command["object_pos_w"]
        #             object_quat_w = command["object_quat_w"]
                    
        #             # object_pos_w and object_quat_w are expected to be 1D: (3,) and (4,)
        #             # Handle both 1D and 2D arrays for robustness
        #             if object_pos_w.ndim == 1:
        #                 box_pos = object_pos_w[:3]
        #             else:
        #                 # 2D array: take first frame
        #                 box_pos = object_pos_w[0, :3] if object_pos_w.shape[1] >= 3 else object_pos_w[0, :]
                    
        #             if object_quat_w.ndim == 1:
        #                 # object_quat_w is in format [w, x, y, z] from ONNX model
        #                 box_quat_mj = object_quat_w[:4]
        #             else:
        #                 # 2D array: take first frame
        #                 box_quat_mj = object_quat_w[0, :4] if object_quat_w.shape[1] >= 4 else object_quat_w[0, :]
        #             # box_pos += np.array([0.2, -0.2, 0.0])
        #             box_quat_mj = quat_mul(box_quat_mj, np.array([0.997, 0, 0, 0.0697]))
        #             # Convert quaternion from [w, x, y, z] to [x, y, z, w] for set_box_state
        #             box_quat = box_quat_mj[[1, 2, 3, 0]]
                    
        #             # Set box state
        #             self.env.set_box_state(box_pos=box_pos, box_quat=box_quat)

                
        #         # Forward kinematics to update dependent quantities
        #         if hasattr(self.env, 'model') and hasattr(self.env, 'data'):
        #             mujoco.mj_forward(self.env.model, self.env.data)  # pyright: ignore[reportAttributeAccessIssue]
        #         self.env.update()
        
        
    def safety_check(self):
        if not self.do_safety_check:
            return
        gravity_ori = get_gravity_orientation(self.env.base_quat)
        angle = np.arccos(np.clip(-gravity_ori[2], -1.0, 1.0))
        if abs(angle) > 1.0:  # more than ~57 degrees
            logger.error("Robot fallen! Shutdown for safety.")
            if hasattr(self.env, "reborn"):
                self.env.reborn()  # pyright: ignore[reportAttributeAccessIssue]
            else:
                self.env.shutdown()

    def post_step_callback(self, env_data, ctrl_data, extras, pd_target):
        self.timestep += 1
        commands = ctrl_data.get("COMMANDS", [])
        for command in commands:
            match command:
                case "[SHUTDOWN]":
                    logger.warning("Emergency shutdown!")
                    self.env.shutdown()
                case "[SIM_REBORN]":
                    if hasattr(self.env, "reborn"):
                        logger.warning("Simulation Env reborn!")
                        self.env.reborn()  # pyright: ignore[reportAttributeAccessIssue]

        self.ctrl_manager.post_step_callback(ctrl_data)

        self.policy.post_step_callback(commands)
        if self.visualizer is not None:
            self.policy.debug_viz(self.visualizer, env_data, ctrl_data, extras)

        self.safety_check()
        if self.cfg.debug.log_obs:
            self.debug_logger.log(
                env_data=env_data,
                ctrl_data=ctrl_data,
                extras=extras,
                pd_target=pd_target,
                timestep=self.timestep,
            )

    def step(self, dry_run=False):
        self.env.update()
        env_data = self.env.get_data()

        ctrl_data = self.ctrl_manager.get_ctrl_data(env_data)

        commands = ctrl_data.get("COMMANDS", [])
        if len(commands) > 0:
            logger.info(f"{'=' * 10} COMMANDS {'=' * 10}\n{commands}")

        obs, extras = self.policy.get_observation(env_data, ctrl_data)
        pd_target = self.policy.get_pd_target(obs)

        if not dry_run:
            self.env.step(pd_target, extras.get("hand_pose", None))

        self.post_step_callback(env_data, ctrl_data, extras, pd_target)

    def prepare(self, init_motor_angle=None):
        if init_motor_angle is not None:
            desired_motor_angle = init_motor_angle
        else:
            desired_motor_angle = self.policy.get_init_dof_pos()

        # logger.info(f"{desired_motor_angle=}")
        current_motor_angle = np.array(self.env.dof_pos)
        # logger.info(f"{current_motor_angle=}")

        traj_len = 1000
        last_step_time = time.time()
        logger.warning("prepare_init")
        pbar = ProgressBar("Prepare", traj_len)

        for t in range(traj_len):
            current_motor_angle = np.array(self.env.dof_pos)

            blend_ratio = np.minimum(t / 300, 1)
            action = (1 - blend_ratio) * current_motor_angle + blend_ratio * desired_motor_angle

            # warm up network
            self.step(dry_run=True)

            self.env.step(action)

            time_diff = last_step_time + self.dt - time.time()
            if time_diff > 0:
                time.sleep(time_diff)
            else:
                logger.error("Warning: frame drop")
            last_step_time = time.time()
            pbar.update()

            if t == 0.9 * traj_len:
                logger.info(f"{'=' * 10} RESET ZERO POSITION {'=' * 10}")
                self.reset()

        time.sleep(0.01)
        pbar.close()
        logger.warning("prepare_done")


if __name__ == "__main__":
    pass
