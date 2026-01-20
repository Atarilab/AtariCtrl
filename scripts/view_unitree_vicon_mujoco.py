"""
Visualization script that reads robot and box states from UnitreeEnv and ViconOdometry
and displays them in MujocoBoxEnv for sanity checking.

This script:
1. Reads robot DOF positions from UnitreeEnv (real robot)
2. Reads robot base position/quaternion from ViconOdometry
3. Reads box position/quaternion from ViconOdometry
4. Sets all states in MujocoBoxEnv for visualization
"""

import logging
import time

import mujoco
import numpy as np
import atarictrl.environment

from atarictrl.config.g1.env.g1_mujuco_env_cfg import G1MujocoBoxEnvCfg
from atarictrl.config.g1.env.g1_real_env_cfg import G1RealEnvCfg, G1UnitreeCfg
from atarictrl.environment import Environment
from atarictrl.environment.mujoco_box_env import MujocoBoxEnv
from atarictrl.environment.utils.mujoco_viz import MujocoVisualizer
from atarictrl.tools.tool_cfgs import ViconOdometryCfg

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """Main visualization loop."""
    # Configuration for ViconOdometry (for robot and box states)
    vicon_cfg = ViconOdometryCfg(
        zmq_address="tcp://localhost:5555",
        topic_filter="",
        segments=[("G1", "G1"), ("yellowbox", "yellowbox")],  # Robot and box segments
        robot_segment=("G1", "G1"),
        pos_offset=[0.0, 0.0, 0.0],
        zero_align=True,
    )

    # Configuration for UnitreeEnv (real robot - for DOF positions)
    unitree_env_cfg = G1RealEnvCfg(
        env_type="UnitreeCppEnv",  # or "UnitreeEnv" for unitree_sdk2py
        unitree=G1UnitreeCfg(
            net_if="enx3c4937046061",  # Change to your network interface
            hand_type="NONE",
        ),
        odometry_type="VICON",  # Use VICON for base position
        vicon_cfg=vicon_cfg,
    )

    # Configuration for MujocoBoxEnv (simulation for visualization)
    mujoco_env_cfg = G1MujocoBoxEnvCfg()

    logger.info("Initializing real robot environment...")
    # Use environment registry to dynamically load the environment class
    env_class: type[Environment] = getattr(atarictrl.environment, unitree_env_cfg.env_type)
    unitree_env = env_class(cfg_env=unitree_env_cfg, device="cpu")

    logger.info("Initializing MujocoBoxEnv (simulation)...")
    mujoco_env = MujocoBoxEnv(cfg_env=mujoco_env_cfg, device="cpu")

    # Disable overlay text
    mujoco_env.viewer._overlay = {}

    # Create visualizer for drawing coordinate frames
    visualizer = MujocoVisualizer(mujoco_env.viewer)

    # Access ViconOdometry from UnitreeEnv (it's already initialized there)
    if not hasattr(unitree_env, "vicon_odometry"):
        raise RuntimeError(
            "UnitreeEnv does not have vicon_odometry. Make sure odometry_type is 'VICON' in the config."
        )
    vicon_odom = unitree_env.vicon_odometry

    # Find box segment name from ViconOdometry
    box_segment_name = None
    for segment_name in vicon_odom._segment_lookup.values():
        if "box" in segment_name.lower():
            box_segment_name = segment_name
            break

    if box_segment_name is None:
        logger.warning("Box segment not found in ViconOdometry segments. Box visualization will be skipped.")

    logger.info("Starting visualization loop...")
    logger.info("Press Ctrl+C to exit")

    try:
        while True:
            # Update UnitreeEnv to get latest robot DOF positions
            unitree_env.update()

            # Update ViconOdometry to get latest robot and box states
            vicon_odom.update()

            # Get robot DOF positions from UnitreeEnv
            robot_dof_pos = unitree_env.dof_pos

            # Get robot base position and quaternion from ViconOdometry
            if vicon_odom.is_valid:
                robot_base_pos = vicon_odom.pos
                robot_base_quat = vicon_odom.quat  # Already in (x, y, z, w) format
            else:
                # Fallback to UnitreeEnv if ViconOdometry is not valid
                robot_base_pos = unitree_env.base_pos
                robot_base_quat = unitree_env.base_quat
                if robot_base_pos is None:
                    robot_base_pos = np.array([0.0, 0.0, 0.8], dtype=np.float32)
                if robot_base_quat is None:
                    robot_base_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
                logger.warning("ViconOdometry not valid, using UnitreeEnv data")

            # Get box position and quaternion from ViconOdometry
            box_pos = None
            box_quat = None
            if box_segment_name is not None and vicon_odom.is_segment_valid(box_segment_name):
                box_pos = vicon_odom.get_segment_pos(box_segment_name)
                box_quat = vicon_odom.get_segment_quat(box_segment_name)

            # Set robot base position in MuJoCo
            mujoco_env.data.qpos[mujoco_env.robot_base_qposadr : mujoco_env.robot_base_qposadr + 3] = robot_base_pos

            # Convert quaternion from (x, y, z, w) to MuJoCo format (w, x, y, z)
            robot_base_quat_mj = robot_base_quat[[3, 0, 1, 2]]
            mujoco_env.data.qpos[
                mujoco_env.robot_base_qposadr + 3 : mujoco_env.robot_base_qposadr + 7
            ] = robot_base_quat_mj

            # Set robot DOF positions in MuJoCo
            mujoco_env.data.qpos[mujoco_env.robot_dof_qpos_indices] = robot_dof_pos

            # Zero out velocities (we're just visualizing, not simulating dynamics)
            mujoco_env.data.qvel[mujoco_env.robot_base_dofadr : mujoco_env.robot_base_dofadr + 6] = 0.0
            mujoco_env.data.qvel[mujoco_env.robot_dof_qvel_indices] = 0.0

            # Set box state if available
            if box_pos is not None and box_quat is not None:
                mujoco_env.set_box_state(box_pos=box_pos, box_quat=box_quat)

            # Forward kinematics to update dependent quantities
            mujoco.mj_forward(mujoco_env.model, mujoco_env.data)  # pyright: ignore[reportAttributeAccessIssue]
            # Draw origin frame (X, Y, Z axes at [0, 0, 0])
            origin_pos = np.array([0.0, 0.0, 0.0])
            origin_quat = np.array([0.0, 0.0, 0.0, 1.0])  # Identity quaternion [x, y, z, w]
            axis_length = 0.2  # Length of each axis arrow
            
            # X axis (red)
            visualizer.draw_arrow(
                origin=origin_pos,
                root_quat=origin_quat,
                vec_local=[axis_length, 0.0, 0.0],
                color=[1.0, 0.0, 0.0, 1.0],  # Red
                scale=1.0,
                id=0,
            )
            
            # Y axis (green)
            visualizer.draw_arrow(
                origin=origin_pos,
                root_quat=origin_quat,
                vec_local=[0.0, axis_length, 0.0],
                color=[0.0, 1.0, 0.0, 1.0],  # Green
                scale=1.0,
                id=1,
            )
            
            # Z axis (blue)
            visualizer.draw_arrow(
                origin=origin_pos,
                root_quat=origin_quat,
                vec_local=[0.0, 0.0, axis_length],
                color=[0.0, 0.0, 1.0, 1.0],  # Blue
                scale=1.0,
                id=2,
            )

            # Draw box frame if box state is available
            if box_pos is not None and box_quat is not None:
                box_axis_length = 0.15  # Slightly smaller than origin frame
                
                # X axis (red)
                visualizer.draw_arrow(
                    origin=box_pos,
                    root_quat=box_quat,
                    vec_local=[box_axis_length, 0.0, 0.0],
                    color=[1.0, 0.0, 0.0, 1.0],  # Red
                    scale=1.0,
                    id=3,
                )
                
                # Y axis (green)
                visualizer.draw_arrow(
                    origin=box_pos,
                    root_quat=box_quat,
                    vec_local=[0.0, box_axis_length, 0.0],
                    color=[0.0, 1.0, 0.0, 1.0],  # Green
                    scale=1.0,
                    id=4,
                )
                
                # Z axis (blue)
                visualizer.draw_arrow(
                    origin=box_pos,
                    root_quat=box_quat,
                    vec_local=[0.0, 0.0, box_axis_length],
                    color=[0.0, 0.0, 1.0, 1.0],  # Blue
                    scale=1.0,
                    id=5,
                )

            # Update camera to follow robot
            mujoco_env.viewer.cam.lookat = mujoco_env.data.qpos.astype(np.float32)[:3]

            # Clear overlay text to keep it hidden
            mujoco_env.viewer._overlay = {}

            # Render
            if mujoco_env.viewer.is_alive:
                mujoco_env.viewer.render()
            else:
                logger.warning("Viewer is not alive, exiting...")
                break

            # Sleep to match control frequency (adjust as needed)
            time.sleep(0.02)  # ~50 Hz

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Error in visualization loop: {e}", exc_info=True)
    finally:
        logger.info("Shutting down...")
        unitree_env.shutdown()
        mujoco_env.shutdown()
        logger.info("Done")


if __name__ == "__main__":
    main()

