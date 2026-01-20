import os
import time
import threading

import msgpack
import msgpack_numpy
import mujoco
import numpy as np

from atarictrl.config.g1.env.g1_mujuco_env_cfg import G1MujocoBoxEnvCfg, G1MujocoCylinderEnvCfg
from atarictrl.config.g1.policy.g1_beyondmimic_policy_cfg import G1BeyondMimicDoF
from atarictrl.environment.mujoco_box_env import MujocoBoxEnv
from atarictrl.environment.utils.mujoco_viz import MujocoVisualizer
from atarictrl.tools.tool_cfgs import ForwardKinematicCfg
from atarictrl.utils.rotation import TransformAlignment

msgpack_numpy.patch()


def get_latest_folder(path, index=-1):
    folder_list = os.listdir(path)
    folder_list = [os.path.join(path, f) for f in folder_list]
    folder_list = list(filter(lambda f: os.path.isdir(f), folder_list))
    folder_list.sort(key=lambda f: os.path.getmtime(f))
    return folder_list[index]


def read_msgpack(folder_path):
    log_files = sorted([f for f in os.listdir(folder_path) if f.endswith(".msgpack")])
    if not log_files:
        exit()
    log_file = log_files[0]
    path = os.path.join(folder_path, log_file)

    records = []
    with open(path, "rb") as f:
        unpacker = msgpack.Unpacker(f, raw=False)
        for obj in unpacker:
            records.append(obj)
    return records


def view_log(folder_path):
    log_frames = read_msgpack(folder_path)
    log_frames = log_frames[:]

    fk_cfg = ForwardKinematicCfg(
        xml_path=G1DummyEnvCfg.model_fields["xml"].default,
        debug_viz=True,
    )
    env_cfg = G1DummyEnvCfg(forward_kinematic=fk_cfg, odometry_type="DUMMY")

    env = DummyEnv(cfg_env=env_cfg)

    for _i, log_frame in enumerate(log_frames):
        env_data = log_frame["env_data"]
        ctrl_data = log_frame["ctrl_data"]
        extras = log_frame["extras"]
        pd_target = log_frame["pd_target"]
        timestep = log_frame["timestep"]
        time_then = log_frame["time"]

        joint_pos = env_data["dof_pos"]
        base_pos = env_data["base_pos"]
        base_quat = env_data["base_quat"]
        env.kinematics.forward(
            joint_pos=joint_pos,
            base_pos=base_pos,
            base_quat=base_quat,
        )
        for command in ctrl_data.get("COMMANDS", []):
            print("----->" + command)
        print(f"Step {timestep}")
        time.sleep(0.005)


def plot_log(folder_path):
    log_frames = read_msgpack(folder_path)
    log_frames = log_frames[:]

    plot_data = [
        [0],
        [],
    ]
    for _i, log_frame in enumerate(log_frames):
        env_data = log_frame["env_data"]
        ctrl_data = log_frame["ctrl_data"]
        extras = log_frame["extras"]
        pd_target = log_frame["pd_target"]
        timestep = log_frame["timestep"]
        time_then = log_frame["time"]

        joint_pos = env_data["dof_pos"]
        base_pos = env_data["base_pos"]
        base_quat = env_data["base_quat"]

        plot_data[0].append(joint_pos[13])  # waist roll
        plot_data[1].append(pd_target[13])

    import matplotlib.pyplot as plt

    fig, ax1 = plt.subplots(figsize=(12, 6))

    ax1.plot(plot_data[0], color="blue", label="DOF Position")
    ax1.set_xlabel("Index", fontsize=12)
    ax1.set_ylabel("DOF Position", color="blue", fontsize=12)
    ax1.tick_params(axis="y", labelcolor="blue")
    ax1.grid()

    ax2 = ax1.twinx()
    ax2.plot(plot_data[1], color="red", label="PD TARGET")
    ax2.set_ylabel("PD target", color="red", fontsize=12)
    ax2.tick_params(axis="y", labelcolor="red")
    fig.legend(loc="upper right", bbox_to_anchor=(0.9, 0.9), fontsize=10)

    plt.title("DOF Position", fontsize=16)
    plt.tight_layout()
    plt.show()


def plot_last_action(folder_path):
    log_frames = read_msgpack(folder_path)
    log_frames = log_frames[:]

    # Initialize plot data for all 14 dimensions
    plot_data = [[] for _ in range(14)]
    
    for _i, log_frame in enumerate(log_frames):
        extras = log_frame["extras"]
        
        if "last_action" in extras:
            last_action = extras["last_action"]
            # Ensure last_action has shape (14)
            if len(last_action) == 14:
                for dim in range(14):
                    plot_data[dim].append(last_action[dim])
            else:
                print(f"Warning: last_action has shape {len(last_action)}, expected 14")
        else:
            print(f"Warning: 'last_action' not found in extras at frame {_i}")

    import matplotlib.pyplot as plt
    import numpy as np

    fig, ax = plt.subplots(figsize=(14, 8))

    # Plot all 14 dimensions
    colors = plt.cm.tab20(np.linspace(0, 1, 14))
    for dim in range(14):
        if plot_data[dim]:  # Only plot if data exists
            ax.plot(plot_data[dim], color=colors[dim], label=f"Action[{dim}]", linewidth=1.5)

    ax.set_xlabel("Timestep", fontsize=12)
    ax.set_ylabel("Action Value", fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", bbox_to_anchor=(1.15, 1.0), fontsize=9, ncol=1)
    
    plt.title("Last Action Values (14 dimensions)", fontsize=16)
    plt.tight_layout()
    plt.show()


def replay_log_mujoco_interactive(folder_path, playback_speed=1.0, use_keyboard=True):
    """
    Replay trajectory with optional interactive keyboard controls.

    Args:
        folder_path: Path to log folder
        playback_speed: Initial playback speed multiplier
        use_keyboard: If False, disable keyboard controls
    Controls (if `pynput` is available and use_keyboard=True):
        SPACE : Pause / Resume
        LEFT  : Step backward one frame
        RIGHT : Step forward one frame
        UP    : Increase speed
        DOWN  : Decrease speed
        R     : Reset to first frame
        ESC   : Quit
    """
    log_frames = read_msgpack(folder_path)
    if not log_frames:
        print("No log frames found!")
        return

    # Try to enable keyboard controls via pynput
    listener = None
    use_pynput = False
    if use_keyboard:
        try:
            from pynput import keyboard as pynput_keyboard
            use_pynput = True
        except ImportError:
            print("Note: 'pynput' not available. Run `pip install pynput` for keyboard controls.")
            use_pynput = False
    else:
        use_pynput = False

    # --- Setup environment & visualizer ---
    bm_dof = G1BeyondMimicDoF()
    fk_cfg = ForwardKinematicCfg(
        xml_path=G1MujocoBoxEnvCfg.model_fields["xml"].default,
        # xml_path=G1MujocoCylinderEnvCfg.model_fields["xml"].default,
        debug_viz=False,
        kinematic_joint_names=bm_dof.joint_names,
    )
    env_cfg = G1MujocoBoxEnvCfg(forward_kinematic=fk_cfg, visualize_extras=True)
    # env_cfg = G1MujocoCylinderEnvCfg(forward_kinematic=fk_cfg, visualize_extras=True)
    env = MujocoBoxEnv(cfg_env=env_cfg)
    visualizer = MujocoVisualizer(env.viewer)

    # --- Initialize command_init_align from first frame ---
    command_init_align = None
    motion_anchor_body_index = -1  # Will be determined from command_dict if available
    
    if log_frames:
        first_frame = log_frames[0]
        first_extras = first_frame.get("extras", {})
        command_dict = first_extras.get("command_dict", None)
        
        if command_dict is not None and isinstance(command_dict, dict):
            # Try to get anchor body index from anchor_pos_w in extras
            if "anchor_pos_w" in first_extras and "anchor_quat_w" in first_extras:
                # Use anchor from extras (reference anchor)
                anchor_pos_w_init = np.array(first_extras["anchor_pos_w"], dtype=np.float32)
                anchor_quat_w_init = np.array(first_extras["anchor_quat_w"], dtype=np.float32)
                
                # Convert quaternion format if needed: [w,x,y,z] -> [x,y,z,w]
                if anchor_quat_w_init[0] > 0.5:  # Heuristic: if first element > 0.5, likely [w,x,y,z]
                    anchor_quat_w_init = anchor_quat_w_init[[1, 2, 3, 0]]
                
                # motion init2anchor alignment (similar to beyondmimicobject_policy.py)
                command_init_align = TransformAlignment(
                    quat=anchor_quat_w_init, pos=anchor_pos_w_init, yaw_only=True, xy_only=True
                )
                print(f"Initialized command_init_align from anchor: pos={anchor_pos_w_init}, quat={anchor_quat_w_init}")
            elif "body_pos_w" in command_dict and "body_quat_w" in command_dict:
                # Fallback: use first body from body_pos_w as anchor
                body_pos_w_init = np.array(command_dict["body_pos_w"], dtype=np.float32)
                body_quat_w_init = np.array(command_dict["body_quat_w"], dtype=np.float32)
                
                if body_pos_w_init.ndim == 2 and body_pos_w_init.shape[0] > 0:
                    motion_anchor_body_index = 0  # Use first body as anchor
                    anchor_pos_w_init = body_pos_w_init[motion_anchor_body_index, :]
                    anchor_quat_w_init = body_quat_w_init[motion_anchor_body_index, :]
                    
                    # Convert quaternion format if needed: [w,x,y,z] -> [x,y,z,w]
                    if anchor_quat_w_init[0] > 0.5:  # Heuristic: if first element > 0.5, likely [w,x,y,z]
                        anchor_quat_w_init = anchor_quat_w_init[[1, 2, 3, 0]]
                    
                    command_init_align = TransformAlignment(
                        quat=anchor_quat_w_init, pos=anchor_pos_w_init, yaw_only=True, xy_only=True
                    )
                    print(f"Initialized command_init_align from body[0]: pos={anchor_pos_w_init}, quat={anchor_quat_w_init}")

    # --- Playback state ---
    class PlaybackState:
        def __init__(self):
            self.current_frame = 5700
            self.paused = False
            self.playback_speed = float(playback_speed)
            self.min_speed = 0.1
            self.max_speed = 5.0
            self.quit = False
            self.lock = threading.Lock()

    state = PlaybackState()

    def clamp_frame(idx: int) -> int:
        if idx < 0:
            return 0
        if idx >= len(log_frames):
            return len(log_frames) - 1
        return idx

    def apply_frame(frame_idx: int):
        """Apply a log frame to the MuJoCo simulation."""
        frame_idx = clamp_frame(frame_idx)
        log_frame = log_frames[frame_idx]
        env_data = log_frame["env_data"]
        extras = log_frame.get("extras", {})

        # Robot state
        joint_pos = np.array(env_data["dof_pos"], dtype=np.float32)
        base_pos = np.array(env_data["base_pos"], dtype=np.float32)
        base_quat = np.array(env_data["base_quat"], dtype=np.float32)

        # Set base pose
        env.data.qpos[env.robot_base_qposadr : env.robot_base_qposadr + 3] = base_pos
        # (x, y, z, w) -> (w, x, y, z)
        base_quat_mj = base_quat[[3, 0, 1, 2]]
        env.data.qpos[env.robot_base_qposadr + 3 : env.robot_base_qposadr + 7] = base_quat_mj

        # Set joints
        env.data.qpos[env.robot_dof_qpos_indices] = joint_pos

        # Zero velocities for clean replay
        env.data.qvel[:] = 0.0

        # Box state (if logged)
        if "box/base_pos_w" in env_data and "box/base_quat" in env_data:
            box_pos = np.array(env_data["box/base_pos_w"], dtype=np.float32)
            box_quat = np.array(env_data["box/base_quat"], dtype=np.float32)
            env.set_box_state(box_pos=box_pos, box_quat=box_quat)
        
        
        # Forward simulation
        mujoco.mj_forward(env.model, env.data)
        # Extra visualization (anchors, desired box pose)
        if visualizer and extras:
            if "robot_anchor_pos_w" in extras and "robot_anchor_quat_w" in extras:
                print("Current Position: ", extras["robot_anchor_pos_w"])
                robot_anchor_pos = extras["robot_anchor_pos_w"]
                robot_anchor_quat = extras["robot_anchor_quat_w"]
                visualizer.draw_arrow(
                    robot_anchor_pos, robot_anchor_quat, [0.2, 0, 0],
                    color=[0, 1, 0, 1], scale=2, id=1
                )

            if "anchor_pos_w" in extras and "anchor_quat_w" in extras:
                print("Reference: ", extras["anchor_pos_w"])
                anchor_pos = extras["anchor_pos_w"]
                anchor_quat = extras["anchor_quat_w"]
                visualizer.draw_arrow(
                    anchor_pos, anchor_quat, [0.2, 0, 0],
                    color=[1, 0, 0, 1], scale=2, id=0
                )

            if "object_pos_w" in extras:
                command_dict = extras.get("command_dict", None)
                object_pos = extras["object_pos_w"]
                if isinstance(object_pos, np.ndarray) and object_pos.size == 3:
                    object_quat = extras.get(
                        "object_quat_w",
                        np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
                    )
                    if isinstance(object_quat, np.ndarray) and object_quat.size == 4:
                        # Heuristic: if format is [w,x,y,z], convert to [x,y,z,w]
                        if object_quat[0] > 0.5:
                            object_quat = object_quat[[1, 2, 3, 0]]
                        box_size = np.array([0.155, 0.155, 0.17], dtype=np.float32)
                        axis_length = 0.5  # Length of each axis arrow
                        visualizer.draw_box(
                            pos=object_pos,
                            quat=object_quat,
                            size=box_size,
                            color=[0.0, 1.0, 0.0, 0.6],
                            label="Desired Box Pose",
                            id=1000,
                        )
                        visualizer.draw_arrow(
                            origin=object_pos,
                            root_quat=object_quat,
                            vec_local=[axis_length, 0.0, 0.0],
                            color=[1.0, 0.0, 0.0, 1.0],  # Red
                            scale=1.0,
                            id=1001,

                        )
                        # Y axis (green)
                        visualizer.draw_arrow(
                            origin=object_pos,
                            root_quat=object_quat,
                            vec_local=[0.0, axis_length, 0.0],
                            color=[0.0, 1.0, 0.0, 1.0],  # Green
                            scale=1.0,
                            id=101,
                        )
                        
                        # Z axis (blue)
                        visualizer.draw_arrow(
                            origin=object_pos,
                            root_quat=object_quat,
                            vec_local=[0.0, 0.0, axis_length],
                            color=[0.0, 0.0, 1.0, 1.0],  # Blue
                            scale=1.0,
                            id=102,
                        )
                        

            # Draw actual object frame (X, Y, Z axes)
            if "box/base_pos_w" in env_data and "box/base_quat" in env_data:
                object_pos_w = np.array(env_data["box/base_pos_w"], dtype=np.float32)
                object_quat_w = np.array(env_data["box/base_quat"], dtype=np.float32)  # [x, y, z, w] format
                
                axis_length = 0.5  # Length of each axis arrow
                
                # X axis (red)
                visualizer.draw_arrow(
                    origin=object_pos_w,
                    root_quat=object_quat_w,
                    vec_local=[axis_length, 0.0, 0.0],
                    color=[1.0, 0.0, 0.0, 1.0],  # Red
                    scale=1.0,
                    id=10,
                )
                
                # Y axis (green)
                visualizer.draw_arrow(
                    origin=object_pos_w,
                    root_quat=object_quat_w,
                    vec_local=[0.0, axis_length, 0.0],
                    color=[0.0, 1.0, 0.0, 1.0],  # Green
                    scale=1.0,
                    id=11,
                )
                
                # Z axis (blue)
                visualizer.draw_arrow(
                    origin=object_pos_w,
                    root_quat=object_quat_w,
                    vec_local=[0.0, 0.0, axis_length],
                    color=[0.0, 0.0, 1.0, 1.0],  # Blue
                    scale=1.0,
                    id=12,
                )

            # Draw green spheres at body_pos_w positions from command
            # First try to get command_dict (new format with body_pos_w)
            command_dict = extras.get("command_dict", None)
            body_pos_w = None
            body_quat_w = None
            
            if command_dict is not None:
                # Try to get body_pos_w from command_dict - handle both string and bytes keys
                if isinstance(command_dict, dict):
                    if "body_pos_w" in command_dict:
                        body_pos_w = command_dict["body_pos_w"]
                    if "body_quat_w" in command_dict:
                        body_quat_w = command_dict["body_quat_w"]

            
            # Fallback: try to get from command if it's a dict (old format)
            if body_pos_w is None:
                command = extras.get("command", None)
                if command is not None and isinstance(command, dict):
                    # Try to get body_pos_w - handle both string and bytes keys
                    if "body_pos_w" in command:
                        body_pos_w = command["body_pos_w"]
                    if "body_quat_w" in command:
                        body_quat_w = command["body_quat_w"]

            
            # Process body_pos_w if we found it
            if body_pos_w is not None:
                # Convert to numpy array if it's a list or other sequence
                if not isinstance(body_pos_w, np.ndarray):
                    body_pos_w = np.array(body_pos_w, dtype=np.float32)
                
                if body_pos_w is not None:
                    body_pos_w = body_pos_w.reshape(-1, 3)
                    
                    if body_pos_w is not None and body_pos_w.size > 0 and body_pos_w.shape[0] > 0:
                        # Process body_quat_w if available
                        if body_quat_w is not None:
                            if not isinstance(body_quat_w, np.ndarray):
                                body_quat_w = np.array(body_quat_w, dtype=np.float32)
                            body_quat_w = body_quat_w.reshape(-1, 4)
                        
                        # Apply alignment transformation if available (similar to beyondmimicobject_policy.py)
                        if command_init_align is not None:
                            # Vectorized position alignment (align_pos supports (N,3) input)
                            body_pos_w = command_init_align.align_pos(body_pos_w)
                            
                            if body_quat_w is not None and body_quat_w.shape[0] == body_pos_w.shape[0]:
                                # Convert quaternion format if needed: [w,x,y,z] -> [x,y,z,w]
                                # Check if first quaternion has w-first format (heuristic: if first element > 0.5)
                                if body_quat_w[0, 0] > 0.5:
                                    body_quat_w = body_quat_w[:, [1, 2, 3, 0]]
                                
                                # Vectorized quaternion alignment
                                # align_quat can handle (N,4) arrays via scipy Rotation
                                from scipy.spatial.transform import Rotation as sRot
                                R_cur = sRot.from_quat(body_quat_w)  # Handles (N,4) arrays
                                R_rel = command_init_align.R_base.inv() * R_cur
                                body_quat_w = R_rel.as_quat()  # Returns (N,4) array in [x,y,z,w] format
                        
                        # Ensure it's float32 and contiguous
                        body_pos_w = np.ascontiguousarray(body_pos_w.astype(np.float32))
                        
                        # Create body_rot array for visualization
                        if body_quat_w is not None and body_quat_w.shape[0] == body_pos_w.shape[0]:
                            # Convert from [x,y,z,w] to [w,x,y,z] format for update_rg_view
                            body_rot = body_quat_w[:, [3, 0, 1, 2]].copy()
                        else:
                            # Create dummy body_rot array (not used by update_rg_view but required)
                            body_rot = np.zeros((body_pos_w.shape[0], 4), dtype=np.float32)  # quaternions [w, x, y, z]
                            body_rot[:, 0] = 1.0  # identity quaternions
                        
                        # Use humanoid_id=1 for green spheres
                        try:
                            visualizer.update_rg_view(body_pos_w, body_rot, humanoid_id=1)
                        except Exception as e:
                            # Silently fail if visualization fails (e.g., wrong shape)
                            if frame_idx == 0:
                                print(f"Warning: Failed to visualize body_pos_w: {e}")

    # --- Keyboard handler (optional, using pynput) ---
    if use_pynput:
        from pynput import keyboard as pynput_keyboard

        def on_press(key):
            try:
                Key = pynput_keyboard.Key
                if key == Key.space:
                    with state.lock:
                        state.paused = not state.paused
                        print("PAUSED" if state.paused else "RESUMED")
                elif key == Key.left:
                    with state.lock:
                        state.current_frame = clamp_frame(state.current_frame - 1)
                        apply_frame(state.current_frame)
                        print(f"Frame: {state.current_frame}/{len(log_frames) - 1}")
                elif key == Key.right:
                    with state.lock:
                        state.current_frame = clamp_frame(state.current_frame + 1)
                        apply_frame(state.current_frame)
                        print(f"Frame: {state.current_frame}/{len(log_frames) - 1}")
                elif key == Key.up:
                    with state.lock:
                        state.playback_speed = min(
                            state.max_speed, state.playback_speed + 0.1
                        )
                        print(f"Speed: {state.playback_speed:.1f}x")
                elif key == Key.down:
                    with state.lock:
                        state.playback_speed = max(
                            state.min_speed, state.playback_speed - 0.1
                        )
                        print(f"Speed: {state.playback_speed:.1f}x")
                elif key == Key.esc:
                    with state.lock:
                        state.quit = True
                elif hasattr(key, "char") and key.char:
                    if key.char.lower() == "r":
                        with state.lock:
                            state.current_frame = 0
                            apply_frame(0)
                            print("Reset to beginning")
            except Exception:
                # Don't let keyboard errors kill the viewer loop
                pass

        listener = pynput_keyboard.Listener(on_press=on_press, suppress=False)
        listener.start()

    print("\n" + "=" * 60)
    print("MuJoCo Log Replay")
    print(f"Total frames: {len(log_frames)}")
    print(f"Initial speed: {playback_speed:.1f}x")
    if use_pynput:
        print("Keyboard controls:")
        print("  SPACE : Pause / Resume")
        print("  LEFT  : Step backward")
        print("  RIGHT : Step forward")
        print("  UP    : Increase speed")
        print("  DOWN  : Decrease speed")
        print("  R     : Reset to first frame")
        print("  ESC   : Quit")
    else:
        print("Keyboard controls disabled (or pynput missing).")
        print("Close the viewer window to quit.")
    print("=" * 60 + "\n")

    # --- Initial frame ---
    apply_frame(0)
    last_time = time.time()

    # --- Main rendering / playback loop ---
    try:
        while env.viewer.is_alive:
            with state.lock:
                if state.quit:
                    break
                paused = state.paused
                current_frame = state.current_frame
                speed = state.playback_speed

            now = time.time()
            frame_delay = 0.02 / max(speed, 1e-4)

            # Advance frame if not paused
            if not paused and (now - last_time) >= frame_delay:
                current_frame = clamp_frame(current_frame + 1)
                apply_frame(current_frame)
                with state.lock:
                    state.current_frame = current_frame
                last_time = now

                # Stop at the last frame and pause
                if current_frame >= len(log_frames) - 1:
                    with state.lock:
                        state.paused = True

            # Camera & render
            env.viewer.cam.lookat = env.data.qpos.astype(np.float32)[:3]
            env.viewer.render()

            # Small sleep to avoid busy-waiting
            time.sleep(0.001)
    finally:
        if listener is not None:
            listener.stop()
        env.shutdown()


if __name__ == "__main__":
    import sys
    
    folder_path = get_latest_folder("logs", -1)
    print(folder_path)
    # plot_log(folder_path)
    # plot_last_action(folder_path)
    
    # Replay in MuJoCo viewer
    # Set use_keyboard=False if keyboard controls are causing issues
    use_keyboard = "--no-keyboard" not in sys.argv
    replay_log_mujoco_interactive(folder_path, playback_speed=1.0, use_keyboard=use_keyboard)

