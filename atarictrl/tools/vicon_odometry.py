import json
import logging
import threading
import time
from collections import deque

import numpy as np
import zmq
from scipy.spatial.transform import Rotation as sRot

from atarictrl.tools.tool_cfgs import ViconOdometryCfg
from atarictrl.utils.rotation import TransformAlignment
from atarictrl.utils.util_func import quat_rotate_inverse_np

logger = logging.getLogger(__name__)


class ViconOdometry:
    """
    Vicon Odometry class that subscribes to Vicon data via ZMQ and provides
    position, orientation, and velocity information for multiple segments.
    """

    def __init__(self, cfg: ViconOdometryCfg):
        self.cfg = cfg
        self.zero_align = TransformAlignment(yaw_only=True, xy_only=True)

        # Determine segments to track
        if cfg.segments is not None:
            # New multi-segment format
            self.segments = cfg.segments
            self.robot_segment = cfg.robot_segment
        else:
            # Legacy single segment format (backward compatibility)
            if cfg.subject_name and cfg.segment_name:
                self.segments = [(cfg.subject_name, cfg.segment_name)]
                self.robot_segment = (cfg.subject_name, cfg.segment_name)
            else:
                raise ValueError("Either segments or (subject_name, segment_name) must be provided")
        
        # Create segment lookup: (subject_name, segment_name) -> segment_name
        self._segment_lookup = {}
        for subject_name, segment_name in self.segments:
            self._segment_lookup[(subject_name, segment_name)] = segment_name
        
        # Verify robot_segment is in segments
        if self.robot_segment is not None:
            if self.robot_segment not in self._segment_lookup:
                raise ValueError(f"robot_segment {self.robot_segment} not in segments list")
            self.robot_segment_name = self._segment_lookup[self.robot_segment]
        else:
            # If no robot_segment specified, use first segment
            self.robot_segment = self.segments[0]
            self.robot_segment_name = self._segment_lookup[self.robot_segment]
            logger.warning(f"[ViconOdometry] No robot_segment specified, using first segment: {self.robot_segment_name}")

        # ZMQ setup
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(cfg.zmq_address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, cfg.topic_filter)

        # Data storage per segment: keyed by segment_name
        self._valid = {}  # segment_name -> bool
        self._pos = {}  # segment_name -> np.ndarray[3]
        self._quat = {}  # segment_name -> np.ndarray[4] (xyzw)
        self._rpy = {}  # segment_name -> np.ndarray[3]
        self._lin_vel = {}  # segment_name -> np.ndarray[3] (world frame)
        self._lin_vel_body = {}  # segment_name -> np.ndarray[3] (body frame)
        self._ang_vel = {}  # segment_name -> np.ndarray[3] (world frame)
        self._ang_vel_body = {}  # segment_name -> np.ndarray[3] (body frame)

        # Initialize data structures for each segment
        for subject_name, segment_name in self.segments:
            self._valid[segment_name] = False
            self._pos[segment_name] = np.zeros(3, dtype=np.float32)
            self._quat[segment_name] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
            self._rpy[segment_name] = np.zeros(3, dtype=np.float32)
            self._lin_vel[segment_name] = np.zeros(3, dtype=np.float32)
            self._lin_vel_body[segment_name] = np.zeros(3, dtype=np.float32)
            self._ang_vel[segment_name] = np.zeros(3, dtype=np.float32)
            self._ang_vel_body[segment_name] = np.zeros(3, dtype=np.float32)

        # Latest received data per segment: keyed by segment_name
        self._latest_data = {}  # segment_name -> {topic, data, timestamp}
        
        # History for velocity computation per segment
        self._last_pos = {}  # segment_name -> np.ndarray[3]
        self._last_quat = {}  # segment_name -> np.ndarray[4]
        self._last_time = {}  # segment_name -> float
        
        # Smoothed values per segment (for smoothing)
        self._smoothed_pos = {}  # segment_name -> np.ndarray[3]
        self._smoothed_quat = {}  # segment_name -> np.ndarray[4]

        # Threading for receiving messages
        self._stop_event = threading.Event()
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()

        # Poller for non-blocking receive
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

        logger.info(f"[ViconOdometry] Connected to ZMQ publisher at {cfg.zmq_address}")
        if cfg.topic_filter:
            logger.info(f"[ViconOdometry] Subscribed to topics matching: '{cfg.topic_filter}'")
        else:
            logger.info("[ViconOdometry] Subscribed to all topics")
        logger.info(f"[ViconOdometry] Tracking segments: {[s[1] for s in self.segments]}")
        logger.info(f"[ViconOdometry] Robot segment: {self.robot_segment_name}")
        logger.info(f"[ViconOdometry] Smoothing: pos_alpha={cfg.smoothing_alpha_pos}, quat_alpha={cfg.smoothing_alpha_quat}")

        # Self-check
        if not self.self_check():
            raise RuntimeError("[ViconOdometry] self_check failed")

        # Set zero if configured (only for robot segment)
        if self.cfg.zero_align:
            self.set_zero()

    def _receive_loop(self):
        """Background thread to continuously receive ZMQ messages"""
        while not self._stop_event.is_set():
            try:
                socks = dict(self.poller.poll(100))  # 100ms timeout

                if self.socket in socks and socks[self.socket] == zmq.POLLIN:
                    try:
                        # Receive topic name
                        topic = self.socket.recv_string(zmq.NOBLOCK)

                        # Receive JSON data
                        data_str = self.socket.recv_string(zmq.NOBLOCK)

                        # Parse JSON
                        try:
                            position_data = json.loads(data_str)
                            
                            subject_name = position_data.get("subject_name")
                            segment_name = position_data.get("segment_name")
                            
                            # Check if this segment is in our tracking list
                            segment_key = (subject_name, segment_name)
                            if segment_key in self._segment_lookup:
                                seg_name = self._segment_lookup[segment_key]
                                # Store latest data for this segment
                                self._latest_data[seg_name] = {
                                    "topic": topic,
                                    "data": position_data,
                                    "timestamp": time.time(),
                                }

                        except json.JSONDecodeError as e:
                            logger.warning(f"[ViconOdometry] Error parsing JSON: {e}")
                    except zmq.Again:
                        # No message available, continue
                        continue

            except Exception as e:
                logger.warning(f"[ViconOdometry] Error in receive loop: {e}")
                time.sleep(0.01)

    def self_check(self):
        """Check if we can receive valid data for all segments"""
        logger.info("[ViconOdometry] Starting self-check...")
        for _ in range(20):
            self.update()
            if self._valid.get(self.robot_segment_name, False):
                logger.info(f"[ViconOdometry] self_check passed (robot segment data valid)")
                return True
            else:
                logger.warning("[ViconOdometry] self_check: data not valid, retry...")
            time.sleep(0.1)
        logger.error("[ViconOdometry] self_check failed: data not valid")
        return False

    def set_zero(self):
        """Set zero reference point for alignment (only for robot segment)"""
        if self.robot_segment_name not in self._valid or not self._valid[self.robot_segment_name]:
            logger.error("[ViconOdometry] set_zero failed: robot segment data not valid")
            return

        pos = self._pos[self.robot_segment_name].copy()
        quat = self._quat[self.robot_segment_name].copy()
        self.zero_align.set_base(quat, pos)
        logger.info(f"[ViconOdometry] Zero reference set for robot segment: {self.robot_segment_name}")

    def _smooth_quaternion(self, quat_new, quat_old, alpha):
        """
        Smooth quaternion using exponential moving average with proper quaternion handling.
        
        Args:
            quat_new: New quaternion [x, y, z, w]
            quat_old: Old quaternion [x, y, z, w]
            alpha: Smoothing factor (0-1), higher = less smoothing
        
        Returns:
            Smoothed quaternion [x, y, z, w]
        """
        # Ensure quaternions are normalized
        quat_new = quat_new / np.linalg.norm(quat_new)
        quat_old = quat_old / np.linalg.norm(quat_old)
        
        # Choose the quaternion with the same sign (closer on the unit sphere)
        if np.dot(quat_new, quat_old) < 0:
            quat_new = -quat_new
        
        # Exponential moving average
        quat_smooth = (1 - alpha) * quat_old + alpha * quat_new
        
        # Normalize
        quat_smooth = quat_smooth / np.linalg.norm(quat_smooth)
        
        return quat_smooth

    def _smooth_position(self, pos_new, pos_old, alpha):
        """
        Smooth position using exponential moving average.
        
        Args:
            pos_new: New position [x, y, z]
            pos_old: Old position [x, y, z]
            alpha: Smoothing factor (0-1), higher = less smoothing
        
        Returns:
            Smoothed position [x, y, z]
        """
        return (1 - alpha) * pos_old + alpha * pos_new

    def _update_segment(self, segment_name: str):
        """Update odometry data for a single segment"""
        if segment_name not in self._latest_data:
            self._valid[segment_name] = False
            # Reset smoothed values when data is invalid
            if segment_name in self._smoothed_pos:
                del self._smoothed_pos[segment_name]
            if segment_name in self._smoothed_quat:
                del self._smoothed_quat[segment_name]
            return

        position_data = self._latest_data[segment_name]["data"]
        current_time = self._latest_data[segment_name]["timestamp"]

        # Extract translation (position)
        translation = position_data.get("translation", None)
        if translation is None or len(translation) != 3:
            self._valid[segment_name] = False
            # Reset smoothed values when data is invalid
            if segment_name in self._smoothed_pos:
                del self._smoothed_pos[segment_name]
            if segment_name in self._smoothed_quat:
                del self._smoothed_quat[segment_name]
            logger.warning(f"[ViconOdometry] update: invalid translation data for {segment_name}")
            return

        # Extract rotation (quaternion)
        rotation = position_data.get("rotation", None)
        if rotation is None or len(rotation) != 4:
            self._valid[segment_name] = False
            # Reset smoothed values when data is invalid
            if segment_name in self._smoothed_pos:
                del self._smoothed_pos[segment_name]
            if segment_name in self._smoothed_quat:
                del self._smoothed_quat[segment_name]
            logger.warning(f"[ViconOdometry] update: invalid rotation data for {segment_name}")
            return

        # Convert to numpy arrays
        # Quaternions are in [x, y, z, w] format from Vicon
        # Position is in millimeters from Vicon, convert to meters
        _pos_raw = np.array(translation, dtype=np.float32) / 1000.0  # Convert mm to m
        _quat_raw = np.array(rotation, dtype=np.float32)

        # Normalize quaternion
        quat_mag = np.linalg.norm(_quat_raw)
        if quat_mag > 0:
            _quat_raw = _quat_raw / quat_mag

        # Apply zero alignment transformation to all segments when enabled
        # This puts them in the same coordinate frame as the robot (z=0 world frame)
        if self.cfg.zero_align:
            _pos_raw = self.zero_align.align_pos(_pos_raw)
            _quat_raw = self.zero_align.align_quat(_quat_raw)
        
        # # Apply smoothing
        # if segment_name in self._smoothed_pos and segment_name in self._smoothed_quat:
        #     # Apply smoothing
        #     _pos = self._smooth_position(_pos_raw, self._smoothed_pos[segment_name], self.cfg.smoothing_alpha_pos)
        #     _quat = self._smooth_quaternion(_quat_raw, self._smoothed_quat[segment_name], self.cfg.smoothing_alpha_quat)
        # else:
        #     # First update, no smoothing yet
        #     _pos = _pos_raw.copy()
        #     _quat = _quat_raw.copy()

        _pos = _pos_raw.copy()
        _quat = _quat_raw.copy()
        
        # Store smoothed values for next iteration
        self._smoothed_pos[segment_name] = _pos.copy()
        self._smoothed_quat[segment_name] = _quat.copy()
        
        # Apply position offset only for robot segment
        if segment_name == self.robot_segment_name:
            self._pos[segment_name] = _pos + np.array(self.cfg.pos_offset)
        else:
            self._pos[segment_name] = _pos
        
        self._quat[segment_name] = _quat

        # Calculate RPY from quaternion
        rot = sRot.from_quat(self._quat[segment_name])
        self._rpy[segment_name] = rot.as_euler("xyz")

        # Compute linear velocity using finite differencing (in world frame)
        if segment_name in self._last_pos and segment_name in self._last_time and self._last_time[segment_name] is not None:
            dt = current_time - self._last_time[segment_name]
            if dt > 0.0:
                # Compute velocity in world frame
                v_world = (self._pos[segment_name] - self._last_pos[segment_name]) / dt
                self._lin_vel[segment_name] = v_world.copy()
                # Transform velocity to body frame
                self._lin_vel_body[segment_name] = quat_rotate_inverse_np(self._quat[segment_name], v_world)
            else:
                # dt is zero or negative, keep previous velocity
                pass
        else:
            # First update, no velocity yet
            self._lin_vel[segment_name] = np.zeros(3, dtype=np.float32)
            self._lin_vel_body[segment_name] = np.zeros(3, dtype=np.float32)

        # Compute angular velocity using finite differencing
        if segment_name in self._last_quat and segment_name in self._last_time and self._last_time[segment_name] is not None:
            dt = current_time - self._last_time[segment_name]
            if dt > 0.0:
                # Compute relative rotation: q_rel = q_current * q_prev^-1
                rot_current = sRot.from_quat(self._quat[segment_name])
                rot_last = sRot.from_quat(self._last_quat[segment_name])
                rot_rel = rot_current * rot_last.inv()
                
                # Convert to rotation vector (axis-angle representation)
                # Angular velocity = rotation_vector / dt (in world frame)
                rot_vec = rot_rel.as_rotvec()
                self._ang_vel[segment_name] = rot_vec / dt
                # Transform to body frame
                self._ang_vel_body[segment_name] = quat_rotate_inverse_np(self._quat[segment_name], rot_vec / dt)
            else:
                # dt is zero or negative, keep previous velocity
                pass
        else:
            # First update, no angular velocity yet
            self._ang_vel[segment_name] = np.zeros(3, dtype=np.float32)
            self._ang_vel_body[segment_name] = np.zeros(3, dtype=np.float32)

        # Store current position, quaternion, and time for next update
        self._last_pos[segment_name] = self._pos[segment_name].copy()
        self._last_quat[segment_name] = self._quat[segment_name].copy()
        self._last_time[segment_name] = current_time

        self._valid[segment_name] = True

    def update(self):
        """Update odometry data from latest received Vicon data for all segments"""
        # Update all segments
        for subject_name, segment_name in self.segments:
            self._update_segment(segment_name)

    @property
    def is_valid(self):
        """Check if robot segment odometry data is valid (backward compatibility)"""
        return self._valid.get(self.robot_segment_name, False)
    
    def is_segment_valid(self, segment_name: str) -> bool:
        """Check if a specific segment's data is valid"""
        return self._valid.get(segment_name, False)

    @property
    def pos(self):
        """Get current position of robot segment (3D) - backward compatibility"""
        return self._pos[self.robot_segment_name].copy()
    
    def get_segment_pos(self, segment_name: str):
        """Get current position of a specific segment (3D)"""
        return self._pos[segment_name].copy()

    @property
    def quat(self):
        """Get current quaternion of robot segment [x, y, z, w] - backward compatibility"""
        return self._quat[self.robot_segment_name].copy()
    
    def get_segment_quat(self, segment_name: str):
        """Get current quaternion of a specific segment [x, y, z, w]"""
        return self._quat[segment_name].copy()

    @property
    def rpy(self):
        """Get current roll-pitch-yaw angles of robot segment - backward compatibility"""
        return self._rpy[self.robot_segment_name].copy()
    
    def get_segment_rpy(self, segment_name: str):
        """Get current roll-pitch-yaw angles of a specific segment"""
        return self._rpy[segment_name].copy()

    @property
    def lin_vel(self):
        """Get current linear velocity of robot segment in body frame - backward compatibility"""
        return self._lin_vel_body[self.robot_segment_name].copy()
    
    def get_segment_lin_vel(self, segment_name: str, world_frame: bool = False):
        """Get current linear velocity of a specific segment
        Args:
            segment_name: Name of the segment
            world_frame: If True, return in world frame; if False, return in body frame
        """
        if world_frame:
            return self._lin_vel[segment_name].copy()
        else:
            return self._lin_vel_body[segment_name].copy()

    @property
    def ang_vel(self):
        """Get current angular velocity of robot segment in body frame - backward compatibility"""
        return self._ang_vel_body[self.robot_segment_name].copy()
    
    def get_segment_ang_vel(self, segment_name: str, world_frame: bool = False):
        """Get current angular velocity of a specific segment
        Args:
            segment_name: Name of the segment
            world_frame: If True, return in world frame; if False, return in body frame
        """
        if world_frame:
            return self._ang_vel[segment_name].copy()
        else:
            return self._ang_vel_body[segment_name].copy()

    def get_all_segments_data(self):
        """Get all segment data in extras format (segment_name/base_*_w)
        Returns a dictionary with keys like "segment_name/base_pos_w", "segment_name/base_quat", etc.
        """
        data = {}
        for subject_name, segment_name in self.segments:
            if self._valid.get(segment_name, False):
                data[f"{segment_name}/base_pos_w"] = self._pos[segment_name].copy()
                data[f"{segment_name}/base_quat"] = self._quat[segment_name].copy()
                data[f"{segment_name}/base_lin_vel_w"] = self._lin_vel[segment_name].copy()
                data[f"{segment_name}/base_ang_vel_w"] = self._ang_vel[segment_name].copy()
        return data

    @property
    def latest_data(self):
        """Get latest raw Vicon data for robot segment - backward compatibility"""
        return self._latest_data.get(self.robot_segment_name, {}).copy() if self.robot_segment_name in self._latest_data else None
    
    def get_segment_latest_data(self, segment_name: str):
        """Get latest raw Vicon data for a specific segment"""
        return self._latest_data.get(segment_name, {}).copy() if segment_name in self._latest_data else None

    def __del__(self):
        """Cleanup on deletion"""
        self._stop_event.set()
        if hasattr(self, "socket"):
            self.socket.close()
        if hasattr(self, "context"):
            self.context.term()


if __name__ == "__main__":
    # Example usage with multiple segments
    cfg = ViconOdometryCfg(
        zmq_address="tcp://localhost:5555",
        topic_filter="",
        segments=[("G1", "G1"), ("largebox", "largebox")],
        robot_segment=("G1", "G1"),
        zero_align=True,
    )
    vicon_odom = ViconOdometry(cfg)

    while True:
        vicon_odom.update()
        if vicon_odom.is_valid:
            print(f"Robot pos: {vicon_odom.pos}, rpy: {vicon_odom.rpy}")
            all_data = vicon_odom.get_all_segments_data()
            for key, value in all_data.items():
                print(f"  {key}: {value}")
        else:
            print("data not valid")
        time.sleep(0.1)
