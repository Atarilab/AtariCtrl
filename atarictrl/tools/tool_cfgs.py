from pydantic import computed_field, model_validator

from atarictrl.config import Config


class ForwardKinematicCfg(Config):
    xml_path: str
    debug_viz: bool = False
    kinematic_joint_names: list[str] | None = None


class ZedOdometryCfg(Config):
    server_ip: str
    pos_offset: list[float] = [0.0, 0.0, 0.8]  # x,y,z

    zero_align: bool = True


class ViconOdometryCfg(Config):
    zmq_address: str = "tcp://localhost:5555"
    """ZMQ publisher address"""
    
    topic_filter: str = ""
    """Topic filter prefix (empty string subscribes to all topics, default: "")"""
    
    # Legacy single segment support (deprecated, use segments instead)
    subject_name: str | None = None
    """Subject name to track (e.g., 'Box', 'G1'). Deprecated: use segments instead"""
    
    segment_name: str | None = None
    """Segment name to track (e.g., 'Box', 'G1'). Deprecated: use segments instead"""
    
    # New multi-segment support
    segments: list[tuple[str, str]] | None = None
    """List of (subject_name, segment_name) tuples to track. 
    Example: [("G1", "G1"), ("Box", "Box")]"""
    
    robot_segment: tuple[str, str] | None = None
    """(subject_name, segment_name) tuple for the robot segment. 
    Zero alignment is only applied to this segment. 
    Other segments' positions will be relative to this segment's zero frame.
    Example: ("G1", "G1")"""
    
    pos_offset: list[float] = [0.0, 0.0, 0.0]  # x,y,z
    """Position offset to apply to robot segment"""
    
    zero_align: bool = True
    """Whether to align robot segment to initial position/orientation (z=0 world frame)"""
    
    smoothing_alpha_pos: float = 0.5
    """Smoothing factor for position (0-1, higher = less smoothing, more responsive). Default: 0.3"""
    
    smoothing_alpha_quat: float = 0.5
    """Smoothing factor for quaternion (0-1, higher = less smoothing, more responsive). Default: 0.2"""
    
    save_data: bool = False
    """Whether to save Vicon data to file"""
    
    log_path: str = "logs/vicon"
    """Path to save Vicon data logs"""


class DoFConfig(Config):
    _subset: bool = False  # if True, simplely inheritance & pick
    _subset_joint_names: list[str] | None = None  # used when subset is True

    joint_names: list[str]
    default_pos: list[float] | None = None
    stiffness: list[float] | None = None
    damping: list[float] | None = None
    torque_limits: list[float] | None = None
    position_limits: list[list[float]] | None = None  # [[min, max], ...]

    @computed_field
    @property
    def num_dofs(self) -> int:
        return len(self.joint_names)

    @property
    def prop_keys(self) -> list[str]:
        prop_keys = list(DoFConfig.model_fields.keys())
        prop_keys_valid = []
        # remove prop that is None
        for prop_key in prop_keys:
            if getattr(self, prop_key) is not None:
                prop_keys_valid.append(prop_key)
        return prop_keys_valid

    @model_validator(mode="after")
    def check_dof_and_process_subset(self):
        length = self.num_dofs
        if self.default_pos is not None and len(self.default_pos) != length:
            raise ValueError(f"default_pos length {len(self.default_pos)} does not match num_dofs {length}")
        if self.stiffness is not None and len(self.stiffness) != length:
            raise ValueError(f"stiffness length {len(self.stiffness)} does not match num_dofs {length}")
        if self.damping is not None and len(self.damping) != length:
            raise ValueError(f"damping length {len(self.damping)} does not match num_dofs {length}")
        if self.torque_limits is not None and len(self.torque_limits) != length:
            raise ValueError(f"torque_limits length {len(self.torque_limits)} does not match num_dofs {length}")
        if self.position_limits is not None:
            if len(self.position_limits) != length:
                raise ValueError(f"position_limits length {len(self.position_limits)} does not match num_dofs {length}")
            for i, limits in enumerate(self.position_limits):
                if len(limits) != 2:
                    raise ValueError(f"position_limits[{i}] length {len(limits)} is not 2")
                if limits[0] >= limits[1]:
                    raise ValueError(f"position_limits[{i}] min {limits[0]} is not less than max {limits[1]}")

        # check subset
        if self._subset:
            if self._subset_joint_names is None or len(self._subset_joint_names) == 0:
                raise ValueError("_subset_joint_names must be provided and non-empty when _subset is True")
            for name in self._subset_joint_names:
                if name not in self.joint_names:
                    raise ValueError(f"_subset_joint_name {name} not in joint_names")

            # do the subset operation
            joint_names = self.joint_names
            subset_joint_names = self._subset_joint_names
            # print(f"[DoFConfig]processing subset joint props from {len(joint_names)} to {len(subset_joint_names)}...")
            from .dof import DoFAdapter

            dof_adapter = DoFAdapter(src_joint_names=joint_names, tar_joint_names=subset_joint_names)

            for prop_key in self.prop_keys:
                prop_val = getattr(self, prop_key)
                subset_value = dof_adapter.fit(data=prop_val, dim=0).tolist()
                assert len(subset_value) == len(subset_joint_names)
                value = subset_value
                setattr(self, prop_key, value)
        # print(f"({self.__class__.__name__}) {self.prop_keys}")
        return self
