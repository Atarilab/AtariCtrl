# Policy

**Policy** is the component that controls the robot. It receives the `env_data` from the environment, `ctrl_data` from the controller, organize the observation and infer the action fo robot.

## [Policy](#policy)

`Policy` is the base class for all policies. It defines the interface for the policy, as in [base_policy.py](../atarictrl/policy/base_policy.py)

---

We provide the following policies:
- [UnitreePolicy](#policy--unitreepolicy)
- [AMOPolicy](#policy--amopolicy)
- [H2HStudentPolicy](#policy--h2hstudentpolicy)
- [HugWBCPolicy](#policy--hugwbcpolicy)
- [BeyondMimicPolicy](#policy--beyondmimicpolicy)
- [ASAPPolicy](#policy--asappolicy)

## [Policy](#policy) > [UnitreePolicy](#policy--unitreepolicy)

`UnitreePolicy` is the policy that controls the robot using the [Unitree official policy](https://github.com/unitreerobotics/unitree_rl_gym). It is a subclass of `Policy` and implements the interface defined in `Policy`.

script: [unitree_policy.py](../atarictrl/policy/unitree_policy.py)

To control the robot using `UnitreePolicy`, you can refer `_get_commands()`:

`commands`:
- `commands[0]`, [-1, 1], control the robot to walk forward and backward
- `commands[1]`, [-1, 1], control the robot to walk left and right
- `commands[2]`, [-1, 1], control the robot to turn left and right

for instance, use `JoystickCtrl` to control:

```python
def _get_commands(self, ctrl_data: dict) -> list[float]:
    commands = np.zeros(3)
    for key in ctrl_data.keys():
        if key in ["JoystickCtrl", "UnitreeCtrl"]:
            axes = ctrl_data[key]["axes"]
            lx, ly, rx, ry = axes["LeftX"], axes["LeftY"], axes["RightX"], axes["RightY"]

            commands[0] = command_remap(ly, self.commands_map[0])
            commands[1] = command_remap(lx, self.commands_map[1])
            commands[2] = command_remap(rx, self.commands_map[2])
            break
    return commands
```

### [UnitreeWoGaitPolicy](#policy--unitreewogaitpolicy)

For Unitree G1, we also provide `UnitreeWoGaitPolicy`, which supports the new `Unitree-G1-29dof-Velocity` conig from [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab).

The difference is that `UnitreeWoGaitPolicy` does not include gait in the observation, so the robot will not keep stepping when standing.

script: [unitree_policy.py](../atarictrl/policy/unitree_policy.py)

## [Policy](#policy) > [AMOPolicy](#policy--amopolicy)

`AMOPolicy` is the policy that controls the robot using the [AMO](https://github.com/OpenTeleVision/AMO). It is a subclass of `Policy` and implements the interface defined in `Policy`.

script: [amo_policy.py](../atarictrl/policy/amo_policy.py)

To control the robot using `AMOPolicy`, you can refer `_get_commands()`:

`commands`:
- `commands[0]`, [-1, 1], control the robot to walk forward and backward
- `commands[1]`, [-1, 1], control the robot to turn left and right
- `commands[2]`, [-1, 1], control the robot to walk left and right
- `commands[3]`, [-0.5, 0.8], control the robot torso height
- `commands[4]`, [-1.57, 1.57], control the robot torso yaw
- `commands[5]`, [-0.52, 1.57], control the robot torso pitch
- `commands[6]`, [-0.7, 0.7], control the robot torso roll

You can apply your own controller to control the robot using `AMOPolicy`. Just set the `commands` in `_get_comands()`



## [Policy](#policy) > [BeyondMimicPolicy](#policy--beyondmimicpolicy)

`BeyondMimicPolicy` is the policy that controls the robot using the [whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking). It is a subclass of `Policy` and implements the interface defined in `Policy`.

We support both `G1FlatEnvCfg` and `G1FlatWoStateEstimationEnvCfg`. 
For motion source, you could use the motion inside onnx policy, or use `BeyondmimicCtrl` with npz files.

script: [beyondmimic_policy.py](../atarictrl/policy/beyondmimic_policy.py)

[`BeyondMimicPolicyCfg`](../atarictrl/policy/policy_cfgs.py): check example at [G1BeyondMimicPolicyCfg](../atarictrl/config/g1/policy/g1_beyondmimic_policy_cfg.py):
 - `policy_name`: The name of the policy. We provive `Jump_wose` for test. You should put your policy in `assets/models/g1/beyondmimic`
 - `without_state_estimator`: Weather policy is `WoStateEstimation`. Default is `True`.
 - `use_modelmeta_config`: Whether to use modelmeta config. Default is `True`. If `False`, the policy will use config in your `BeyondMimicPolicyCfg`.
 - `use_motion_from_model`: Whether to use motion in the onnx model. Default is `True`. If `False`, you need to enable `BeyondMimicCtrl`.
.

 You can refer to `g1_beyondmimic` and `g1_beyondmimic_with_ctrl` in [g1_cfg.py](../atarictrl/config/g1/g1_cfg.py) for details.
