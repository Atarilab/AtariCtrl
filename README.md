<div align="center">
<h1>ATARICtrl 🤖</h1>

*A plug-and-play deploy framework for robots. Just deploy, just do.*

<p>
  <!-- Platforms -->
  <img src="https://img.shields.io/badge/platform-Ubuntu%20%7C%20Linux-green" alt="platform"/>
  <!-- Multi-Robot -->
  <img src="https://img.shields.io/badge/robot-UnitreeG1-orange" alt="multi-robot"/>
  <!-- Pre Commit -->
  <img src="https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white]" alt="pre-commit"/>
  <!-- License -->
  <a href="https://creativecommons.org/licenses/by-nc/4.0/">
    <img src="https://img.shields.io/badge/License-CC--BY--NC--4.0-lightgrey.svg" alt="license"/>
  </a>
</p>
</div>


Our framework highlights:
- **Out-of-the-box**: After setting up ATARICtrl, multiple policies can be deployed on both simulation and real robots in minutes: [Quick Start](#quick-start).

- **Decoupled & Modular Design**: With a Python-first design, ATARICtrl makes fast prototyping easy. Environment, Controller, and Policy are modular and freely composable, while minimal code changes allow seamless adaptation across robots and setups: See how we achieve this: [Add a new module](#add-a-new-module).

- **Multi-policy switching**: Seamlessly switch between different policies during a task. Try this: [Multi-Policy Switching](#multi-policy-switch).

- **Light-Weight**: Our framework is lightweight, after 5 minutes of setup, it runs smoothly onboard. By [UnitreeCpp](https://github.com/Atarilab/unitree_cpp), ATARICtrl runs on Unitree G1 without the need for an Ethernet cable.


# 📓Content
 - [📄Introduction](#introduction)
 - [🛠️Easy Setup](#%EF%B8%8Feasy-setup)
 - [📖Quick Start](#quick-start)
 - [🧩 Develop and Contribute](#develop-and-contribute)


# 🗺️Roadmap

<table>
<tr>
<td width="80%">

- [x] [2025.04] Initialized project
- [x] [2025.05] Add support for Unitree G1
- [x] [2025.06] Integrated Unitree C++ SDK
- [x] [2025.08] Add support for beyondmimic
- [x] Add policy-switch pipeline with interpolation, check [LocoMimic Example](#loco-mimic-policy-switch-with-interpolation)!
- [ ] Upcoming policies...

 

</td>
<td width="20%">

<div align="center">
<img src="docs\images\job.gif" alt="working" width="100%" >
</div>

</td>
</table>

# 📄Introduction

This repository provides a deployment framework for humanoid robots, supporting the use of different policies across different environments (real robots and simulation).  
We decouple the **controller**, **environment**, and **policy**, making it easy for users to add their own policies or environments.  
Experiment configurations can be organized through config files.

The main modules of **ATARICtrl** consist of:

- 🎮 **Controller**: A collection of control signals. It receives external inputs (e.g., joystick, keyboard, motion sequences) and forwards them as `ctrl_data` to the pipeline.  
- 🤖 **Environment**: The execution environment (e.g., Mujoco, real robot). It processes actions provided by the policy and sends real-time sensor data as `env_data` to the pipeline.  
- 🌐 **Policy**: A trained control policy (from various wbc & locomotion works). It generates actions based on information from both the environment and the controller.

Currently, **ATARICtrl** supports the following policy–environment combinations:


<div align="center">

| Policy | Unitree G1 | Ref | Doc |
|:-------:|:--------:|:-------:|:-------:|
| Unitree Official | 🖥️ 🤖 | [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) | [UnitreePolicy](./docs/policy.md/#policy--unitreepolicy)| 
| Unitree Wo Gait | 🖥️ 🤖 | [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_gym) | [UnitreeWoGaitPolicy](./docs/policy.md/#unitreewogaitpolicy)|
| Smooth | 🖥️ 🤖 | [Smooth](https://github.com/zixuan417/smooth-humanoid-locomotion) |
| **BeyondMimic** | 🖥️ 🤖 | [whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking) | [BeyondmimicPolicy](./docs/policy.md/#policy--beyondmimicpolicy) |
| **BeyondMimicObject** | 🖥️ 🤖 | [whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking) | [BeyondMimicObjectPolicy](./docs/policy.md/#policy--beyondmimicobjectpolicy) | FFTAI
</div>

🖥️ means policy is ready for simulation, while 🤖 means policy has been tested on real robot.




# 🛠️Easy Setup

ATARICtrl supports **Ubuntu/Linux** platforms, officially tested on **Ubuntu**. 

Robot onboard PCs running Linux are also supported.


## 1️⃣ Basic Installation

**Step 1: Clone the repository and create a Python environment**

```bash
git clone https://github.com/Atarilab/ATARICtrl.git
cd ATARICtrl/

# Install uv if you haven't already
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**Step 2: Create a virtual environment and install AtariCtrl**

```bash
# Create virtual environment with Python 3.11
uv venv --python 3.11
source .venv/bin/activate 

# Optional, install cpu version for speed up
uv pip install torch --index-url https://download.pytorch.org/whl/cpu

# Install AtariCtrl
uv pip install -e .
```

## 2️⃣ Install Optional Modules

ATARICtrl is a **plug-and-play framework**. After a minimal default installation, you can selectively configure and install only the modules you need.

---

**Step 0: \[Optional\] Install Robot SDK**

> *You can skip this for sim2sim and development.*

If you plan to control a real robot, install the corresponding SDK.

For example, see [unitree_setup.md](docs/unitree_setup.md) for Unitree robots.

---

**Step 1: Configure modules**

Edit [submodule_cfg.yaml](./submodule_cfg.yaml) to select modules, by setting `install` as `true`.

> As default, `mujoco_viewer` is selected for sim2sim.

**Step 2: Install modules**

```bash
# Install all required modules
uv run python submodule_install.py

# Or specify modules to install with args
# uv run python submodule_install.py unitree_cpp
```

# 📖Quick Start

`ATARICtrl` is a modular framework where tasks can be flexibly defined by composing configuration files.  
In the following, we use the deployment on G1 as an example.

<!-- 😎For module combinations, we provide ready-to-use config files that can be directly applied.  -->
1. [Run Sim2Sim](#run-atarictrl-on-simulation)
2. [Run Sim2Real](#run-atarictrl-on-real-robot-🤖)
3. [Deploy More Policies✨](#deploy-more-policies)

## Run ATARICtrl on Simulation

Begin your journey with unitree g1 sim2sim.

> A Xbox controller is needed for control.

```bash
# run the default g1 sim2sim cfg
uv run python scripts/run_pipeline.py
```

You can control the motivation using any Xbox controller:

- `left axes` move forward/backward/lfet/right
- `right axes` turn left/right

<!-- Or a keyboard:

- `wsad` move forward/backward/left/right
- `qe` turn left/right -->

<!-- For cooler policy, run:

```bash
uv run python scripts/run_pipeline.py -c g1_beyondmimic
```
You can control the simulation environment using the Keyboard:

- `shift + <` start the motion play
- `shift + >` pause the motion play
- `shift + |` reset the motion progress
- `~` button: reset robot. -->

## Run ATARICtrl on Real Robot 🤖

### Alert & Disclaimer ⚠️⚠️⚠️
> Before deployment, you'd better first purchase accident insurance to cover any potential incidents that may occur during real-world operation. Policies could cause ⚠️**violent motions**⚠️ when losing balance. Always verify that the emergency stop button (e.g., **A** for default config) works properly.

> Unless you have strong sim-to-real expertise and rigorous safety measures, **DO NOT run these models on real robots**. They are provided for research only, and we disclaim any responsibility for harm, loss, or malfunction.

### Robot Setup

Follow our [setup guide](./docs/unitree_setup.md) to set up the robot sdk on your computer or robot.

### Start ATARICtrl

Open [`g1_cfg.py`](atarictrl/config/g1/g1_cfg.py) and modify the `g1_real` config.

Edit the `env_type` and `net_if` according to your robot sdk setup.

```python
class g1_real(g1):
    env: G1RealEnvCfg = G1RealEnvCfg(
        env_type="UnitreeEnv",  # For unitree_sdk2py
        # env_type="UnitreeCppEnv",  # For unitree_cpp, check README for more details
        unitree=G1UnitreeCfg(
            net_if="eth0",  # note: change to your network interface
        ),
    )
```

Refer to [official guide](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/README.md#startup-process) to prepare and start the robot.

Then start the pipeline on the real robot:

```bash
uv run python scripts/run_pipeline.py -c g1_real
```

Your robot should move into default pos. 
**During the preparation, put your robot on the ground.**

You can control the real robot using the Unitree controller:
- `A` button: Emergency stop. The robot immediately switches to damping mode. Be careful.
- `left axes` move forward/backward/let/right
- `right axes` turn left/right

### Using Vicon Odometry

ATARICtrl supports Vicon motion capture system for precise robot/objects localization. When Vicon is streaming from the Vicon PC, you can listen to it using the Vicon ZMQ client:

```bash
./packages/vicon_zmq_publisher/build/vicon_zmq_client --hostname <host-ip-addr>
```

Replace `<host-ip-addr>` with the IP address of the Vicon PC.

To use Vicon Odometry in your configuration, set `odometry_type="VICON"` and configure `vicon_cfg` in your environment config. See [`g1_real_vicon`](atarictrl/config/g1/g1_custom_cfg.py) for an example configuration.

## Deploy More Policies

💡Now you’re familiar with ATARICtrl’s config design, it’s time to experience the **amazing variety of policies**!

### BeyondMimic

Try the out of box experience of **BeyondMimic**:

```bash
uv run python scripts/run_pipeline.py -c g1_beyondmimic
```

check documentation [BeyondmimicPolicy](./docs/policy.md/#policy--beyondmimicpolicy) for more details.

### Multi-Policy Switch
`g1_switch` config in [g1_cfg.py](atarictrl/config/g1/g1_cfg.py) is equipped with Multi-Policy Pipeline.

```bash
uv run python scripts/run_pipeline.py -c g1_switch
```

Xbox Controller:

- `left axes` move forward/backward/left/right
<!-- - `right axes(for/back)` stand higher/squat -->
- `right axes(left/right)` turn left/right

Switch between different policies using the controller.

### Loco-Mimic Policy Switch with Interpolation

For deploying **Motion Mimic Policies** with **Locomotion** as backup, we built [LocoMimicPipeline](atarictrl/pipeline/rl_loco_mimic_pipeline.py) for multi-policy switching with interpolation, 

Check `g1_locomimic` config in [g1_cfg.py](atarictrl/config/g1/g1_cfg.py), and more fancy locomimic configs in [g1_loco_mimic_cfg.py](atarictrl/config/g1/g1_loco_mimic_cfg.py).

```bash
uv run python scripts/run_pipeline.py -c g1_locomimic_beyondmimic
```

Keyboard control:
- `[` to switch to MotionMimic
- `]` to switch to LocoMotion
- `;` toggle next mimic policy
- `'` toggle prev mimic policy

<div align="center">
<img src="docs/images/locomimic_asap.gif" width="20%" alt="locomimic_asap"/>
</div>

### More Policies

We also provide config files for other policies, check [config_g1](atarictrl/config/g1) for more details.


# 🧩Develop and Contribute

## Add a new module

Refer to the documentation on [Policy](docs/policy.md), [Controller](docs/controller.md), [Env](docs/environment.md), create and deploy your own policy in minutes.

(By the way, deploying GMT takes about 1 hour in our framework.)

Or simply create an issue — we will include updates in future releases!


# 🔗 Related Repo

- [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python): used for implementing `UnitreeEnv`.
- [PHC](https://github.com/ZhengyiLuo/PHC): used for implementing the `MotionCtrl` module for OmniH2O.
- [UnitreeCpp](https://github.com/GDDG08/unitree_cpp): our pybind of `unitree_sdk2` used in `UnitreeCppEnv`.
- [ZED Proxy](https://github.com/GDDG08/ZED-Proxy/): ZED Camera Odometry Service.