<div align="center">
    <summary>
      <h1>Safe Reinforcement Learning via Hamilton-Jacobi Safety Filter with Turtlebot 3 Burger</h1>
      <br>
    </summary>
</div>

# Overview

This repository provides a framework for **safe reinforcement learning** using Hamilton-Jacobi (HJ) reachability-based control barrier functions (HJ-CBFs). It supports simulated dynamics, Gazebo, and real world environments. 

We used the industry standard [Robot Operating System 2 (ROS 2) Humble](https://docs.ros.org/en/humble/index.html) as our framework for interacting with the robot in Gazebo and real world training.

# Safety Filtering Formulation

General form of a safety filter with a monitor $V(x)$ and controller $\pi_{\text{safe}}(x)$
```math
\begin{align*}
    \phi(x, \pi_\theta(x)) :=
    \begin{cases}
        \pi_\theta(x) & V(x) \geq \epsilon \\
        \pi_{\text{safe}}(x) & \text{otherwise}
    \end{cases} \quad \text{(Least-Restrictive Safety Filter)}
\end{align*}
```
The monitor and the controller can be customized to make the safe control deviate smoothly from the nominal control and adapt to dynamics model mismatch.

# Installation and Setup

## Overview
Our project is a stand-alone package inside a standard ROS 2 workspace. Specifically, it should be residing in the `src` folder so the Colcon build system can operate properly. Furthermore, most commands should be run inside the top-level ROS 2 workspace directory.

ROS 2 is incompatible with Conda, so we opted to use `pip` as our package manager. We highly recommend using a Docker container, so the environment is sandboxed. One option is [MARS Lab container](https://github.com/SFU-MARS/ros2_ws). 

## Dependencies
We used ROS 2 Humble on Ubuntu 22 operating system inside a docker environment. Please follow the [official documentation](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) to install ROS 2

Additionally, to install other dependencies using `pip`
```
pip install -r new_req.txt
```

## Build and Install
```python
colcon build --symlink-install --packages-select safe_rl_py
source install/setup.bash
```

## Computing Backward Reachable Tubes (BRTs)
BRT Computations requires the OptimizedDP library (https://github.com/SFU-MARS/optimized_dp). Please follow the instructions inside the repository.

```
conda activate odp
pip install -e .
pip install gymnasium
python3 redexp/brts/dubins_3d.py
python3 redexp/brts/turtlebot_brt.py
```

# Training
Please note that for Gazebo and the real word, the training requires a map of the environment and the BRT to be pre-computed. These should be loaded appropriately in the config files and in the code.

Save the folder name where this project was saved
```
export SAFE_RL_FOLDER="adaptive-hj-safety-ros2" # could be anything, e.g. "safe_rl_py"
```

## Dynamics Simulation Training (Dubins3D)
```
python3 src/$SAFE_RL_FOLDER/train/train_sac_lag.py \
    --config src/$SAFE_RL_FOLDER/train/droq_config.py \
    --env_name=Safe-Dubins3d-NoModelMismatch-v1 \
    --cbf \
    --cbf_gamma=1.0 \
    --max_steps=10000 \
    --seed=0
```

## Gazebo with True State Simulation Training
In one terminal
```
ros2 launch safe_rl_py turtlebot_sim.launch.py
```

In another terminal
```
python3 src/$SAFE_RL_FOLDER/train/train_ros.py \
    --config src/$SAFE_RL_FOLDER/train/droq_config.py \
    --env_name=Turtlebot3BgEnvGazebo-ModelMismatch-v1 \
    --cbf \
    --cbf_gamma=1.0 \
    --max_steps=10000 \
    --seed=0
```

## Monte Carlo Localization Training (Gazebo and Real World)
In one terminal
```
# Choose the launch file depending on your target environment
# i.e. Gazebo vs the real world
# ros2 launch safe_rl_py turtlebot_sim.launch.py
ros2 launch safe_rl_py turtlebot_real.launch.py
```

In another terminal
```
python3 src/$SAFE_RL_FOLDER/train/train_ros.py \
    --config src/$SAFE_RL_FOLDER/train/droq_config.py \
    --env_name=Turtlebot3BgEnvAmcl-ModelMismatch-v1 \
    --cbf \
    --cbf_gamma=1.0 \
    --max_steps=10000 \
    --seed=0
```

On the Turtlebot 3 (if in the real world)
```
ros2 launch turtlebot3_bringup robot.launch.py
```
For more information, please checkout [Turtlebot 3 bringup documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup)

# Real World Training Video

## Turtlebot 3 Burger with Robust HJ-CBF Shielding Method
<div align=center>
<a href="https://youtu.be/mjOnphvu6WY">
  <img src="https://markdown-videos-api.jorgenkh.no/url?url=https%3A%2F%2Fyoutu.be%2FmjOnphvu6WY" alt="Robust HJ-CBF Shielding Method" title="Robjust HJ-CBF Shielding Method"/>
</a></div>

# Attribution and Prior Work

This project is adapted from and builds upon:

- Michael Lu et al., *Safe Learning in the Real World via Adaptive Shielding with Hamilton-Jacobi Reachability* [[link](https://github.com/sudo-michael/robust-hj-cbf-safe-rl)]
- Aaron Laitner, *ROS2-CMPT416-Fall2025* [[link](https://github.com/aaronlaitner/ROS2-CMPT416-Fall2025)]

All original licensing terms are preserved.
