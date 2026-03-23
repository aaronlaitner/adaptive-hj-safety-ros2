<div align="center">
    <summary>
      <h1>Adaptive Hamilton-Jacobi Safety Filtering with Learned Dynamics for Turtlebot Systems</h1>
      <br>
    </summary>
</div>

# Overview

This repository provides a simulation framework for safe reinforcement learning using Hamilton-Jacobi (HJ) reachability-based control barrier functions (HJ-CBFs)

This codebase is **adapted from and builds upon**
https://github.com/sudo-michael/robust-hj-cbf-safe-rl
https://github.com/aaronlaitner/ROS2-CMPT416-Fall2025
with modifications to enable:

- TODO

## Current Capabilities:

- Dubins3D navigation in simulation
- SAC-LAG reinforcement learning
- Online HJ-CBF safety filtering
- Precomputed Backward Reachable Tubes (BRTs)
- Single-regime Dubins3D experiments
- Structured to support future ROS2 Turtlebot integration but real-world Turtlebot deployment is not yet enabled

# Safety Filtering Formulation

A learned policy is filtered online using the HJ value function:
```math
\begin{align*}
    \phi(x, \pi_\theta(x)) :=
    \begin{cases}
        \pi_\theta(x) & V(x) \geq \epsilon \\
        \pi_{\text{safe}}(x) & \text{otherwise}
    \end{cases} \quad \text{(Least-Restrictive Safety Filter)}
\end{align*}
```

# Installation
```
conda env create -f environment.yml 
conda activate rhj
pip install -e .
```
## Computing Backward Reachable Tubes (BRTs)
BRT Computations requires the OptimizedDP library (https://github.com/SFU-MARS/optimized_dp)

```
conda activate odp
pip install -e .
pip install gymnasium
python3 redexp/brts/dubins_3d.py
python3 redexp/brts/turtlebot_brt.py
```

# Simulation Training (Dubins3D)
To run SAC-LAG training with optional HJ-CBF safety filtering:
```
python train/train_sac_lag.py \
    --config train/droq_config.py \
    --env_name=Safe-Dubins3d-NoModelMismatch-v1 \
    --cbf \
    --cbf_gamma=1.0 \
    --max_steps=10000 \
    --seed=0
```

# Simulation Training (Turtlebot - Gazebo)
```
python3 train/train_ros.py \
    --config train/droq_config.py \
    --env_name=TurtlebotEnvGazebo-ModelMismatch-v1 \
    --cbf \
    --cbf_gamma=1.0 \
    --max_steps=10000 \
    --seed=0
```

## Attribution and Prior Work

This project is adapted from and builds upon:

- Michael Lu et al., *Safe Learning in the Real World via Adaptive Shielding with Hamilton-Jacobi Reachability*  
- https://github.com/sudo-michael/robust-hj-cbf-safe-rl
- Aaron Laitner, *ROS2-CMPT416-Fall2025*
- https://github.com/aaronlaitner/ROS2-CMPT416-Fall2025

Major modifications include:
- TODO

All original licensing terms are preserved.
