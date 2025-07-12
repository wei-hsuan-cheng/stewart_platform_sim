# stewart_platform_sim

ROS 2 Python simulation of a six-DoF Stewart platform (or Stewart-Gough platform), which is a parallel manipulator comprising a fixed base platform, a moving end-effector platform, and six serial SPS sturctures.

The inverse kinematic (IK) and differential kinematics (DK) problems are solved as to obtain the required leg *lengths* and *velocities* for driving the robot to achieve a desired motion in $SE(3)$.

The map for IK, $H_{\mathrm{ik}}:\ SE(3) \to \mathbb{R}^6$ is provided, where 
- $\boldsymbol{s} = H_{\mathrm{ik}}(^b\boldsymbol{\xi}_e)$
- $\boldsymbol{s}$ is the leg length
- $^b\boldsymbol{\xi}_e$ is the end-effector pose w.r.t. base.

The Jacobian for, DK $J_b:\ se(3) \to \mathbb{R}^6$ is also provided, where
- $\dot{\boldsymbol{s}} = J_b \cdot \mathcal{V}_b = J_b \cdot \mathrm{Ad}(^b\boldsymbol{\xi}_e) \cdot \mathcal{V}_e$
- $\dot{\boldsymbol{s}}$ is the leg velocity
- $J_b$ and $\mathcal{V}_b$ are the base Jacobian and twist, respectively
- $\mathcal{V}_e$ is the end-effector twist (spatial velocity) that can be transformed into $\mathcal{V}_b$ through the adjoint map $\mathrm{Ad}(^b\boldsymbol{\xi}_e)$.

## Simulation (visualised in ROS 2 RViz2)
<img src="assets/stewart_platform_sim.gif" alt="Stewart Platform Sim Demo" width="400"/>

## Demo
Run the simulation in ros2:
```bash
# Simulation in ros2 and visualise in rivz2
cd ~/stewart_platform_sim && python3 sim_ros2.py
rviz2
```