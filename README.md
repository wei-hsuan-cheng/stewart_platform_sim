# stewart_platform_sim

ROS 2 Python simulation of a six-DoF Stewart platform (or Stewart-Gough platform), which is a parallel manipulator comprising a fixed base platform, a moving end-effector platform, and six serial SPS sturctures.

## Simulation (visualised in ROS 2 RViz2)
<img src="assets/stewart_platform_sim.gif" alt="Stewart Platform Sim Demo" width="400"/>

## Demo
Run the simulation in ros2:
```bash
# Simulation in ros2 and visualise in rivz2
cd ~/stewart_platform_sim && python3 sim_ros2.py
rviz2
```