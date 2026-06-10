# 🤖 Master-Slave 3-DOF Arm (ROS 2 & Gazebo Harmonic)

This repository contains a ROS 2 package (`zayan`) that simulates a 3-DOF robotic arm with a gripper operating in a master-slave configuration.

## 📖 Overview

The project demonstrates **Analytic Inverse Kinematics (IK)** for robotic manipulation. The user controls a virtual "Master" arm using a GUI slider interface. The node calculates the physical end-effector coordinates in 3D space, computes the inverse kinematics, and commands a simulated "Slave" arm in Gazebo to dynamically track and match the master's position.

## 📦 Dependencies

This package is built for **ROS 2** and requires **Gazebo Harmonic**. The following ROS 2 packages are required:
- `urdf` and `xacro`
- `robot_state_publisher` and `joint_state_publisher_gui`
- `ros_gz_sim` and `ros_gz_bridge`
- `gz_ros2_control` and `ros2controlcli`
- `controller_manager` and `position_controllers`

## 🚀 How to Build

1. Clone this package into your ROS 2 workspace (e.g., `~/master_slave_ws/`) (NOTE: you can use the repo itself as the workspace).
2. Navigate to your workspace root:
   ```bash
   cd ~/master_slave_ws
   ```
3. Source your ROS 2 installation (e.g., Humble or Jazzy):
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```
4. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```

## 🎮 How to Run

After building, open a terminal, source your workspace, and execute the main launch file:

```bash
cd ~/master_slave_ws
source install/setup.bash
ros2 launch zayan master_slave.launch.py
```

### What happens when you run this?
- **Gazebo Harmonic** opens displaying the physical Slave arm.
- **RViz2** opens, showing an overlap of the virtual Master arm (`master_` frame) and the Slave arm.
- A **Joint State Publisher GUI** window appears. Moving the sliders adjusts the Master arm directly. The custom IK script (`master_slave_ik_node.py`) will automatically calculate the required joint angles and command the Gazebo Slave arm to reach the same Cartesian coordinates.

## 🧠 How it Works

1. **Forward Kinematics (FK):** The script reads the Master arm's joint angles ($q_1, q_2, q_3$) and calculates its absolute endpoint $X, Y, Z$ Cartesian coordinates.
2. **Inverse Kinematics (IK):** The solver mathematically reverse-engineers the necessary joint angles ($\theta_1, \theta_2, \theta_3$) for the Slave arm to reach that exact $X, Y, Z$ target using trigonometric cosine rules and workspace clamping.
3. **Cartesian Mapping:** The arms are mapped by end-effector space, not joint-to-joint directly.
4. **Gripper Check:** The master's gripper state is passed directly 1:1 to the slave's gripper controller.
