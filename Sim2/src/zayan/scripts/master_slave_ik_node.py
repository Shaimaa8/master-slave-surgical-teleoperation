#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class MasterSlaveIK(Node):
    def __init__(self):
        super().__init__('master_slave_ik')

        # ----- Kinematic constants (from your xacro) -----
        self.L1 = 0.15   # base to shoulder (joint1->joint2)
        self.L2 = 0.25   # shoulder to elbow (joint2->joint3)
        self.L3 = 0.20   # elbow to wrist (joint3->gripper_base)

        # ----- Subscriber to master arm joint states -----
        self.sub = self.create_subscription(
            JointState,
            '/master/joint_states',
            self.master_joint_callback,
            10
        )

        # ----- Publishers for slave controllers -----
        # Using JointGroupPositionController which subcribes to ~/commands
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

        self.get_logger().info("Slave IK node ready, publishing position commands.")

        # Store last master joint values for filtering (optional)
        self.last_master_positions = None

    # ----------------------------------------------------------
    # Forward Kinematics for the master arm (aligned with URDF)
    # ----------------------------------------------------------
    def forward_kinematics(self, q1, q2, q3):
        # In URDF: q2=0, q3=0 creates a perfectly VERTICAL arm straight UP.
        # r_plane is the radial distance from Z-axis. When straight up, r_plane is 0.
        r_plane = self.L2 * math.sin(q2) + self.L3 * math.sin(q2 + q3)
        x = math.cos(q1) * r_plane
        y = math.sin(q1) * r_plane
        
        # When straight up, Z is L1 + L2 + L3.
        z = self.L1 + self.L2 * math.cos(q2) + self.L3 * math.cos(q2 + q3)
        return x, y, z

    # ----------------------------------------------------------
    # Inverse Kinematics for the slave arm (Analytic)
    # ----------------------------------------------------------
    def inverse_kinematics(self, xd, yd, zd):
        theta1 = math.atan2(yd, xd)
        r = math.hypot(xd, yd)
        z_rel = zd - self.L1
        d2 = r*r + z_rel*z_rel

        max_reach = self.L2 + self.L3
        min_reach = abs(self.L2 - self.L3)
        d = math.sqrt(d2)

        # Clamp to reachable workspace
        if d > max_reach:
            scale = max_reach / d
            r *= scale
            z_rel *= scale
            d = max_reach
        elif d < min_reach:
            scale = min_reach / d
            r *= scale
            z_rel *= scale
            d = min_reach

        # Cosine rule for q3 (elbow angle relative to link2)
        # Using standard formula: d^2 = L2^2 + L3^2 - 2*L2*L3*cos(pi - q3), which simplifies to + 2*L2*L3*cos(q3)
        cos_q3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        
        # We pick positive sin_q3 for standard elbow configuration
        sin_q3 = math.sqrt(1 - cos_q3**2)
        theta3 = math.atan2(sin_q3, cos_q3)
        
        # Calculate theta2 (shoulder pitch angle from vertical Z-axis)
        alpha = math.atan2(r, z_rel)  # Target vector angle from vertical
        
        # Angle beta between target vector and link2
        cos_beta = (d**2 + self.L2**2 - self.L3**2) / (2 * d * self.L2)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        
        # Since we picked positive sin_q3 for the elbow, theta2 is alpha - beta
        theta2 = alpha - beta
        
        return theta1, theta2, theta3

    # ----------------------------------------------------------
    # Callback: master joint states
    # ----------------------------------------------------------
    def master_joint_callback(self, msg):
        try:
            idx1 = msg.name.index('joint1')
            idx2 = msg.name.index('joint2')
            idx3 = msg.name.index('joint3')
            idx_grip = msg.name.index('gripper_joint')
        except ValueError:
            self.get_logger().warn("Master joint state message missing some joints, skipping.")
            return

        q1 = msg.position[idx1]
        q2 = msg.position[idx2]
        q3 = msg.position[idx3]
        grip = msg.position[idx_grip]

        # 1. Forward kinematics -> get target end effector Cartesian pose from master
        x, y, z = self.forward_kinematics(q1, q2, q3)

        # 2. Inverse kinematics -> solve what slave joints we need to reach it
        t1, t2, t3 = self.inverse_kinematics(x, y, z)

        # 3. Publish to controllers
        arm_msg = Float64MultiArray()
        arm_msg.data = [t1, t2, t3]
        self.arm_pub.publish(arm_msg)
        
        grip_msg = Float64MultiArray()
        grip_msg.data = [grip]
        self.gripper_pub.publish(grip_msg)

        self.get_logger().debug(f"Master pose: ({x:.3f},{y:.3f},{z:.3f}) -> Slave joints: ({t1:.3f},{t2:.3f},{t3:.3f})")

def main(args=None):
    rclpy.init(args=args)
    node = MasterSlaveIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()