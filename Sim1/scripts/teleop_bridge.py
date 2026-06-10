#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math

class DaVinciBridge(Node):
    def __init__(self):
        super().__init__('davinci_bridge')
        # أطوال الأذرع من الـ URDF (0.25m و 0.20m)
        self.L1, self.L2 = 0.25, 0.20
        self.create_subscription(Point, '/target_point', self.ik_callback, 10)
        self.master_pub = self.create_publisher(JointState, '/master/joint_states', 10)
        self.slave_pub = self.create_publisher(JointState, '/slave/joint_states', 10)

    def solve_ik(self, x, y, z):
        try:
            # حساب زاوية القاعدة (Base Yaw)
            t1 = math.atan2(y, x)
            r = math.sqrt(x**2 + y**2)
            s = z - 0.05 # تعويض ارتفاع الـ Base Cylinder
            
            # حساب قانون جيب التمام (Cosine Law) للذراع
            D = (r**2 + s**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            D = max(-1.0, min(1.0, D))
            t3 = math.acos(D)
            t2 = math.atan2(s, r) - math.atan2(self.L2 * math.sin(t3), self.L1 + self.L2 * math.cos(t3))
            
            return [float(t1), float(t2), float(t3)]
        except: return None

    def ik_callback(self, msg):
        angles = self.solve_ik(msg.x, msg.y, msg.z)
        if angles:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ['joint1', 'joint2', 'joint3']
            js.position = angles
            self.master_pub.publish(js)
            self.slave_pub.publish(js)

def main():
    rclpy.init()
    rclpy.spin(DaVinciBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
