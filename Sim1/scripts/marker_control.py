#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import Point

class DaVinciMarker(Node):
    def __init__(self):
        super().__init__('davinci_marker')
        self.server = InteractiveMarkerServer(self, 'robot_marker')
        self.target_pub = self.create_publisher(Point, '/target_point', 10)
        
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.name = "davinci_target"
        int_marker.description = "DaVinci Precision Control"
        int_marker.scale = 0.2

        # إضافة أسهم للحركة في X, Y, Z
        for axis in ['x', 'y', 'z']:
            control = InteractiveMarkerControl()
            control.name = f"move_{axis}"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            control.orientation.w = 1.0
            if axis == 'y': control.orientation.y = 1.0
            elif axis == 'z': control.orientation.z = 1.0
            else: control.orientation.x = 1.0
            int_marker.controls.append(control)

        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        self.target_pub.publish(feedback.pose.position)

def main():
    rclpy.init()
    rclpy.spin(DaVinciMarker())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
