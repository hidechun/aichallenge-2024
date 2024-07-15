#!/usr/bin/env python3
import rclpy
import rclpy.node
import rclpy.qos
import rclpy.executors
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from tier4_debug_msgs.msg import Float32MultiArrayStamped
from autoware_auto_control_msgs.msg import AckermannControlCommand



class DisplayInfoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("object_marker")
        self.info = [0.0]*10
        self.sub_control_cmd = self.create_subscription(
            AckermannControlCommand, 
            "/control/command/control_cmd", 
            lambda msg: (self.info.__setitem__(3, msg.longitudinal.acceleration),
                         self.info.__setitem__(4, msg.longitudinal.speed), 
                         self.info.__setitem__(5, msg.lateral.steering_tire_angle), 
                         self.info.__setitem__(6, msg.lateral.steering_tire_rotation_rate)), 
            1
        )
        self.sub_kinematics_state = self.create_subscription(
            Odometry, 
            "/localization/kinematic_state", 
            lambda msg: (self.info.__setitem__(0, msg.twist.twist.linear.x),
                         self.info.__setitem__(1, msg.twist.twist.angular.z)), 
            1
        )

        #self.sub_kinematics_state = self.create_subscription(Odometry, "/localization/kinematic_state", self.callback, 1)
        self.pub = self.create_publisher(Float32MultiArrayStamped, "/aichallenge/info", 1) 
        self.timer = self.create_timer(0.02, lambda: self.pub.publish(
            Float32MultiArrayStamped(
                stamp=self.get_clock().now().to_msg(),
                data=self.info
            )
        ))

       
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(DisplayInfoNode())
    executor.spin()
    rclpy.spin(DisplayInfoNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
