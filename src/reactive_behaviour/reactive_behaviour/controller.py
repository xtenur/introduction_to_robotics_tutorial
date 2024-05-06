import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.node import DrivingSwarmNode, main_fn

class VelocityController(DrivingSwarmNode):

    # use with use_rviz:=false

    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.started = True
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.setup_command_interface()
        
    def timer_cb(self):
        msg = Twist()
        # x = self.forward_distance - 0.3
        # x = x if x < 0.1 else 0.1
        # x = x if x >= 0 else 0.0
        # msg.linear.x = x
        msg.linear.x = -1.0
        self.publisher.publish(msg)
    
    def laser_cb(self, msg):
        self.forward_distance = msg.ranges[0]



def main():
    main_fn('controller', VelocityController)


if __name__ == '__main__':
    main()
