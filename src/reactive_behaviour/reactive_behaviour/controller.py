import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.node import DrivingSwarmNode, main_fn

class VelocityController(DrivingSwarmNode):

    def __init__(self, name: str) -> None:
        super().__init__(name)
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0.0
        self.right_distance=0.0
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.setup_command_interface()
        self.started=True
        
    def timer_cb(self):
        if not self.started:
            return
        msg = Twist()
        if self.forward_distance>0.2:
            msg.linear.x= -0.1
        if self.forward_distance>0.4 or self.forward_distance==float('inf'):
            msg.linear.x = -0.25
        if self.right_distance>=0.45 and self.forward_distance>0.2:
            msg.angular.z=-0.7
        if self.forward_distance <= 0.4:
            msg.angular.z=0.2
        if self.forward_distance<=0.2 or self.right_distance<=0.2:
            msg.angular.z=1.0
        self.publisher.publish(msg)
    
    def laser_cb(self, msg):
        start_index=150
        end_index=210
        min = float('inf')
        for i in range(start_index,end_index):
            if msg.ranges[i]<min:
                min=msg.ranges[i]
        self.forward_distance=min

        start_index_right=90
        end_index_right=100
        min_right = float('inf')
        for i in range(start_index_right,end_index_right):
            if msg.ranges[i]<min_right:
                min_right=msg.ranges[i]
        self.right_distance=min_right
        #self.get_logger().info(str(min_right))
        #self.get_logger().info(str(msg.ranges))



def main():
    main_fn('controller', VelocityController)
    


if __name__ == '__main__':
    main()


