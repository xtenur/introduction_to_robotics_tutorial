import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class WatchdogNode(Node):

    def __init__(self):
        super().__init__('watchdog')
        self.turtle_can_turn_ = 1
        self.turtle_can_move_ = 1
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'input_cmd', self.cmd_callback, 10)
        self.create_subscription(String, 'controller_cmd', self.controller_callback, 10)
        self.get_logger().info('Watchdog node started')

    def cmd_callback(self, msg: Twist):
        # this makes the turle go backwards
        # (just so you know its working)
        if self.turtle_can_turn_ == 0:
            msg.angular.z = 0.0
        else:
            self.get_logger().info('The turtle can still turn and move.')
        if self.turtle_can_move_ == 0:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
        else:
            msg.linear.x = -1 * msg.linear.x
            self.get_logger().info('The turtle can still move.')
        self.get_logger().info(f'msg.linear.x: {msg.linear.x}')
        self.publisher.publish(msg)
        
    def controller_callback(self, msg):
        if msg.data == 'start':
            self.turtle_can_turn_ = 0
            self.get_logger().warn('The turtle cannot turn anymore!')
        if msg.data == 'stop':
            self.turtle_can_move_ = 0
            self.get_logger().warn('The turtle cannot move (or turn) anymore!')
        self.get_logger().warn(f'The controller says I should {msg.data} the turtle ...')



def main(args=None):
    rclpy.init(args=args)

    node = WatchdogNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
