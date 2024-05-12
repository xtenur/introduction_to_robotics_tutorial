import rclpy
import numpy as np
from scipy.optimize import minimize
from rclpy.node import Node

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped


class LocatorNode(Node):

    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('locator node started')
        self.last_position=[0.0, 0.0, 0.0]
        
    def range_cb(self, msg):
        self.anchor_ranges.append(msg)
        self.anchor_ranges = self.anchor_ranges[-10:]
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')

    def timer_cb(self):
        if not self.initialized:
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)
    
    def calculate_position(self):
        if not len(self.anchor_ranges):
            return 0.0, 0.0, 0.0

        position=minimize(self.calculate_msqerror,self.last_position,    options={
        'ftol':1e-10,         # Tolerance
        'maxiter': 1000      # Maximum iterations
        }).x
        self.get_logger().info("estimated Position: "+str(position))
        self.last_position=position
        return position[0],position[1],position[2]
    #driving_swarm_messages.msg.Range(range=1.5280119180679321, anchor=geometry_msgs.msg.Point(x=1.0, y=1.0, z=1.0))
    def calculate_msqerror(self,guess):
        return np.mean([np.square(r.range-self.distance(guess,r.anchor)) for r in self.anchor_ranges])
    def distance(self,guess,anchor):
        return np.sqrt(np.square(guess[0]-anchor.x)+np.square(guess[1]-anchor.y)+np.square(guess[2]-anchor.z))


def main(args=None):
    rclpy.init(args=args)

    node = LocatorNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
