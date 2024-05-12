import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import tf_transformations

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.goal = None
        self.position = None
        self.angle=0.0
        self.required_angle=0.0
        self.last_time=self.get_clock().now()
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose_marker', 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        
    def timer_cb(self):
        msg = Twist()
        x = 0.0
        time=self.get_clock().now()
        #self.get_logger().info("Delta_time: \n"+str((float((time-self.last_time).nanoseconds)/1_000_000_000.0)))
        if self.goal and self.position:
            #self.get_logger().info("Distance to goal: "+str(np.sqrt(np.square(self.position[0]-self.goal[0])+np.square(self.position[1]-self.goal[1]))))
            # Calculate the correct angle between "forward" vector (1,0) and "goal" vector (goal-position)
            #self.required_angle=np.deg2rad(360)-np.arccos((1*(self.goal[0]-self.position[0])+0*(self.goal[1]-self.position[1]))/(1*np.sqrt(np.square(self.goal[0]-self.position[0])+np.square(self.goal[1]-self.position[1]))))
            dot = 1*(self.goal[0]-self.position[0])+ 0*(self.goal[1]-self.position[1])      # dot product
            det = 1*(self.goal[1]-self.position[1]) - 0*(self.goal[0]-self.position[0])    # determinant
            self.required_angle = (np.deg2rad(360)+np.arctan2(det, dot))%np.deg2rad(360)  # atan2(y, x) or atan2(sin, cos)

            if np.absolute(self.angle-self.required_angle)>0.01:
                factor = -1.0
                if self.required_angle>self.angle:
                    if self.required_angle-self.angle<np.deg2rad(180):
                        factor=1.0
                else:
                    if self.angle-self.required_angle>np.deg2rad(180):
                        factor=1.0

                msg.angular.z=factor*0.1
                self.angle+=msg.angular.z*(float((time-self.last_time).nanoseconds)/1_000_000_000.0)
                self.angle=self.angle%(2*np.pi) 
                #self.get_logger().info("Actual angle: "+str(self.angle)+ "Required: "+str(self.required_angle))
            elif np.absolute(self.angle-self.required_angle)>0.02:
                factor = -1.0
                if self.required_angle>self.angle:
                    if self.required_angle-self.angle<np.deg2rad(180):
                        factor=1.0
                else:
                    if self.angle-self.required_angle>np.deg2rad(180):
                        factor=1.0
                msg.angular.z=factor* 0.01
                self.angle+=msg.angular.z*(float((time-self.last_time).nanoseconds)/1_000_000_000.0)
                self.angle=self.angle%(2*np.pi) 
                #self.get_logger().info(str(np.rad2deg(self.angle)))
            else:
                if np.sqrt(np.square(self.position[0]-self.goal[0])+np.square(self.position[1]-self.goal[1])) >0.05:
                    x=0.05
                if np.sqrt(np.square(self.position[0]-self.goal[0])+np.square(self.position[1]-self.goal[1])) >0.3:
                    x=0.1
        self.last_time=time
        msg.linear.x=x


        self.publisher.publish(msg)
        angle = 0.0
        pose_x=0.0
        pose_y=0.0
        if self.position:
            pose_x=self.position[0]
            pose_y=self.position[1]
        self.publish_marker((pose_x,pose_y), self.angle, relative=True)
    
    def goal_cb(self, msg):
        goal = msg.pose.position.x, msg.pose.position.y
        if self.goal != goal:
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal
    
    def laser_cb(self, msg):
        self.forward_distance = msg.ranges[0]
        
    def position_cb(self, msg):
        self.position = msg.point.x, msg.point.y
    
    def publish_marker(self, position, angle, relative=False):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if not relative:
            msg.header.frame_id = 'map'
        else:
            msg.header.frame_id = 'base_link'

        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, angle)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pose_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

