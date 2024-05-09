import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
import numpy as np
import random

class VelocityController(DrivingSwarmNode):

    # use with ros2 launch reactive_behaviour robot.launch.py use_rviz:=false

    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.turn_around_narrow_corner: bool = False
        self.turn_bc_front_blocked: bool = False
        self.randy: int = 1

        self.ranges = []
        self.east = 0
        self.northeast = 0
        self.north = 0
        self.northwest = 0
        self.west = 0

        self.started = True
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.setup_command_interface()

    def timer_cb(self):
        msg = Twist()
        threshold_vel_x = 0.12
        turn_vel = 0.5

        x = self.north - 0.3
        x = x if x < threshold_vel_x else threshold_vel_x
        x = x if x >= 0 else 0.0
        # negative velocity means going in direction of wheel-less side of robot
        msg.linear.x = -x

        if self.west > 2.0:
            msg.angular.z = random.randrange(10) * 0.1

        # set state for getting out of narrow corners
        if self.north < 1.0 and self.northeast < 0.2 and self.northwest < 0.2:
            self.turn_around_narrow_corner = True
        else:
            self.turn_around_narrow_corner = False

        if self.turn_around_narrow_corner:
            msg.angular.z = turn_vel
            msg.linear.x = 0.0
        else:
            # as long as robot is walking straight, randy gets assigned new values
            # if robot is in state turn_bc_front_blocked, it uses randy with an unchanging value
            if not self.turn_bc_front_blocked:
                self.randy = -1 if random.randint(0, 1) == 0 else 1

            if self.north < 0.5:
                self.turn_bc_front_blocked = True
                # positive angular.z means turning left
                msg.angular.z = self.randy * turn_vel
            elif self.northwest < 0.5:
                self.turn_bc_front_blocked = False
                msg.angular.z = -turn_vel
            elif self.northeast < 0.5:
                self.turn_bc_front_blocked = False
                msg.angular.z = turn_vel

        self.publisher.publish(msg)

    def laser_cb(self, msg):
        self.ranges = msg.ranges
        # 180° is forward if velocity is negative (go in wheel-less-side direction), ranges[0] is forward if velocity is positive (go in wheel-having-side direction)
        forward = 179
        self.north = msg.ranges[forward]
        # ranges[] is filled counterclockwise (clockwise would mean ADDING for east and SUBTRACTING for west)
        self.west = msg.ranges[(forward + 90) % 360]
        self.east = msg.ranges[(forward - 90) % 360]
        self.northwest = msg.ranges[(forward + 45) % 360]
        self.northeast = msg.ranges[(forward - 45) % 360]

    def find_highest_range(self):
        return np.max(self.ranges)


def main():
    main_fn('controller', VelocityController)


if __name__ == '__main__':
    main()
