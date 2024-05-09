import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
import numpy as np
import random

class VelocityController(DrivingSwarmNode):

    # ros2 launch reactive_behaviour robot.launch.py use_rviz:=false
    # ... (with arb mode 1 & 2 off)
    # ... (with arb mode 1 on) -> sometimes off-balance
    # ... (with arb mode 2 on)
    #
    # ros2 launch reactive_behaviour swarmlab.launch.py use_rviz:=false
    # ... (with arb mode 1 & 2 off)
    # ... (with arb mode 1 on) -> falls in narrow part
    # ... (with arb mode 2 on)

    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.turn_around_narrow_corner: bool = False
        self.turn_bc_front_blocked: bool = False
        self.random_pos_neg_one: int = 1
        self.random_turn_vel: float = 0.0
        # only one arb mode can be active
        self.arb_mode_1: bool = False
        self.arb_mode_2: bool = False

        self.ranges = []
        self.east = 0
        self.northeast = 0
        self.no_no_ea = 0
        self.north = 0
        self.no_no_we = 0
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

        # x = np.min([self.north, self.northeast, self.northwest, self.no_no_ea, self.no_no_we]) - 0.3
        x = self.north - 0.3
        x = x if x < threshold_vel_x else threshold_vel_x
        x = x if x >= 0 else 0.0
        # negative velocity means going in direction of wheel-less side of robot
        msg.linear.x = -x

        # turn left or right arbitrarily
        if self.arb_mode_1:
            if random.randint(1, 5) == 3:
                if self.northwest > 3.0:
                    self.random_turn_vel = random.randrange(10) * 0.1
                elif self.northeast > 3.0:
                    self.random_turn_vel = -random.randrange(10) * 0.1
                else:
                    self.random_turn_vel = 0.0
            msg.angular.z = self.random_turn_vel
        elif self.arb_mode_2:
            if self.northwest > 2.0 and self.no_no_we > 3.0:
                msg.angular.z = random.randrange(10) * 0.1
            elif self.northeast > 2.0 and self.no_no_ea > 3.0:
                msg.angular.z = -random.randrange(10) * 0.1

        # set state for getting out of narrow corners
        if self.north < 0.5 and self.northeast < 0.2 and self.northwest < 0.2:
            self.turn_around_narrow_corner = True
        else:
            self.turn_around_narrow_corner = False

        if self.turn_around_narrow_corner:
            msg.angular.z = turn_vel
            msg.linear.x = 0.0
        else:
            # as long as robot is walking straight, random_pos_neg_one gets assigned new values
            # if robot is in state turn_bc_front_blocked, it uses random_pos_neg_one with an unchanging value
            if not self.turn_bc_front_blocked:
                self.random_pos_neg_one = -1 if random.randint(0, 1) == 0 else 1

            if self.north < 0.5:
                self.turn_bc_front_blocked = True
                # positive angular.z means turning left; it is randomized with +1 or -1 here (turning left or right is arbitrary)
                msg.angular.z = self.random_pos_neg_one * turn_vel
            elif self.northwest < 0.5 or self.no_no_we < 0.5:
                self.turn_bc_front_blocked = False
                msg.angular.z = -turn_vel
                # msg.angular.x *= 0.5
            elif self.northeast < 0.5 or self.no_no_ea < 0.5:
                self.turn_bc_front_blocked = False
                msg.angular.z = turn_vel
                # msg.angular.x *= 0.5

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
        self.no_no_we = msg.ranges[(forward + 22) % 360]
        self.no_no_ea = msg.ranges[(forward - 22) % 360]


def main():
    main_fn('controller', VelocityController)


if __name__ == '__main__':
    main()
