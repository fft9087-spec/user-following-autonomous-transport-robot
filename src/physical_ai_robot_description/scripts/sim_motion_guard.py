#!/usr/bin/env python3
"""Simulation command gate with latched tilt stop and wall-clock watchdogs.

Not a certified hardware safety controller. Gazebo drive must subscribe only
to /cmd_vel_safe. Explicit reset is required after excessive tilt.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class MotionGuard(Node):
    def __init__(self):
        super().__init__('sim_motion_guard')
        self.declare_parameter('max_linear_speed', 0.26)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('max_linear_accel', 0.6)
        self.declare_parameter('max_angular_accel', 0.8)
        self.target = Twist()
        self.v = self.w = 0.0
        self.last_command = self.last_odom = 0.0
        self.tilt = math.inf
        self.latched = False
        self.publisher = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.status = self.create_publisher(Bool, '/sim_motion_guard/stopped', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data)
        self.create_service(Trigger, '~/reset', self.reset)
        self.last_tick = time.monotonic()
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def command(self, msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            self.target = msg
            self.last_command = time.monotonic()

    def odometry(self, msg):
        q = msg.pose.pose.orientation
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if not math.isfinite(norm) or norm < 0.5:
            return
        self.tilt = math.degrees(math.acos(max(-1.0,min(1.0,1-2*(q.x*q.x+q.y*q.y)/norm))))
        self.last_odom = time.monotonic()
        if self.tilt > 15.0 and not self.latched:
            self.latched = True
            self.get_logger().error(f'Tilt {self.tilt:.1f} deg: motion latched OFF')

    def reset(self, request, response):
        response.success = time.monotonic()-self.last_odom < 0.3 and self.tilt < 5.0
        if response.success:
            self.latched = False
            self.last_command = 0.0
        response.message = 'Reset; fresh command required' if response.success else 'Need fresh upright odometry'
        return response

    def tick(self):
        now = time.monotonic()
        dt = min(now-self.last_tick, 0.1)
        self.last_tick = now
        stopped = self.latched or now-self.last_odom > 0.3 or now-self.last_command > 0.5
        out = Twist()
        if stopped:
            self.v = self.w = 0.0
        else:
            vmax = self.get_parameter('max_linear_speed').value
            wmax = self.get_parameter('max_angular_speed').value
            target_v = max(-vmax,min(vmax,self.target.linear.x))
            target_w = max(-wmax,min(wmax,self.target.angular.z))
            a = self.get_parameter('max_linear_accel').value
            alpha = self.get_parameter('max_angular_accel').value
            self.v += max(-a*dt,min(a*dt,target_v-self.v))
            self.w += max(-alpha*dt,min(alpha*dt,target_w-self.w))
            out.linear.x, out.angular.z = self.v, self.w
        self.publisher.publish(out)
        self.status.publish(Bool(data=stopped))

def main():
    rclpy.init()
    node = MotionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
