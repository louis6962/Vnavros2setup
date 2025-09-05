#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class TwistToAck(Node):
    def __init__(self):
        super().__init__('twist_to_ack')

        # Parameters
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steering_angle', 0.42)
        self.declare_parameter('speed_limit', 1.0)
        self.declare_parameter('twist_topic', '/cmd_vel')
        self.declare_parameter('ackermann_topic', '/ackermann_cmd')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('curv_denom_min', 0.15)  # floor on speed used for curvature at near-zero v

        # Cache params
        self.L = float(self.get_parameter('wheelbase').value)
        self.max_steer = float(self.get_parameter('max_steering_angle').value)
        self.v_lim = float(self.get_parameter('speed_limit').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.curv_denom_min = float(self.get_parameter('curv_denom_min').value)

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.sub = self.create_subscription(
            Twist,
            self.get_parameter('twist_topic').value,
            self.cb,
            qos
        )
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            self.get_parameter('ackermann_topic').value,
            qos
        )
        self.timer = self.create_timer(0.02, self.watchdog)
        self.last = self.get_clock().now()

    def cb(self, t: Twist):
        v = clamp(float(t.linear.x), -self.v_lim, self.v_lim)
        wz = float(t.angular.z)

        # Allow steering even when |v| ~ 0 by using a minimum denominator
        den = max(self.curv_denom_min, abs(v))
        curv = wz / den  # 1/m
        steer = math.atan(self.L * curv)  # radians

        steer = clamp(steer, -self.max_steer, self.max_steer)
        self.publish(v, steer)
        self.last = self.get_clock().now()

    def publish(self, v, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.drive = AckermannDrive()
        msg.drive.speed = float(v)
        msg.drive.steering_angle = float(steer)
        self.pub.publish(msg)

    def watchdog(self):
        if (self.get_clock().now() - self.last).nanoseconds * 1e-9 > self.timeout:
            self.publish(0.0, 0.0)

def main():
    rclpy.init()
    n = TwistToAck()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
