#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import serial
import threading
import math
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class AMRNode(Node):
    def __init__(self):
        super().__init__('gati_bot_node')

        # --- SERIAL CONFIGURATION ---
        serial_port = '/dev/motor'
        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=0.05)
            self.ser_lock = threading.Lock()  # Prevents collision between read/write
            self.get_logger().info(f"Connected to {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            exit(1)

        # --- ROBOT PARAMETERS ---
        # Confirmed measured value.
        self.wheel_radius = 0.05

        # Confirmed measured value (your actual center-to-center wheel distance).
        self.wheel_base = 0.495

        # Encoder ticks per wheel revolution.
        # IMPORTANT: the .ino firmware now decodes BOTH encoder channels on
        # every CHANGE (full X4 quadrature) instead of the old single-edge
        # (X1) read. The same physical wheel revolution now produces ~4x the
        # raw ticks it used to (old value was 498.4 for X1). Using the old
        # value here is exactly what caused 1 real meter to show up as ~5 m
        # in RViz: the node was dividing a 4x-inflated tick count by a
        # 4x-too-small ticks_per_rev.
        # 1993.6 = 498.4 * 4 is the right STARTING point, but treat it as a
        # starting point only — measure your actual value with
        # calibrate_odom.py (see below) and paste the printed result here.
        self.ticks_per_rev = 1993.6

        # Confirmed: both encoders rise together with matching sign during
        # straight driving, so no sign flip is needed (left as 1/1).
        self.left_tick_sign = 1
        self.right_tick_sign = 1

        # CALIBRATION: even with matching sign, real wheels rarely produce
        # perfectly identical ticks-per-distance (tolerance in the wheel,
        # gearing, or encoder). Any small mismatch shows up as a constant
        # small d_theta every cycle even while driving dead straight, which
        # is what makes the path curve in RViz.
        #
        # Don't hand-tune these by trial and error — run calibrate_odom.py
        # while driving the robot a known straight-line distance. It reads
        # the live "ODOM l r" ticks straight from the .ino over serial,
        # computes the exact ticks_per_rev and left/right scale for your
        # hardware, and plots the reconstructed path so you can confirm it's
        # straight and the right length before touching RViz again. Paste
        # the three values it prints (ticks_per_rev, left_tick_scale,
        # right_tick_scale) in here.
        self.left_tick_scale = 1.0
        self.right_tick_scale = 1.0

        # Max physical speed of your robot (m/s)
        # Increase this if the robot moves too slow, decrease if it's too fast
        self.max_speed = 0.5

        # --- ODOMETRY STATE ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_l = None
        self.prev_r = None
        self.curr_l = 0
        self.curr_r = 0
        self.last_time = self.get_clock().now()

        # -- ROS2 SUBSCRIBER FOR CMD_VEL ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # -- Odom Publisher and TF Broadcaster ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Start serial reading thread
        threading.Thread(target=self.read_serial_loop, daemon=True).start()

    def cmd_vel_callback(self, msg: Twist):
        """ Converts Twist messages to Differential Drive wheel speeds """
        v = msg.linear.x
        w = msg.angular.z

        # Differential Drive Kinematics
        left_vel  = v - (w * self.wheel_base / 2.0)
        right_vel = v + (w * self.wheel_base / 2.0)

        # Map velocity to -1.0 to 1.0 range based on max_speed
        left_pwm  = left_vel / self.max_speed
        right_pwm = right_vel / self.max_speed

        # Constrain to hard limits
        left_pwm  = max(min(left_pwm, 1.0), -1.0)
        right_pwm = max(min(right_pwm, 1.0), -1.0)

        # Deadzone to stop motor humming
        if abs(v) < 0.001 and abs(w) < 0.001:
            left_pwm = 0.0
            right_pwm = 0.0

        line = f"CMD {left_pwm:.2f} {right_pwm:.2f}\n"
        self.get_logger().info(f"Sending to ESP32 : {line.strip()}")

        with self.ser_lock:
            self.ser.write(line.encode('utf-8'))

    def read_serial_loop(self):
        """ Continuously reads Odom ticks from the microcontroller """
        while rclpy.ok():
            with self.ser_lock:
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith("ODOM"):
                            parts = line.split()
                            if len(parts) == 3:
                                self.curr_l = int(parts[1])
                                self.curr_r = int(parts[2])
                                self.update_odometry()
                    except Exception as e:
                        self.get_logger().warn(f"Serial Read Error: {e}")

    def update_odometry(self):
        if self.prev_l is None:
            self.prev_l = self.curr_l
            self.prev_r = self.curr_r
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Delta Ticks
        dl_ticks = self.left_tick_sign  * self.left_tick_scale  * (self.curr_l - self.prev_l)
        dr_ticks = self.right_tick_sign * self.right_tick_scale * (self.curr_r - self.prev_r)
        self.prev_l = self.curr_l
        self.prev_r = self.curr_r

        # Distance per tick = (2 * PI * R) / Ticks_Per_Rev
        dist_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_l = dl_ticks * dist_per_tick
        d_r = dr_ticks * dist_per_tick

        d_center = (d_r + d_l) / 2.0
        d_theta  = (d_r - d_l) / self.wheel_base

        # Update pose using the mid-point heading (better accuracy on turns
        # than applying the fully-updated heading to this step's translation)
        theta_mid = self.theta + d_theta / 2.0
        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # normalize

        # 1. Publish Odometry Message
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)

        # 2. Publish Transform (TF)
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = AMRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors before closing
        node.ser.write(b"CMD 0.00 0.00\n")
        node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
