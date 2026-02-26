import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math

class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('arduino_bridge')

        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.01)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(0.02, self.read_serial)

        self.frame_id = 'odom'
        self.child_frame_id = 'base_link'

    def cmd_callback(self, msg):
        cmd = f"$CMD,{msg.linear.x},{msg.angular.z}\n"
        self.serial.write(cmd.encode())

    def read_serial(self):
        line = self.serial.readline().decode(errors='ignore').strip()

        if not line.startswith("$ODOM"):
            return

        try:
            _, x, y, theta, v_l, v_r = line.split(",")

            x = float(x)
            y = float(y)
            theta = float(theta)
            v = (float(v_l) + float(v_r)) / 2.0

            now = self.get_clock().now().to_msg()

            # -------- Odometry message --------
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.frame_id
            odom.child_frame_id = self.child_frame_id

            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0

            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)

            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw

            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = float(v_r) - float(v_l)

            self.odom_pub.publish(odom)

            # -------- TF transform --------
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id

            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0

            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f"ODOM parse error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
