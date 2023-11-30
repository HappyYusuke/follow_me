import time
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point


class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        # Publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscriber
        self.create_subscription(Point, '/follow_me/target_point', self.callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Parameters
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('tolerance', Parameter.Type.DOUBLE),
                    ('i_range', Parameter.Type.DOUBLE),
                    ('kp', Parameter.Type.DOUBLE),
                    ('ki', Parameter.Type.DOUBLE),
                    ('kd', Parameter.Type.DOUBLE)])
        # Value
        self.twist = Twist()
        self.angle = 0.0
        self.delta_t = 0.0
        self.robot_angular_vel = 0.0
        self.tolerance = 0.0
        self.i_range = 0.0
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0

    def point_to_angle(self, point):
        return math.degrees(math.atan2(point.y, point.x))

    def callback(self, receive_msg):
        self.angle = self.point_to_angle(receive_msg)

    def odom_callback(self, receive_msg):
        self.robot_angular_vel = receive_msg.twist.twist.angular.z

    # 比例制御量計算
    def p_control(self):
        return self.kp*self.angle

    # 微分制御量計算
    def d_control(self, p_term):
        return self.kd*(p_term - self.robot_angular_vel)

    # 積分制御量計算
    def i_control(self, p_term, d_term):
        value = 0.0
        diff = (p_term + d_term) - self.robot_angular_vel

        if not diff < self.tolerance and diff < self.i_range:
            value = self.ki*self.angle*self.delta_t

        return value

    def pid_update(self):
        # Get parameters
        self.tolerance = self.get_parameter('tolerance').value
        self.i_range = self.get_parameter('i_range').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        p_term = self.p_control()
        d_term = self.d_control(p_term)
        i_term = self.i_control(p_term, d_term)

        self.twist.angular.z = p_term + i_term + d_term
        self.pub.publish(self.twist)

    def execute(self, rate=100):
        start_time = time.time()
        before_time = 0.0

        while rclpy.ok():
            self.delta_t = time.time() - start_time
            rclpy.spin_once(self)
            self.pid_update()
            time.sleep(1/rate)


def main():
    rclpy.init()
    node = BaseController()
    try:
        node.execute()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
