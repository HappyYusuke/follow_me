import time
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
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
                    ('lkp', Parameter.Type.DOUBLE),
                    ('akp', Parameter.Type.DOUBLE),
                    ('aki', Parameter.Type.DOUBLE),
                    ('akd', Parameter.Type.DOUBLE)])
        self.add_on_set_parameters_callback(self.param_callback)
        # Get parameters
        self.tolerance = self.get_parameter('tolerance').value
        self.i_range = self.get_parameter('i_range').value
        self.lkp = self.get_parameter('lkp').value
        self.akp = self.get_parameter('akp').value
        self.aki = self.get_parameter('aki').value
        self.akd = self.get_parameter('akd').value
        # Value
        self.twist = Twist()
        self.angle = 0.0
        self.distance = 0.0
        self.delta_t = 0.0
        self.robot_angular_vel = 0.0
        # Output
        self.output_screen()

    def output_screen(self):
        self.get_logger().info(f"tolerance: {self.tolerance}")
        self.get_logger().info(f"i_range: {self.i_range}")
        self.get_logger().info(f"lkp: {self.lkp}")
        self.get_logger().info(f"akp: {self.akp}")
        self.get_logger().info(f"aki: {self.aki}")
        self.get_logger().info(f"akd: {self.akd}")

    def param_callback(self, params):
        for param in params:
            if param.name == 'tolerance':
                self.tolerance = param.value
            elif param.name == 'i_range':
                self.i_range = param.value
            elif param.name == 'kp':
                self.kp = param.value
            elif param.name == 'ki':
                self.ki = param.value
            else:
                self.kd = param.value
        self.get_logger().info(f"Set param: {param.name} >>> {param.value}")
        return SetParametersResult(successful=True)

    def point_to_angle(self, point):
        return math.degrees(math.atan2(point.y, point.x))

    def point_to_distance(self, point):
        return math.sqrt(point.x**2 + point.y**2)

    def callback(self, receive_msg):
        self.distance = self.point_to_distance(receive_msg)
        self.angle = self.point_to_angle(receive_msg)

    def odom_callback(self, receive_msg):
        self.robot_angular_vel = receive_msg.twist.twist.angular.z

    # 比例制御量計算
    def p_control(self):
        return self.akp*self.angle

    # 微分制御量計算
    def d_control(self, p_term):
        return self.akd*(p_term - self.robot_angular_vel)

    # 積分制御量計算
    def i_control(self, p_term, d_term):
        value = 0.0
        diff = (p_term + d_term) - self.robot_angular_vel

        if not diff < self.tolerance and diff < self.i_range:
            value = self.aki*self.angle*self.delta_t

        return value

    def pid_update(self):
        p_term = self.p_control()
        d_term = self.d_control(p_term)
        i_term = self.i_control(p_term, d_term)

        self.twist.linear.x = self.lkp*self.distance
        self.twist.angular.z = -1*(p_term + i_term + d_term)
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
