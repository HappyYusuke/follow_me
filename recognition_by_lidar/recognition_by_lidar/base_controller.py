import time
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult, ParameterEvent
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point


class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        # Publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.data_pub = self.create_publisher(Point, '/follow_me/distance_angle_data', 10)
        # Subscriber
        self.create_subscription(Point, '/follow_me/target_point', self.callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Service
        self.srv_client = self.create_client(GetParameters, '/follow_me/person_detector/get_parameters')
        while not self.srv_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('/follow_me/laser_to_img server is not available ...')
        # Parameters
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('tolerance', Parameter.Type.DOUBLE),
                    ('i_range', Parameter.Type.DOUBLE),
                    ('target_width', Parameter.Type.DOUBLE),
                    ('target_height', Parameter.Type.DOUBLE),
                    ('lkp', Parameter.Type.DOUBLE),
                    ('akp', Parameter.Type.DOUBLE),
                    ('aki', Parameter.Type.DOUBLE),
                    ('akd', Parameter.Type.DOUBLE)])
        self.add_on_set_parameters_callback(self.param_callback)
        # Get parameters
        self.param_dict = {}
        self.param_dict['tolerance'] = self.get_parameter('tolerance').value
        self.param_dict['i_range'] = self.get_parameter('i_range').value
        self.param_dict['target_width'] = self.get_parameter('target_width').value
        self.param_dict['target_height'] = self.get_parameter('target_height').value
        self.param_dict['lkp'] = self.get_parameter('lkp').value
        self.param_dict['akp'] = self.get_parameter('akp').value
        self.param_dict['aki'] = self.get_parameter('aki').value
        self.param_dict['akd'] = self.get_parameter('akd').value
        self.param_dict['target_radius'] = self.get_param()  # person_detectorからもってくる
        # Value
        self.twist = Twist()
        self.target_angle = 0.0
        self.target_distance = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.delta_t = 0.0
        self.robot_angular_vel = 0.0
        # Output
        self.output_screen()

    def output_screen(self):
        for key, value in self.param_dict.items():
            self.get_logger().info(f"{key}: {value}")

    def param_event_callback(self, receive_msg):
        for data in receive_msg.changed_parameters:
            if data.name == 'target_radius':
                self.param_dict['target_radius'] = data.value.double_value
                self.get_logger().info(f"Param event: {data.name} >>> {self.param_dict['target_radius']}")

    def param_callback(self, params):
        for param in params:
            self.param_dict[param.name] = param.value
            self.get_logger().info(f"Set param: {param.name} >>> {param.value}")
        return SetParametersResult(successful=True)

    def get_param(self):
        req = GetParameters.Request()
        req.names = ['target_radius']
        future = self.srv_client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                break
        return future.result().values[0].double_value

    def point_to_angle(self, point):
        return math.degrees(math.atan2(point.y, point.x))

    def point_to_distance(self, point):
        distance = math.sqrt(point.x**2 + point.y**2)
        if point.x < 0.0:
            distance *= -1
        return distance

    def callback(self, receive_msg):
        self.target_x = receive_msg.x
        self.target_y = receive_msg.y
        self.target_distance = self.point_to_distance(receive_msg)
        self.target_angle = self.point_to_angle(receive_msg)

    def odom_callback(self, receive_msg):
        self.robot_angular_vel = receive_msg.twist.twist.angular.z

    # 比例制御量計算
    def p_control(self):
        return self.param_dict['akp']*self.target_angle

    # 微分制御量計算
    def d_control(self, p_term):
        return self.param_dict['akd']*(p_term - self.robot_angular_vel)

    # 積分制御量計算
    def i_control(self, p_term, d_term):
        value = 0.0
        diff = (p_term + d_term) - self.robot_angular_vel

        if not diff < self.param_dict['tolerance'] and diff < self.param_dict['i_range']:
            value = self.param_dict['aki']*self.target_angle*self.delta_t

        return value

    def pid_update(self):
        # 制御量を計算
        p_term = self.p_control()
        d_term = self.d_control(p_term)
        i_term = self.i_control(p_term, d_term)

        linear_vel = self.param_dict['lkp']*self.target_distance
        angular_vel = -1*(p_term + i_term + d_term)

        if linear_vel < 0.0:
            angular_vel = 0.0

        return linear_vel, angular_vel

    def in_range(self):
        result = False
        if abs(self.target_x) <= self.param_dict['target_width']:
            result = True
        return result

    def execute(self, rate=100):
        start_time = time.time()
        before_time = 0.0

        while rclpy.ok():
            self.delta_t = time.time() - start_time
            rclpy.spin_once(self)

            # 許容範囲内外を判
            if self.in_range():
                linear_vel = 0.0
                angular_vel = 0.0
            else:
                linear_vel, angular_vel = self.pid_update()

            # 制御量をパブリッシュ
            self.twist.linear.x = linear_vel
            self.twist.angular.z = angular_vel
            self.pub.publish(self.twist)

            # 実験用に目標までの距離と角度をパブリッシュ
            data = Point()
            data.x = self.target_distance
            data.z = self.target_angle
            self.data_pub.publish(data)

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
