import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from yolov8_msgs.msg import DetectionArray


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        # OpenCV Bridge
        self.bridge = CvBridge()
        # Publisher
        self.pub = self.create_publisher(Point, '/follow_me/target_point', 10)
        # Subscriber
        self.create_subscription(DetectionArray, '/yolo/detections', self.yolo_callback, 10)
        self.create_subscription(Image, '/yolo/dbg_image', self.img_show, 10)
        # Service
        self.srv_client = self.create_client(GetParameters, '/follow_me/laser_to_img/get_parameters')
        while not self.srv_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('/follow_me/laser_to_img server is not available ...')
        # Parameters
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('target_dist', Parameter.Type.DOUBLE)])
        self.add_on_set_parameters_callback(self.param_callback)
        # Get parameters
        self.other_param = 'discrete_size'  # laser_to_imgからもってくる
        self.target_dist = self.get_parameter('target_dist').value
        self.discrete_size = self.get_param()
        # Value
        self.center_x = self.center_y = None
        self.target_point = Point()
        self.target_px = []
        self.laser_img = None
        self.height = None
        self.width = None

    def param_callback(self, params):
        for param in params:
            if param.name=='target_dist':
                self.target_dist = param.value
    
    def get_param(self):
        req = GetParameters.Request()
        req.names = [self.other_param]
        future = self.srv_client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                self.discrete_size = future.result().values[0].double_value
                self.get_logger().info(f"Get param: discrete_size >>> {self.discrete_size}")
                break
            self.get_logger().info('Could not get params ...')

    def yolo_callback(self, receive_msg):
        if not receive_msg.detections:
            self.center_x = self.center_y = None
        else:
            person = receive_msg.detections
            self.center_x = person[0].bbox.center.position.x
            self.center_y = person[0].bbox.center.position.y

    def plot_robot_point(self):
        # 画像の中心を算出
        robot_x = round(self.width / 2)
        robot_y = round(self.height / 2)
        # 描画処理
        cv2.circle(img = self.laser_img,
                   center = (robot_x, robot_y),
                   radius = 5,
                   color = (0, 255, 0),
                   thickness = -1)

    def plot_person_point(self):
        cv2.circle(img = self.laser_img,
                   center = (round(self.center_x), round(self.center_y)),
                   radius = 8,
                   color = (0, 0, 255),
                   thickness = -1)

    def plot_target_point(self):
        cv2.circle(self.laser_img,
                   center = (self.target_px[0], self.target_px[1]),
                   radius = 10,
                   color = (255, 0, 0),
                   thickness = 2)

    def generate_target(self):
        self.target_px.clear()
        # Get parameters
        
        # 画像の中心を算出
        robot_x = round(self.height / 2)
        robot_y = round(self.width / 2)
        # 目標座標を生成(px): 横x, 縦y
        target_x = round(self.center_x)
        target_y = round(self.center_y + (target_dist/self.discrete_size))
        self.target_px.append(target_x)
        self.target_px.append(target_y)
        # 目標座標を生成(m): 縦x, 横y(ロボット座標系に合わせる)
        self.target_point.x = (robot_x - target_y)*self.discrete_size
        self.target_point.y = (robot_y - target_x)*self.discrete_size
        # パブリッシュ
        self.pub.publish(self.target_point)

    def img_show(self, receive_msg):
        self.laser_img = self.bridge.imgmsg_to_cv2(receive_msg, desired_encoding='bgr8')
        self.height, self.width, _ = self.laser_img.shape[:3]
        self.plot_robot_point()
        if self.center_x == self.center_y == None:
            pass
        else:
            self.generate_target()
            self.plot_target_point()
            self.plot_person_point()
        cv2.imshow('follow_me', self.laser_img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = PersonDetector()
    try:
        node.get_logger().info('Running')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
