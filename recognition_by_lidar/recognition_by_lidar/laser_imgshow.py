import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class LaserImgshow(Node):
    def __init__(self):
        super().__init__('laser_imgshow_node')
        self.sub_img = self.create_subscription(Image, '/follow_me/laser_img', self.show_img, 10)
        self.bridge = CvBridge()

    def show_img(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('laser_image', img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    laser_imgshow = LaserImgshow()
    try:
        rclpy.spin(laser_imgshow)
    except KeyboardInterrupt:
        pass
    laser_imgshow.destroy_node()
    rclpy.shutdown()
