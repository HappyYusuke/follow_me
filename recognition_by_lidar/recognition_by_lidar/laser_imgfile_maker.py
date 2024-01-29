import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# 保存先のパス(末尾に / 入れる)
dir_path = '/home/demulab/follow_me_images/yolo/'
# 保存するファイル名(拡張子なしのファイル名)
file_name = 'laser_img'


class LaserImgshow(Node):
    def __init__(self):
        super().__init__('laser_imgshow_node')
        self.sub_img = self.create_subscription(Image, '/yolo/dbg_image', self.show_img, 10)
        self.bridge = CvBridge()
        # Value
        self.num = 0
        self.file_path = None

    def show_img(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 画像保存処理
        self.file_path = f'{dir_path}{file_name}_{self.num}.jpg'
        result = cv2.imwrite(self.file_path, img)
        self.num += 1
        self.get_logger().info(f"{result}: {self.file_path}")

        # 表示処理
        cv2.imshow('laser_image', img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    laser_imgshow = LaserImgshow()
    try:
        os.makedirs(dir_path)
        rclpy.spin(laser_imgshow)
    except KeyboardInterrupt:
        pass
    laser_imgshow.destroy_node()
    rclpy.shutdown()
