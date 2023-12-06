import numpy as np
import os
import sys
import cv2
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan, Image
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge, CvBridgeError
# Custom
from .modules.gradient import gradation_3d_img as gradation


class LaserToImg(Node):
    def __init__(self):
        super().__init__('laser_to_img')
        # Publisher
        self.pub = self.create_publisher(Image, '/follow_me/laser_img', 10)
        # Subscriber
        self.create_subscription(LaserScan, '/scan', self.cloud_to_img_callback, qos_profile_sensor_data)
        # OpenCV
        self.bridge = CvBridge()
        # Parameters
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('discrete_size', Parameter.Type.DOUBLE),
                    ('max_lidar_range', Parameter.Type.DOUBLE),
                    ('img_show_flg', Parameter.Type.BOOL)])
        self.add_on_set_parameters_callback(self.param_callback)
        # Get parameters
        self.param_dict = {}
        self.param_dict['discrete_size'] = self.get_parameter('discrete_size').value
        self.param_dict['max_lidar_range'] = self.get_parameter('max_lidar_range').value
        self.param_dict['img_show_flg'] = self.get_parameter('img_show_flg').value
        # Values
        self.color_list = gradation([0,0,255], [255,0,0], [1, 100], [True,True,True])[0]
        # Output
        self.output_screen()

    def output_screen(self):
        for key, value in self.param_dict.items():
            self.get_logger().info(f"{key}: {value}")

    def param_callback(self, params):
        for param in params:
            self.param_dict[param.name] = param.value
            self.get_logger().info(f"Set param: {param.name} >>> {param.value}")
        return SetParametersResult(successful=True)

    def cloud_to_img_callback(self, scan):
        
        # discrete_factor
        discrete_factor = 1/self.param_dict['discrete_size']
        # max_lidar_rangeとdiscrete_factorを使って画像サイズを設定する
        img_size = int(self.param_dict['max_lidar_range']*2*discrete_factor)

        # LiDARデータ
        maxAngle = scan.angle_max
        minAngle = scan.angle_min
        angleInc = scan.angle_increment
        maxLength = scan.range_max
        ranges = scan.ranges
        intensities = scan.intensities
        #intensities = scan.intensities
        
        # 距離データの個数を格納
        num_pts = len(ranges)
        # 721行2列の空行列を作成
        xy_scan = np.zeros((num_pts, 2))
        # 3チャンネルの白色ブランク画像を作成
        blank_img = np.zeros((img_size, img_size, 3), dtype=np.uint8) + 255
        # rangesの距離・角度からすべての点をXYに変換する処理
        for i in range(num_pts):
            # 範囲内かを判定
            if (ranges[i] > self.param_dict['max_lidar_range']) or (math.isnan(ranges[i])):
                pass
            else:
                # 角度とXY座標の算出処理
                angle = minAngle + float(i)*angleInc
                xy_scan[i][1] = float(ranges[i]*math.cos(angle))  # y座標
                xy_scan[i][0] = float(ranges[i]*math.sin(angle))  # x座標

        # ブランク画像にプロットする処理
        for i in range(num_pts):
            pt_x = xy_scan[i, 0]
            pt_y = xy_scan[i, 1]
            if (pt_x < self.param_dict['max_lidar_range']) or (pt_x > -1*(self.param_dict['max_lidar_range']-self.param_dict['discrete_size'])) or (pt_y < self.param_dict['max_lidar_range']) or (pt_y > -1 * (self.param_dict['max_lidar_range']-self.param_dict['discrete_size'])):
                pix_x = int(math.floor((pt_x + self.param_dict['max_lidar_range']) * discrete_factor))
                pix_y = int(math.floor((self.param_dict['max_lidar_range'] - pt_y) * discrete_factor))
                if (pix_x > img_size) or (pix_y > img_size):
                    print("Error")
                else:
                    blank_img[pix_y, pix_x] = [0, 0, 0]

        # CV2画像からROSメッセージに変換してトピックとして配布する
        img = self.bridge.cv2_to_imgmsg(blank_img, encoding="bgr8")
        self.pub.publish(img)

        # 画像の表示処理. imgshow_flgがTrueの場合のみ表示する
        if self.param_dict['img_show_flg']:
            cv2.imshow('laser_img', blank_img)
            cv2.waitKey(3)
            #更新のため一旦消す
            blank_img = np.zeros((img_size, img_size, 3))
        else:
            pass

def main():
    rclpy.init()
    node = LaserToImg()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
