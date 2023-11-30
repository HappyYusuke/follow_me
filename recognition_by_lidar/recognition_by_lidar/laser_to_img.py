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
        # Value
        self.color_list = gradation([0,0,255], [255,0,0], [1, 100], [True,True,True])[0] 

    def cloud_to_img_callback(self, scan):
        # Get parameters
        discrete_size = self.get_parameter('discrete_size').value
        max_lidar_range = self.get_parameter('max_lidar_range').value
        img_show_flg = self.get_parameter('img_show_flg').value
        # discrete_factor
        discrete_factor = 1/discrete_size
        # max_lidar_rangeとdiscrete_factorを使って画像サイズを設定する
        img_size = int(max_lidar_range*2*discrete_factor)

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
            if (ranges[i] > max_lidar_range) or (math.isnan(ranges[i])):
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
            if (pt_x < max_lidar_range) or (pt_x > -1*(max_lidar_range-disc_size)) or (pt_y < max_lidar_range) or (pt_y > -1 * (max_lidar_range-disc_size)):
                pix_x = int(math.floor((pt_x + max_lidar_range) * disc_factor))
                pix_y = int(math.floor((max_lidar_range - pt_y) * disc_factor))
                if (pix_x > img_size) or (pix_y > img_size):
                    print("Error")
                else:
                    # 色を付ける処理（赤:高, 青:低）
                    #colorMap_num = int((intensities(i)+min(intensities))/max(intensities) * 100)
                    blank_img[pix_y, pix_x] = [0, 0, 0] #self.color_list[colorMap_num]

        # CV2画像からROSメッセージに変換してトピックとして配布する
        img = self.bridge.cv2_to_imgmsg(blank_img, encoding="bgr8")
        self.pub.publish(img)

        # 画像の表示処理. imgshow_flgがTrueの場合のみ表示する
        if img_show_flg:
            cv2.imshow('laser_img', blank_img)
            #更新のため一旦消す
            blank_img = np.zeros((img_size, img_size, 3))
        else:
            pass

def main():
    rclpy.init()
    laser_to_img = LaserToImg()
    try:
        rclpy.spin(laser_to_img)
    except KeyboardInterrupt:
        pass
    laser_to_img.destroy_node()
    rclpy.shutdown()
