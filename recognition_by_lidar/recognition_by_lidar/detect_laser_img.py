import numpy as np
import cv2
import math
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError

# 縮小サイズを取得. 1[pixel] = 0.01[mm]pixel
disc_size = 0.01
# disc_factor
disc_factor = 1/disc_size
# Max LiDAR Range
max_lidar_range = 3.5
# max_lidar_rangeとdisc_factorを使って画像サイズを設定する
img_size = int(max_lidar_range*2*disc_factor)
# 画像を表示するか否かのフラグ
imgshow_flg = True

# YOLO
model = YOLO('/home/demulab/kanazawa_ws/src/follow_me/recognition_by_lidar/weights/best.pt')

class LaserToImg(Node):
    def __init__(self):
        super().__init__('laser_to_img_node')
        # Subscriber
        self.create_subscription(LaserScan, '/scan', self.cloud_to_img_callback, qos_profile_sensor_data)
        # OpenCV
        self.bridge = CvBridge()

    def cloud_to_img_callback(self, scan):
        # LiDARデータ
        maxAngle = scan.angle_max
        minAngle = scan.angle_min
        angleInc = scan.angle_increment
        maxLength = scan.range_max
        ranges = scan.ranges
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
                    blank_img[pix_y, pix_x] = [0, 0, 0]

        # CV2画像からROSメッセージに変換してトピックとして配布する
        # 後で書く

        # 画像の表示処理. imgshow_flgがTrueの場合のみ表示する
        if imgshow_flg:
            results = model(blank_img)
            annotated_frame = results[0].plot()
            cv2.imshow('YOLOv8 Inference', annotated_frame), cv2.waitKey(3)
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
