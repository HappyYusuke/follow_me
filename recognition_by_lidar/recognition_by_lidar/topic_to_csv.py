import time
import math
import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


# 保存するcsvファイル名
file_name = '/home/demulab/follow_me_max-datas/max_0.6.csv'


class TopicToCSV(Node):
    def __init__(self):
        super().__init__('topic_to_csv_node')
        # Subscriber
        self.create_subscription(Point, '/follow_me/distance_angle_data', self.experiment_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Value
        self.x = None
        self.y = None
        self.tracking_dist = None
        self.start_flg = False

    def experiment_callback(self, msg):
        self.tracking_dist = msg.x

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def change_to_csv(self, dictionary):
        self.get_logger().info("Converting ...")
        with open(file_name, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(dictionary.keys())
            writer.writerows(zip(*dictionary.values()))
        self.get_logger().info("Converted!")

    def run(self):
        data_dict = {'travel_dist':[], 'tracking_dist':[]}
        processed = False
        start_time = time.time()
        while rclpy.ok:
            now_time = time.time() - start_time
            rclpy.spin_once(self, timeout_sec=0.05)
            # パブリッシャを取得
            publisher = self.get_publishers_info_by_topic('/follow_me/distance_angle_data')
            if publisher:
                processed = True
                if not self.tracking_dist is None and self.tracking_dist >= 0.0 and not self.x is None and not self.y is None:
                    # 初期化処理
                    if not self.start_flg:
                        self.start_flg = True
                        x_init = self.x
                        y_init = self.y
                    # 移動距離を計算
                    travel_dist = math.sqrt((self.x-x_init)**2+(self.y-y_init)**2)
                    # データ格納
                    data_dict['travel_dist'].append(travel_dist)
                    data_dict['tracking_dist'].append(self.tracking_dist)
                    self.get_logger().info(f"Travel dist: {travel_dist}")
                    self.get_logger().info(f"Tracking dist: {self.tracking_dist}\n")
                else:
                    self.start_flg = False
                    data_dict['travel_dist'].clear()
                    data_dict['tracking_dist'].clear()
                    self.get_logger().info("Value is negative.")
            elif not publisher and now_time <= 30 and not processed:
                self.get_logger().info("Waiting topic ...")
            else:
                self.change_to_csv(data_dict)
                break
        self.get_logger().info("All Completed!")


def main():
    rclpy.init()
    node = TopicToCSV()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
