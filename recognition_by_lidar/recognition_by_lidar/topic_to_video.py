import cv2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# ファイル名
video_name = 'max_0.6.mp4'
# 動画ファイルの保存先
video_path = '/home/yusukepad/Videos/'
save_path = video_path + video_name


class TopicToVideo(Node):
    def __init__(self):
        super().__init__('topic_to_video_node')
        self.sub_img = self.create_subscription(Image, '/follow_me/image', self.callback, 10)
        self.bridge = CvBridge()
        # Value
        self.img = None
        self.frames = []
        self.frame_cnt = 0

    def callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def video_generator(self):
        start_time = time.time()
        while rclpy.ok():
            now_time = time.time() - start_time
            # ROS2通信
            rclpy.spin_once(self, timeout_sec=0.05)
            # パブリッシャの情報を取得
            publisher = self.get_publishers_info_by_topic('/follow_me/image')
            # パブリッシャを取得できたら画像をリストに保存
            if publisher:
                self.frames.append(self.img)
                self.get_logger().info(f"{len(self.frames)} frames")
            # パブリッシャを待機
            elif not publisher and now_time <= 30:
                self.get_logger().info("Waiting topic ...")
            # パブリッシャが消失したら動画を生成する
            else:
                # 動画の情報を生成
                frame_rate = len(self.frames)/now_time
                size = (700, 700)
                fmt = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
                self.get_logger().info(f"{len(self.frames)} frames to video ...")
                self.get_logger().info(f"{frame_rate} fps")
                # 動画を作成
                writer = cv2.VideoWriter(save_path, fmt, frame_rate, size)
                for frame in self.frames:
                    writer.write(frame)
                writer.release()
                self.get_logger().info("Completed")
                break


def main():
    rclpy.init()
    ttv = TopicToVideo()
    try:
        ttv.video_generator()
    except KeyboardInterrupt:
        pass
    ttv.destroy_node()
    rclpy.shutdown()
