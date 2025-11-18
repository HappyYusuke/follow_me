# follow_me
follow_meは2D-LiDARとYOLOv8を用いた人追従機能を提供するROS2パッケージです。
本パッケージに含まれる機能は以下の通りです。
* 雑多な環境下における人追従
* ロボット内の処理の様子を可視化

## Description
本パッケージは以下の環境を想定しています。
* 屋内環境
* 二輪差動駆動台車
* 2D-LiDARがロボットの前方に搭載されている
* 2D-LiDARの高さは人の足首レベル
* GPUを搭載したPC
  > Laptop RTX 4070 8GB で0.7GB程度

本システムは、YOLOv8による人の両脚検出器から算出される目標座標までの距離と角度の偏差を収束させるようなPID制御により人追従を実現しています。
また、動的検出範囲（ピンク色の円型範囲）により追従目標を特定し、雑多な環境下での追従が可能にしています。

### 人の両脚検出器の作成
人の両脚検出器は両脚部分の画像データセットを収集し、リアルタイム物体検出アルゴリズムである[YOLO](https://arxiv.org/pdf/1506.02640.pdf)を用いて作成します。
前処理として、2D-LiDARから取得した[LaserScan型](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)の
距離データを1[pixel]=0.01[m]とした大きさ700x700[pixel]の俯瞰画像に変換します。
これは、[laser_to_img.py](recognition_by_lidar/recognition_by_lidar/laser_to_img.py)が提供する機能であり、変換された画像は
[Image型](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)のトピックとして配布され、
[ros2 bag](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
で記録することでデータセットの収集を行います。<br>
本手法では人の両脚部分を1つのpersonクラスとしてアノテーションし、回転処理、モザイク処理、MixUp処理といったデータ拡張を行うことで、
合計21061枚の画像からデータセットを作成しました。学習モデルの初期重みは、YOLOv8xを用いています。
本手法で作成した重みファイルが[weights](https://kanazawa-it.box.com/s/lfyox8d2pab6dd741z6i4juj9ea9jc32)（金沢工業大学が提供するメールアドレスのみアクセス可）にあります。<br>

### 追従目標の特定
追従目標の特定は、1フレーム前の目標座標を中心とした円型範囲を現在のフレームに生成し、範囲内に両脚が検出されればそれを追従対象としています。
円型範囲の半径は[follow_me_params.yaml](recognition_by_lidar/config/follow_me_params.yaml)にあります。初期設定は0.5[m]です。<br>

### 追従の制御
ロボット台車の制御は[base_controller.py](recognition_by_lidar/recognition_by_lidar/base_controller.py)が提供する機能であり、
ロボットから目標座標までの距離と角度の偏差を収束させるようにPID制御を実装しており、PID制御の出力は並進速度[m/s]と旋回速度[rad/s]
です。それぞれのPIDゲインは[follow_me_params.yaml](recognition_by_lidar/config/follow_me_params.yaml)にあります。<br>

<p align="center">
  <img src="https://github.com/HappyYusuke/follow_me/assets/82449194/130acd02-e3d9-4419-9a4f-3c534209d5d5" width="50%">
</p>
<p align="center">
  追従中のイメージ
</p>

## Demonstration Video
雑多な環境下での直線経路、曲線経路、直角経路による実験中の動画です。<br>
👉 [【Demonstration Video】follow me](https://youtu.be/t0HLpdR9z9w)

## Requirement
| 項目 | バージョン |
| --- | --- |
| Ubuntu | 20.04 |
| ROS2 | Humble |
| Python | 3.10.12 |
| OpenCV | 4.8.1 |
| YOLO | v8 |

## Installation
ROS2のインストールは完了している前提です。
<details>
<summary>Step 1: yolov8_rosのインストール</summary>
  
  [Miguel Ángel González Santamarta](https://github.com/mgonzs13)氏の[yolov8_ros](https://github.com/mgonzs13/yolov8_ros.git)
  の「[Installation](https://github.com/mgonzs13/yolov8_ros?tab=readme-ov-file#installation)」を参考にしてください。

</details>

<details>
<summary>Step 2: 重みファイルのダウンロードとパス設定</summary>

* **重みファイルのダウンロード** <br>
  [best.pt](https://kanazawa-it.box.com/s/dpgnab1ihxors82bm6cs3cz6s2g099ns)（金沢工業大学が提供するメールアドレスのみアクセス可）
  から重みファイルをダウンロードしてください。ダウンロード先はホームディレクトリ直下です（~/）。

* **パス設定** <br>
  [yolov8.launch.pyのパス指定](https://github.com/mgonzs13/yolov8_ros/blob/e4fb26e4c58f99b641e80ee0bf90bb4775632d69/yolov8_bringup/launch/yolov8.launch.py#L31C9-L31C36)の値を``~/best.pt``にしてください。<br>
  変更前
  ```python
  def generate_launch_description():


    #
    # ARGS
    #
    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument(
        "model",
        default_value="yolov8m.pt",
        description="Model name or path")
  ```
  変更後
  ```py
  def generate_launch_description():


    #
    # ARGS
    #
    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument(
        "model",
        default_value="~/best.pt",
        description="Model name or path")
  ```

</details>


<details>

<summary>Step 3: ビルド</summary>
  
  ```bash
  cd ~/ros2_ws/
  colcon build --symlink-install
  ```

</details>

## Usage
ロボット台車と2D-LiDARは起動済みであることが前提です。<br>
yolov8.launch.pyを実行
```bash
ros2 launch yolov8_bringup yolov8.launch.py
```
follow_me.launch.pyを実行
```bash
ros2 launch recognition_by_lidar follow_me.launch.py
```

## TODO
* アクション通信で実装

## Author
金澤祐典 (金沢工業大学　工学部　ロボティクス学科　出村研究室)


