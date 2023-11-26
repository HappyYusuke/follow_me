import rclpy
from rclpy.node import Node

class BaseController(Node):
    def __init__(self):
        super().__init__('follow_me/base_controller')


def main():
    print('Hello World')
