# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveTB3(Node):
    def __init__(self):
        # /tb3_moveノードの作成
        super().__init__('tb3_move')
        # Twist型のメッセージを配信する/cmd_velトピックの作成
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 定期的に呼ばれてTB3 Burgerの速度を配信するコールバック関数
        self.tmr = self.create_timer(1.0, self.timer_callback)

    # 定期的に呼ばれるコールバック関数
    def timer_callback(self):
        msg = Twist()
        # 並進方向の速度のx成分を設定
        msg.linear.x = 0.1
        # 回転方向の速度のz成分を設定
        msg.angular.z = -1.0
        # メッセージの配信
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    move = MoveTB3()
    rclpy.spin(move)

if __name__ == '__main__':
    main()