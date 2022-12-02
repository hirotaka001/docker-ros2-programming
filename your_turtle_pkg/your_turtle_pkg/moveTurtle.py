# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveTurtle(Node):
    def __init__(self):
        # turtlesim_moveノードの作成
        super().__init__('turtlesim_move')
        # Twist型メッセージを配信する/turtle1/cmd_velトピックを作成
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 定期的に呼ばれる亀の速度を配信するコールバック関数
        self.tmr = self.create_timer(1.0, self.timer_callback)

        # /turtle1/poseからメッセージを購読
        # メッセージが配信された時に実行されるコールバック関数pose_callbackを登録
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

    # メッセージが配信された時に呼ばれるコールバック関数
    def pose_callback(self, msg):
        # 位置と姿勢を画面に表示
        self.get_logger().info('(x, y, theta):[%f, %f, %f]' % (msg.x, msg.y, msg.theta))

    # 定期的に呼ばれるコールバック関数の定義
    def timer_callback(self):
        msg = Twist()
        # 並進方向の速度のx成分
        msg.linear.x = 1.0
        # 回転方向の速度のz成分
        msg.angular.z = 0.5
        # メッセージの配信
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    move = MoveTurtle()
    rclpy.spin(move)

if __name__ == '__main__':
    main()
