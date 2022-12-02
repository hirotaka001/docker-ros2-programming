import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.action import RotateAbsolute

class RotateTurtle(Node):
    def __init__(self):
        super().__init__('rotate_turtle')

        # アクションを呼び出す準備
        self._action_client = ActionClient(
            self, RotateAbsolute, '/turtle1/rotate_absolute')

    # アクションにゴールを送信する関数定義
    def send_goal(self, theta):
        # アクションに送信するメッセージ
        goal_msg = RotateAbsolute.Goal()
        # 亀の角度を指定
        goal_msg.theta = theta

        # アクションサーバの開始を待機
        self._action_client.wait_for_server()
        # アクションサーバに非同期にゴールを送信
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = RotateTurtle()

    # 目標となる亀の姿勢を送信
    theta = 90.0
    action_client.send_goal(theta)

if __name__ == '__main__':
    main()