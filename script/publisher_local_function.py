# -*- coding: utf-8 -*-
import rclpy
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    # minimal_publisherノードの作成
    node = rclpy.create_node('minimal_publisher')
    # String方のメッセージを配信する/topicトピックを作成
    publisher = node.create_publisher(String, 'topic', 10)

    msg = String()
    i = 0

    # 定期的に呼ばれるコールバック関数の定義。文字列「Hello World」と通番を/topicに配信
    def timer_callback():
        nonlocal i
        msg.data = 'Hello World: %d' % i
        i += 1
        node.get_logger().info('Publishing: "%s"' % msg.data)
        publisher.publish(msg)

    # 0.5秒に1回メッセージを配信
    timer_period = 0.5  # seconds
    timer = node.create_timer(timer_period, timer_callback)
    # ループに入り、コール関数を待つ
    rclpy.spin(node)
    # プログラムの終了処理
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()