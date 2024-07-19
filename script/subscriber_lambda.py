# -*- coding: utf-8 -*-
import rclpy
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    # minimal_subscriberノードの作成
    node = rclpy.create_node('minimal_subscriber')

    # String型メッセージを購読する/topicトピックを作成。メッセージを受信した時にデータを画面に表示
    subscription = node.create_subscription(
        String, 'topic', lambda msg: node.get_logger().info('I heard: "%s"' % msg.data), 10)
    subscription  # prevent unused variable warning

    # ループに入り、メッセージが配信されるのを待つ
    rclpy.spin(node)
    # プログラムの終了処理
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()