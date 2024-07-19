import rclpy
from turtlesim.srv import Spawn

def main(args=None):
    rclpy.init(args=args)
    # spawn_clientノード作成
    node = rclpy.create_node('spawn_client')
    # Spawnサービスのクライアントノード/spawn作成
    client = node.create_client(Spawn, '/spawn')
    # サービスを呼び出す設定
    req = Spawn.Request()
    # 新しく亀を生成する場所のx座標を指定
    req.x = 4.0
    # 新しく亀を生成する場所のy座標を指定
    req.y = 4.0
    # 新しく生成する亀の姿勢を指定
    req.theta = 0.4
    # 新しく生成する亀の名前を指定
    req.name = 'new_turtle'

    # サービスが開始されるまで待機
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
    # サービスを非同期に呼び出し
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    # サービス呼び出しが成功した時のメッセージ表示
    else:
        node.get_logger().info(
            'Result of x, y, theta, name: %f, %f, %f, %s' % (req.x, req.y, req.theta, result.name))

    # プログラムの終了処理
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()