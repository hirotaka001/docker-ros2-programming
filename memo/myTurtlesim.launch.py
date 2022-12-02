import launch.actions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Launch(起動)時にメッセージを出力する
        launch.actions.LogInfo(msg="Launch turtlesim node and turtle_teleop_key node."),
        # Launch(起動)してから3秒後にメッセージを出力する
        launch.actions.TimerAction(period=3.0,actions=[
            launch.actions.LogInfo(msg="It's been three minutes since the launch."),
        ]),
        # Turtlesim シミュレータの起動
        Node(
            # パッケージ名
            package='turtlesim',
            # ノードを起動する名前空間
            namespace='turtlesim',
            # ノードの実行ファイル名
            executable='turtlesim_node',
            # ノード名
            name='turtlesim',
            # 標準出力をコンソールに表示
            output='screen',
            # パラメータの値を設定する(黄色)
            parameters=[{'background_r':255}, {'background_g':255}, {'background_b':0},]
        ),
        # Turtlesimをキーボードで操作するノードの起動
        Node(
            # パッケージ名
            package='turtlesim',
            # ノードを起動する名前空間
            namespace='turtlesim',
            # ノードの実行ファイル名
            executable='turtle_teleop_key',
            # ノード名
            name='teleop_turtle',
            # turtle_teleop_keyをxterm上で実行する
            prefix="xterm -e"
        ),
    ])