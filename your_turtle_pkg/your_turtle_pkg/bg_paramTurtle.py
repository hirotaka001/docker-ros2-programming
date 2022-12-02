import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters

class Bg_Param(Node):
    def __init__(self):
        super().__init__('cg_turtle')

    # パラメータを変更するための関数定義
    def setParam(self, red, green, blue):
        client = self.create_client(
            SetParameters,
            '/turtlesim/set_parameters'.format_map(locals()))

        # サービス開始を待機
        ready = client.wait_for_service(timeout_sec=5.0)
        if not ready:
            raise RuntimeError('Wait for service timed out')

        req = SetParameters.Request()

        # バックグラウンドの色の赤成分の設定
        param = Parameter()
        param.name = "background_r"
        param.value.type = ParameterType.PARAMETER_INTEGER
        param.value.integer_value = red
        req.parameters.append(param)

        # バックグラウンドの色の緑成分の設定
        param = Parameter()
        param.name = "background_g"
        param.value.type = ParameterType.PARAMETER_INTEGER
        param.value.integer_value = green
        req.parameters.append(param)

        # バックグラウンドの青成分の設定
        param = Parameter()
        param.name = "background_b"
        param.value.type = ParameterType.PARAMETER_INTEGER
        param.value.integer_value = blue
        req.parameters.append(param)

        # サービスを非同期に呼び出し
        future = client.call_async(req)

    # パラメータの値を取得して表示する関数定義
    def getParam(self):
        # パラメータ値を取得して表示するための準備
        client = self.create_client(
            GetParameters,
            '/turtlesim/get_parameters'.format_map(locals()))

        # サービスが開始されるまで待機
        ready = client.wait_for_service(timeout_sec=5.0)
        if not ready:
            raise RuntimeError('Wait for Service timed out')

        # サービスを非同期に呼び出し
        request = GetParameters.Request()
        request.names = ["background_r", "background_g", "background_b"]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            e = future.exception()
            raise RuntimeError(
                'Exception while calling service of node'
                "'{args.node_name}': {e}".format_map(locals()))

        # 得られたパラメータの値を端末に表示
        print('background_r: ', response.values[0].integer_value)
        print('background_g: ', response.values[1].integer_value)
        print('background_b: ', response.values[2].integer_value)

def main(args=None):
    rclpy.init(args=args)
    param_client = Bg_Param()

    # パラメータの値を設定
    # バックグラウンドの色を変更。赤成分: 0, 緑成分: 10, 青成分: 100
    param_client.setParam(0, 10, 100)

    # パラメータの現在の値を取得して表示
    param_client.getParam()

if __name__ == '__main__':
    main()