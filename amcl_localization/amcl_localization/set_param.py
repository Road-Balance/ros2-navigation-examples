from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('set_param_service_client')
        self.cli = self.create_client(SetParameters, 'amcl/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to start...')
        self.req = None

    def send_request(self):
        self.req = SetParameters.Request()

        param = Parameter()
        param.name = "alpha1"
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = 0.3
        self.req.parameters.append(param)

        param = Parameter()
        param.name = "alpha2"
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = 0.15
        self.req.parameters.append(param)

        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            if minimal_client.future.result() is not None:
                response = minimal_client.future.result()
                minimal_client.get_logger().info(
                    'Result of set parameters: for %s' %
                    (str(response)))
            else:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (minimal_client.future.exception(),))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()