import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import ListParameters, GetParameters
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType

class ParamsDumperServer(Node):
    def __init__(self):
        super().__init__('params_dumper_server')
        package_name = 'base_driver'
        config_dir = os.path.join(get_package_share_directory(package_name), 'config')
        os.makedirs(config_dir, exist_ok=True)
        output_path = os.path.join(config_dir, 'base.yaml')
        self.dump_parameters('hm_base_driver_node', output_path)

    def dump_parameters(self, node_name, output_file):
        if node_name not in self.get_node_names():
            self.get_logger().error(f"Node '{node_name}' not found!")
            return

        list_client = self.create_client(ListParameters, f'/{node_name}/list_parameters')
        get_client = self.create_client(GetParameters, f'/{node_name}/get_parameters')

        # List parameters
        if not list_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("list_parameters service unavailable")
            return

        list_future = list_client.call_async(ListParameters.Request())
        rclpy.spin_until_future_complete(self, list_future)

        if list_future.result() is None:
            self.get_logger().error("Failed to list parameters")
            return

        param_names = list_future.result().result.names
        if not param_names:
            self.get_logger().warning(f"No parameters found for {node_name}")
            return

        # Get parameter values
        get_request = GetParameters.Request()
        get_request.names = param_names
        get_future = get_client.call_async(get_request)
        rclpy.spin_until_future_complete(self, get_future)

        if get_future.result() is not None:
            params = {
                node_name: {
                    'ros__parameters': {
                        name: self._parameter_to_python(value)
                        for name, value in zip(param_names, get_future.result().values)
                    }
                }
            }
            with open(output_file, 'w') as f:
                yaml.dump(params, f, default_flow_style=False)
            self.get_logger().info(f"Parameters dumped to {output_file}")


    def _parameter_to_python(self, param_value):
        """Convert ParameterValue to Python native type"""
        if param_value.type == ParameterType.PARAMETER_BOOL:
            return param_value.bool_value
        elif param_value.type == ParameterType.PARAMETER_INTEGER:
            return param_value.integer_value
        elif param_value.type == ParameterType.PARAMETER_DOUBLE:
            return param_value.double_value
        elif param_value.type == ParameterType.PARAMETER_STRING:
            return param_value.string_value
        elif param_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return list(param_value.byte_array_value)
        elif param_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(param_value.bool_array_value)
        elif param_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(param_value.integer_array_value)
        elif param_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(param_value.double_array_value)
        elif param_value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return list(param_value.string_array_value)
        return None

def main(args=None):
    rclpy.init(args=args)
    dumper = ParamsDumperServer()
    rclpy.shutdown()

if __name__ == '__main__':
    main()