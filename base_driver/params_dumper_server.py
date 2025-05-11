import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import ListParameters

class ParamsDumperServer(Node):
    def __init__(self):
        super().__init__('params_dumper_server')
        package_name = 'base_driver'
        config_dir = os.path.join(get_package_share_directory(package_name), 'config')
        os.makedirs(config_dir, exist_ok=True)
        output_path = os.path.join(config_dir, 'base.yaml')
        self.dump_parameters('hm_base_driver_node', output_path)

    def dump_parameters(self, node_name, output_file):
        # Get list of all nodes
        node_names = self.get_node_names()
        
        if node_name not in node_names:
            self.get_logger().error(f"Node '{node_name}' not found!")
            return

        # Get parameters from the node
        client = self.create_client(
            GetParameters,
            f'/{node_name}/get_parameters'
        )
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service not available for {node_name}")
            return

        # # Get all parameter names
        # list_client = self.create_client(
        #     GetParameters,
        #     f'/{node_name}/list_parameters'  # Use list_parameters service
        # )
        # if not list_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().error("list_parameters service unavailable")
        #     return
        # list_request = GetParameters.Request()
        # list_future = list_client.call_async(list_request)
        # rclpy.spin_until_future_complete(self, list_future)
        # if list_future.result() is None:
        #     self.get_logger().error("Failed to list parameters")
        #     return
        # param_names = list_future.result().names  # Get names from response

        request = GetParameters.Request()
        request.names = ['*']

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            params = {
                node_name: {
                    'ros__parameters': {
                        name: value.value 
                        for name, value in zip(request.names, future.result().values)
                    }
                }
            }

            # Write to YAML file
            with open(output_file, 'w') as f:
                yaml.dump(params, f, default_flow_style=False)
            self.get_logger().info(f"Parameters dumped to {output_file}")
        else:
            self.get_logger().error(f"Failed to get parameters from {node_name}")

def main(args=None):
    rclpy.init(args=args)
    dumper = ParamsDumperServer()
    rclpy.shutdown()

if __name__ == '__main__':
    main()