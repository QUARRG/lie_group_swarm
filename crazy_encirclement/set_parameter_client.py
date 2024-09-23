# set_parameter_client.py
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

class SetParameterClient(Node):
    def __init__(self, node_name='set_parameter_client'):
        super().__init__(node_name)
        self.client = self.create_client(SetParameters, '/crazyflie_server/set_parameters')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def set_parameter(self, param_name, param_value, param_type=2):
        # Create a request
        request = SetParameters.Request()

        # Define the parameter to be set
        parameter = Parameter()
        parameter.name = param_name

        # Define the value for the parameter
        parameter_value = ParameterValue()
        parameter_value.type = param_type
        if param_type == 2:
            parameter_value.integer_value = param_value
        elif param_type == 3:
            parameter_value.double_value = param_value
        elif param_type == 4:
            parameter_value.string_value = param_value
        elif param_type == 1:
            parameter_value.bool_value = param_value

        # Add the parameter and value to the request
        parameter.value = parameter_value
        request.parameters.append(parameter)

        # Send the request asynchronously
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Process response
        try:
            response = future.result()
            if response.results[0].successful:
                self.get_logger().info(f'Parameter {param_name} set successfully')
            else:
                self.get_logger().info(f'Failed to set parameter {param_name}: {response.results[0].reason}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
