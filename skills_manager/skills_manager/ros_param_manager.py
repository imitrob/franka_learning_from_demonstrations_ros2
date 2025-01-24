from typing import Any, List, Tuple

from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
from rcl_interfaces.msg import ParameterType, ParameterDescriptor, IntegerRange
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from ros2node.api import get_node_names

def list_all_rosnodes(node):
    available_nodes = get_node_names(node=node, include_hidden_nodes=False)
    names = []
    for name, namespace, full_name in available_nodes:
        names.append(name)
        # print(f"Found node {name} in namespace {namespace} (full name: {full_name}")
    return names

def list_remote_parameters(node, server="parameter_tester"):
    """ Get all ROS parameter names declared. """
    # Create a client for the 'list_parameters' service
    client = node.create_client(ListParameters, f'/{server}/list_parameters')
    while not client.wait_for_service(timeout_sec=0.01):
        return None

    request = ListParameters.Request()
    # Optionally, specify a prefix, this list all parameters by default
    request.prefixes = []
    request.depth = 0  # Depth=0 means all parameters

    future = client.call(request)
    return future.result()

def set_remote_parameters(self, param_name, param_value, param_type=None, server="parameter_tester"):
    client = self.create_client(SetParameters, f'/{server}/set_parameters')
    
    while not client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('Waiting for SetParameters service...')

    request = SetParameters.Request()

    # Add the new parameter to the request
    if isinstance(param_name, (list, tuple)):
        assert param_type==None, "Not Implemented"
        for param_name_, param_value_ in zip(param_name, param_value):
            new_param = Parameter(param_name_, type_=None, value=param_value_)
            request.parameters.append(new_param.to_parameter_msg())
    else:
        new_param = Parameter(param_name, type_=param_type, value=param_value)
        request.parameters.append(new_param.to_parameter_msg())

    # Call the service to set the remote parameter
    future = client.call(request)
    
    if future.results is not None:
        self.get_logger().info(f"Successfully set {param_name} on the remote node to values {param_value}")
    else:
        self.get_logger().error(f"Failed to set {param_name} on the remote node")


def get_remote_parameters(node, param_names: List[str], server="parameter_tester"):
    """ Get ROS parameters details based on providing parameter names. """
    # Create a client for the 'get_parameters' service
    destroy_node = False
    try:
        if node is None:
            destroy_node = True
            node = Node("get_remote_parameters_temporary_node")
        
        client = node.create_client(GetParameters, f'/{server}/get_parameters')

        while not client.wait_for_service(timeout_sec=0.01):
            raise Exception(f"get_remote_parameter() cannot access ROS node name: {server} and its parameter {param_names}")
            return None

        request = GetParameters.Request()
        request.names = param_names  # Pass the list of parameter names

        future = client.call(request)
        
        # THIS WAS CHANGED, IT EXTRACTS THE PARAMETER VALUES DIRECTLY!
        # return future.result()
        # ADDED:
        get_result = future #.result()
        if get_result is None:
            raise Exception(f"get_remote_parameter() cannot access ROS node name: {server} and its parameter {param_names}")
            return None
        ret = []
        for i in range(len(get_result.values)):
            value, type = extract_param_value(get_result.values[i])
            ret.append(value)
        return ret
    finally:
        if destroy_node:
            node.destroy_node()

def get_remote_parameter(node, param_name: List[str], server="parameter_tester"):
    return get_remote_parameters(node, [param_name], server)[0]


def extract_param_value(pvalue) -> Tuple[Any, Parameter.Type]:
    """ Extract ROS parameter ParameterValue object to the parameter value (e.g., float) and type. """
    param_type = pvalue.type

    # Check the type and extract the corresponding value
    if param_type == ParameterType.PARAMETER_BOOL:
        return pvalue.bool_value, Parameter.Type.BOOL
    elif param_type == ParameterType.PARAMETER_INTEGER:
        return pvalue.integer_value, Parameter.Type.INTEGER
    elif param_type == ParameterType.PARAMETER_DOUBLE:
        return pvalue.double_value, Parameter.Type.DOUBLE
    elif param_type == ParameterType.PARAMETER_STRING:
        return pvalue.string_value, Parameter.Type.STRING
    elif param_type == ParameterType.PARAMETER_BYTE_ARRAY:
        return pvalue.byte_array_value, Parameter.Type.BYTE_ARRAY
    elif param_type == ParameterType.PARAMETER_BOOL_ARRAY:
        return pvalue.bool_array_value, Parameter.Type.BOOL_ARRAY
    elif param_type == ParameterType.PARAMETER_INTEGER_ARRAY:
        return pvalue.integer_array_value, Parameter.Type.INTEGER_ARRAY
    elif param_type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return pvalue.double_array_value, Parameter.Type.DOUBLE_ARRAY
    elif param_type == ParameterType.PARAMETER_STRING_ARRAY:
        return pvalue.string_array_value, Parameter.Type.STRING_ARRAY
    else:
        raise Exception("Unknown type or unsupported type")

def declare_parameter_slider(node, parameter_name: str, parameter_value: int, from_value=0, to_value=100, step=1):
    node.declare_parameter(
        parameter_name,
        parameter_value,
        descriptor=ParameterDescriptor(
            description='Integer slider parameter',
            integer_range=[
                IntegerRange(
                    from_value=from_value,  # Minimum value
                    to_value=to_value,  # Maximum value
                    step=step  # Increment step
                )
            ]
        )
    )


