from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import yaml 

def generate_launch_description():
    package_name = 'triton_cam_driver'
    
    # --- 0. Define Launch Arguments ---
    mode = LaunchConfiguration('mode')
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='single_cam',
        description='Camera configuration mode: "left", "right", "single_cam", or "dual_cam".'
    )
    
    # --- Paths and Definitions ---
    # Use TextSubstitution for package-relative paths when defining constants
    package_dir = get_package_share_directory(package_name)

    # script_path = os.path.join(package_dir, 'scripts', 'configure_ips.py') 
    # network_config_path = os.path.join(package_dir, 'config', 'network_config.yaml')
    left_params = os.path.join(package_dir, 'config', 'triton_left_params.yaml')
    right_params = os.path.join(package_dir, 'config', 'triton_right_params.yaml')
    
    driver_package = 'triton_cam_driver'
    driver_executable = 'start'

    # # --- 1. Load Custom Network Configuration from YAML ---
    # # This YAML reading happens synchronously at launch definition time
    # try:
    #     with open(network_config_path, 'r') as file:
    #         config = yaml.safe_load(file)
    # except Exception as e:
    #     print(f"ERROR: Failed to load network configuration YAML: {e}")
    #     return LaunchDescription([mode_arg])

    # # Retrieve values
    # subnet = config['subnet_mask']
    # gateway = config['gateway']
    # utility_path = os.environ.get(config['arena_sdk_path_env'], '')
    
    # if not utility_path:
    #     print(f"ERROR: Environment variable {config['arena_sdk_path_env']} not set.")
    #     return LaunchDescription([mode_arg])

    # # --- 2. Define all static configuration variables from the YAML ---
    # left_mac = config['cameras']['triton_left']['mac']
    # left_ip = config['cameras']['triton_left']['ip']
    # right_mac = config['cameras']['triton_right']['mac']
    # right_ip = config['cameras']['triton_right']['ip']

    # --- 3. Execute the Python IP Configuration Script ---
    
    # We pass ALL MAC/IP values and the MODE. The Python script will use the MODE 
    # to decide which configurations to execute/ignore. This keeps the launch file simple.
    
    # ip_config_process = ExecuteProcess(
    #     cmd=[
    #         script_path,
    #         '--utility-path', utility_path,
    #         '--subnet', subnet,
    #         '--gateway', gateway,
    #         '--left-mac', left_mac,
    #         '--left-ip', left_ip,
    #         '--right-mac', right_mac,
    #         '--right-ip', right_ip,
    #         '--mode', mode,
    #     ],
    #     name='ip_config_setup',
    #     output='screen'
    # )
    
    # --- 4. Conditionally Launch Nodes using IfCondition ---
    
    # Condition to launch Left Node: mode is 'left' OR mode is 'dual_cam'
    left_condition = IfCondition(PythonExpression(["'", mode, "' in ['left', 'single_cam', 'dual_cam']"]))

    # Condition to launch Right Node: mode is 'right' OR mode is 'dual_cam'
    right_condition = IfCondition(PythonExpression(["'", mode, "' in ['right', 'dual_cam']"]))

    # Node for Triton Left
    triton_left_node = Node(
        package=driver_package, 
        executable=driver_executable, 
        name='triton_left', 
        output='screen', 
        parameters=[left_params],
        condition=left_condition
    )

    # Node for Triton Right
    triton_right_node = Node(
        package=driver_package, 
        executable=driver_executable, 
        name='triton_right', 
        output='screen', 
        parameters=[right_params],
        condition=right_condition
    )

    # Additional environment variables 
    # 1. Force CycloneDDS for everything in this launch
    rmw_impl = SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp')
    
    # 2. Force the XML config path (ensure this path exists inside the container)
    config_path = 'file://' + os.path.expanduser('~/ros2_configs/cyclonedds.xml')
    cyclonedds_uri = SetEnvironmentVariable(name='CYCLONEDDS_URI', value=config_path)

    return LaunchDescription([
        rmw_impl,
        cyclonedds_uri,
        mode_arg,
        # ip_config_process,
        triton_left_node,
        triton_right_node,
    ])