import os
import yaml
from xml.etree import ElementTree
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

# read xml file from path and change the value of the tag
def modify_sdf(xml_file_path, namespace, frame_prefix):
    """
        The sdf file of turtlebot3 use gazebo_ros_diff_drive plugin to drive the car 
        and publish odometry.
        But the odometry_frame and robot_base_frame are hard coded in the sdf file.
        So we need to change the value of the tag to add frame_prefix to the value.
    """
    if not os.path.exists(xml_file_path):
        print('File not found: {}'.format(xml_file_path))
        exit()
    if not os.path.isfile(xml_file_path):
        print('Path is not a file: {}'.format(xml_file_path))
        exit()
    # load file
    try:
        f = open(xml_file_path, 'r')
        entity_xml = f.read()
    except IOError as e:
        print('Could not open file {}: {}'.format(xml_file_path, e))
        exit()
    if entity_xml == '':
        print('File is empty: {}'.format(xml_file_path))
        exit()
    
    # parse xml
    try:
        xml_parsed = ElementTree.fromstring(entity_xml)
    except ElementTree.ParseError as e:
        print('Invalid xml {}: {}'.format(xml_file_path, e))
        exit()
    
    """
        change
        ```
        <plugin name="turtlebot3_diff_drive" filename="libgazebo_ros_diff_drive.so">
            <odometry_frame>odom</odometry_frame>
            <robot_base_frame>base_footprint</robot_base_frame>
        </plugin>
        ```
        to
        ```
        <plugin name="turtlebot3_diff_drive" filename="libgazebo_ros_diff_drive.so">
            <odometry_frame>frame_prefix+odom</odometry_frame>
            <robot_base_frame>frame_prefix+base_footprint</robot_base_frame>
        </plugin>
        ```
    """
    model_tag = xml_parsed.find('model')
    for plugin in model_tag.findall('plugin'):
        if plugin.get('name') == 'turtlebot3_diff_drive':
            for child in plugin:
                if child.tag == 'odometry_frame':
                    child.text = frame_prefix + child.text
                if child.tag == 'robot_base_frame':
                    child.text = frame_prefix + child.text
    
    """
        add
        ```
        <plugin name="turtlebot3_p3d" filename="libgazebo_ros_p3d.so">
            <ros>
                <namespace>namespace</namespace>
                <remapping>odom:=odom_ground_truth</remapping>
            </ros>
            <update_rate>50.0</update_rate>
            <body_name>base_link</body_name>
            <gaussian_noise>0.01</gaussian_noise>
        </plugin>
        ```
        to model_tag
    """
    plugin_tag = ElementTree.SubElement(model_tag, 'plugin')
    plugin_tag.set('name', 'turtlebot3_p3d')
    plugin_tag.set('filename', 'libgazebo_ros_p3d.so')
    ros_tag = ElementTree.SubElement(plugin_tag, 'ros')
    namespace_tag = ElementTree.SubElement(ros_tag, 'namespace')
    namespace_tag.text = namespace
    remapping_tag = ElementTree.SubElement(ros_tag, 'remapping')
    remapping_tag.text = 'odom:=odom_ground_truth'
    update_rate_tag = ElementTree.SubElement(plugin_tag, 'update_rate')
    update_rate_tag.text = '50.0'
    body_name_tag = ElementTree.SubElement(plugin_tag, 'body_name')
    body_name_tag.text = 'base_link'
    gaussian_noise_tag = ElementTree.SubElement(plugin_tag, 'gaussian_noise')
    gaussian_noise_tag.text = '0.01'

    entity_xml = ElementTree.tostring(xml_parsed)
    return entity_xml


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim_gazebo',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Start RViz'
        )
    )

    # ----------------------------------------
    # Get arguments
    # ----------------------------------------
    sim_gazebo = LaunchConfiguration('sim_gazebo')
    start_rviz = LaunchConfiguration('start_rviz')

    # ----------------------------------------
    # File path 
    # ----------------------------------------
    world_file_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot3_multi'), 'worlds', 'empty.world']
    )
    robot_description_file_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot3_description'), 'urdf', 'turtlebot3_'+TURTLEBOT3_MODEL+'.urdf']
    )
    robot_sdf_file_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 
        'models', 
        'turtlebot3_'+TURTLEBOT3_MODEL, 'model.sdf'
    )
    rviz_config_file_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot3_multi'), 'rviz2', 'turtlebot3_multi.rviz']
    )
    multi_tb3_config_file_path = os.path.join(
        get_package_share_directory('turtlebot3_multi'),
        'config',
        'multi_tb3_config.yaml'
    )

    # parse the config yaml file to get the names and initial pose of multiple turtlebot3
    with open(multi_tb3_config_file_path, 'r') as file:
        multi_tb3_config = yaml.load(file, Loader=yaml.FullLoader)
    tb3_names = multi_tb3_config['names']
    tb3_initial_x = multi_tb3_config['initial_x']
    tb3_initial_y = multi_tb3_config['initial_y']
    # check their length
    assert len(tb3_names) == len(tb3_initial_x) == len(tb3_initial_y), \
        'Length of tb3_names, tb3_initial_x and tb3_initial_y must be the same'

    # The original sdf file provided by ROBOTIS in ROS2 is not suitable for multiple robots
    # because the odometry_frame and robot_base_frame are hard coded in the sdf file.
    # we need to add frame_prefix to the value of odometry_frame and robot_base_frame
    # and save the modified sdf file to a temp file
    temp_sdf_file_path_list = []
    for i in range(len(tb3_names)):
        temp_sdf_file_path_list.append(
            os.path.join(
                get_package_share_directory('turtlebot3_multi'),
                'model_modified'+str(i)+'.sdf'
            )
        )

    # ----------------------------------------
    # Node and Launch
    # ----------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'),
                    'launch', 'gazebo.launch.py']
            )]
        ),
        launch_arguments={'verbose': 'false', 'world': world_file_path}.items(),
        condition=IfCondition(sim_gazebo),
    )

    spawn_entity_list = [] # list of spawn_entity nodes
    robot_state_publisher_list = [] # list of robot_state_publisher nodes
    for i in range(len(tb3_names)):
        # namespace and frame_prefix of the robot
        namespace = tb3_names[i]
        frame_prefix = namespace + '_'

        # change the sdf file to add frame_prefix to the value of odometry_frame and robot_base_frame
        entity_xml = modify_sdf(robot_sdf_file_path, namespace, frame_prefix)
        # save the temp sdf file
        with open(temp_sdf_file_path_list[i], 'wb') as file:
            file.write(entity_xml)
        
        # spawn robot 
        # for more information of its arguments, 
        # refer to https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_ros/scripts/spawn_entity.py
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity'+str(i),
            output='screen',
            arguments = ['-file', temp_sdf_file_path_list[i], # use the modified sdf file
                         '-entity', namespace, # Name of entity to spawn
                         '-robot_namespace', namespace, # change ROS namespace of gazebo-plugins
                         '-x', str(tb3_initial_x[i]), # Initial x pose
                         '-y', str(tb3_initial_y[i]), # Initial y pose
                         ],
            condition=IfCondition(sim_gazebo),
        )
        spawn_entity_list.append(spawn_entity)

        # robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher'+str(i),
            output='screen',
            parameters=[{
                'use_sim_time': sim_gazebo, 
                'frame_prefix': frame_prefix,}],
            namespace=namespace,
            arguments=[robot_description_file_path],
        )
        robot_state_publisher_list.append(robot_state_publisher)
    
    # rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file_path],
        parameters=[
            {"use_sim_time": sim_gazebo},
        ],
        condition=IfCondition(start_rviz),
    )

    nodes = [
        gazebo_launch,
        rviz_node,
    ]
    nodes.extend(spawn_entity_list)
    nodes.extend(robot_state_publisher_list)

    return LaunchDescription(declared_arguments + nodes)
