from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_base = '/home/ace428/autoware.agv428/src/sensor_component/external/usb_cam/config'

    camera_nodes = []
    image_proc_nodes = []
    override_nodes = []

    sides = ['f', 'b', 'l', 'r']
    output_names = ['camera0', 'camera1', 'camera2', 'camera3']

    for i, side in enumerate(sides):
        input_ns = f'/sensing/camera_{side}'
        output_ns = f'{output_names[i]}'
        config_file = f'{config_base}/params_{side}.yaml'

        # usb_cam node
        camera_nodes.append(
            ComposableNode(
                package='usb_cam',
                plugin='usb_cam::UsbCamNode',
                name=f'usb_cam_{side}',
                namespace=f'camera_{side}',
                parameters=[
                    config_file,
                    {'image_transport': 'raw'}
                ]
            )
        )

        # image_proc node
        image_proc_nodes.append(
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name=f'rectify_{side}',
                remappings=[
                    ('image', f'{input_ns}/image_raw'),
                    ('camera_info', f'{input_ns}/camera_info'),
                    ('image_rect', f'{output_ns}/image_rect')
                ]
            )
        )

        # override node
        override_nodes.append(
            Node(
                package='camera_info_tools',
                executable='override_node',
                name=f'camera_info_override_{side}',
                remappings=[
                    ('camera_info_raw', f'{input_ns}/camera_info'),
                    ('camera_info', f'camera/{output_ns}/camera_info')
                ]
            )
        )

    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=camera_nodes + image_proc_nodes
    )

    yolox_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('autoware_tensorrt_yolox'),
            'launch',
            'multiple_yolox.launch.xml'
        ])),
        launch_arguments={
            'image_raw0': '/sensing/camera0/image_rect',
            'image_raw1': '/sensing/camera1/image_rect',
            'image_raw2': '/sensing/camera2/image_rect',
            'image_raw3': '/sensing/camera3/image_rect',
            'image_raw4': '',
            'image_raw5': '',
            'image_raw6': '',
            'image_raw7': '',
            'image_number': '4'
        }.items()
    )

    return LaunchDescription([
        camera_container,
        yolox_launch,
        *override_nodes
    ])
