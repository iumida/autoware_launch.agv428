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

    # 改成固定使用 isaac_ros_image_proc
    rectify_package = 'isaac_ros_image_proc'

    # --------- camera_f ---------
    """
    camera_nodes.append(
        ComposableNode(
            package='usb_cam',
            plugin='usb_cam::UsbCamNode',
            name='usb_cam_f',
            namespace='camera_f',
            parameters=[
                f'{config_base}/params_f.yaml',
                {'image_transport': 'raw'}
            ]
        )
    )

    image_proc_nodes.append(
        ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::RectifyNode',
            name='rectify_f',
            namespace='camera_f',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect')
            ]
        )
    )
    """

    # --------- camera_b ---------
    camera_nodes.append(
        ComposableNode(
            package='usb_cam',
            plugin='usb_cam::UsbCamNode',
            name='usb_cam_b',
            namespace='camera_b',
            parameters=[
                f'{config_base}/params_b.yaml',
                {'image_transport': 'raw'}
            ]
        )
    )

    image_proc_nodes.append(
        ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::RectifyNode',
            name='rectify_b',
            namespace='camera_b',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect')
            ]
        )
    )

    # --------- camera_l ---------
    """
    camera_nodes.append(
        ComposableNode(
            package='usb_cam',
            plugin='usb_cam::UsbCamNode',
            name='usb_cam_l',
            namespace='camera_l',
            parameters=[
                f'{config_base}/params_l.yaml',
                {'image_transport': 'raw'}
            ]
        )
    )

    image_proc_nodes.append(
        ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::RectifyNode',
            name='rectify_l',
            namespace='camera_l',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect')
            ]
        )
    )
    """

    # --------- camera_r ---------
    """
    camera_nodes.append(
        ComposableNode(
            package='usb_cam',
            plugin='usb_cam::UsbCamNode',
            name='usb_cam_r',
            namespace='camera_r',
            parameters=[
                f'{config_base}/params_r.yaml',
                {'image_transport': 'raw'}
            ]
        )
    )

    image_proc_nodes.append(
        ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::RectifyNode',
            name='rectify_r',
            namespace='camera_r',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect')
            ]
        )
    )
    """

    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=camera_nodes + image_proc_nodes
    )

    # 註解掉 yolox launch
    """
    yolox_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('autoware_tensorrt_yolox'),
            'launch',
            'multiple_yolox.launch.xml'
        ])),
        launch_arguments={
            'image_raw0': 'camera_f/image_rect',
            'image_raw1': 'camera_b/image_rect',
            'image_raw2': 'camera_l/image_rect',
            'image_raw3': 'camera_r/image_rect',
            'image_raw4': '',
            'image_raw5': '',
            'image_raw6': '',
            'image_raw7': '',
            'image_number': '4'
        }.items()
    )
    """

    return LaunchDescription([
        camera_container,
        # yolox_launch,
    ])
