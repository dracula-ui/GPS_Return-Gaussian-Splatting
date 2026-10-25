"""Starts the ROS side: gz<->ROS bridge, (optional) SLAM bridge, mission node.
Gazebo, PX4 and the SLAM server are started by the scripts/ helpers.

ros2 launch rtlf_ros stack.launch.py method:=b4 pose_source:=emulated seed:=1
ros2 launch rtlf_ros stack.launch.py method:=s4 pose_source:=slam
ros2 launch rtlf_ros stack.launch.py method:=b4 record_dir:=$HOME/rtlf_data/run1      # record TUM dataset
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    L = lambda k: context.launch_configurations[k]
    sim_time = {"use_sim_time": True}
    rgb, depth = L("rgb_gz_topic"), L("depth_gz_topic")
    nodes = [Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge", output="screen",
        arguments=[rgb + "@sensor_msgs/msg/Image[gz.msgs.Image",
                   depth + "@sensor_msgs/msg/Image[gz.msgs.Image",
                   "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": False}])]
    if L("pose_source") == "slam":
        nodes.append(Node(package="rtlf_ros", executable="slam_bridge_node", name="slam_bridge", output="screen",
                          parameters=[sim_time, {"rgb_topic": rgb, "depth_topic": depth, "send_hz": float(L("send_hz"))}]))
    mission = Node(
        package="rtlf_ros", executable="mission_node", name="rtlf_mission", output="screen",
        parameters=[sim_time, {
            "method": L("method"), "seed": int(L("seed")), "pose_source": L("pose_source"),
            "naive_anchor": L("naive_anchor").lower() == "true", "eps": float(L("eps")),
            "kappa_scale": float(L("kappa_scale")), "cruise": float(L("cruise")), "out": L("out"),
            "record_dir": L("record_dir"), "record_hz": float(L("record_hz")), "tol": float(L("tol")),
            "ret_timeout": float(L("ret_timeout")), "yaw_mode": L("yaw_mode"), "yaw_rate": float(L("yaw_rate")),
            "rgb_topic": rgb, "depth_topic": depth}])
    nodes.append(mission)
    # stop the whole launch when the mission finishes (so scripted sweeps can continue)
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=mission, on_exit=[EmitEvent(event=Shutdown())])))
    return nodes


def generate_launch_description():
    args = {"method": "b4", "seed": "1", "pose_source": "emulated", "naive_anchor": "false", "eps": "4.0",
            "kappa_scale": "1.0", "cruise": "-1.0", "out": "results", "record_dir": "", "send_hz": "10.0",
            "record_hz": "10.0", "tol": "3.0", "ret_timeout": "300.0", "yaw_mode": "fixed", "yaw_rate": "20.0",
            "rgb_gz_topic": "/rtlf/rgbd/image", "depth_gz_topic": "/rtlf/rgbd/depth_image"}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args.items()] + [OpaqueFunction(function=_setup)])
