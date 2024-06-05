import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription


def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")


def get_ros2_nodes(*args):
    package_dir = get_package_share_directory("measure_process_ros2_pkg")

    robot_node_list = []

    process_name = "ros2_control_node, robot_state_publisher, spawner, component_container, zenoh-bridge-ros2dds, containerd-shim, nice_robotics_gui"
    proc_list = process_name.split(", ")

    robot_node_list.append(
        Node(
            package="measure_process_ros2_pkg",
            executable="measure_process",
            name="benchmark",
            output="screen",
            parameters=[
                {
                    "process_name": process_name,
                    "process_period": 0.1,
                },
            ],
        )
    )

    for i in range(len(proc_list)):
        robot_node_list.append(
            Node(
                package="measure_process_ros2_pkg",
                executable="record_cpu_usage",
                name="stats_cpu_recorder",
                output="screen",
                on_exit=launch.actions.Shutdown(),
                parameters=[
                    {
                        "output_file": f"/tmp/cpu_{proc_list[i]}.txt",
                        "initial_delay": 10,
                        "number_samples": 800,
                        "topic_array_index": i,  # Index within "process_name[]" of measure_process
                    },
                ],
            )
        )

    return robot_node_list


def generate_launch_description():

    return LaunchDescription(
        [
            # DeclareLaunchArgument(
            #     "world",
            #     default_value="N" + str(num_robots) + "_world.wbt",
            #     description="Benchmark World",
            # ),
            # DeclareLaunchArgument(
            #     "mode", default_value="realtime", description="Webots startup mode"
            # ),
            # webots,
            # webots._supervisor,
            # # This action will kill all nodes once the Webots simulation has exited
            # launch.actions.RegisterEventHandler(
            #     event_handler=launch.event_handlers.OnProcessExit(
            #         target_action=webots,
            #         on_exit=[
            #             launch.actions.UnregisterEventHandler(
            #                 event_handler=reset_handler.event_handler
            #             ),
            #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
            #         ],
            #     )
            # ),
            # # Add the reset event handler
            # reset_handler,
        ]
        + get_ros2_nodes()
    )
