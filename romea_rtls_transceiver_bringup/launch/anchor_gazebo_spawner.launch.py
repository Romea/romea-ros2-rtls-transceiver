# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from romea_rtls_transceiver_bringup import RTLSTransceiverMetaDescription
from romea_rtls_transceiver_description import anchor_urdf


def get_meta_description(context):
    anchor_meta_description_file_path = LaunchConfiguration(
        "meta_description_file_path"
    ).perform(context)

    return RTLSTransceiverMetaDescription(anchor_meta_description_file_path)


def launch_setup(context, *args, **kwargs):

    meta_description = get_meta_description(context)

    urdf = anchor_urdf(
        meta_description.get_name(),
        meta_description.get_type(),
        meta_description.get_communication(),
        meta_description.get_pan_id(),
        meta_description.get_id())

    urdf_filename = "/tmp/"+meta_description.get_name()+".urdf"

    with open(urdf_filename, "w") as f:
        f.write(urdf)

    launch_description = LaunchDescription()

    spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_" + meta_description.get_name(),
        namespace=meta_description.get_name(),
        arguments=[
            "-file",
            urdf_filename,
            "-entity",
            str(meta_description.get_name()),
            "-x",
            str(meta_description.get_xyz()[0]),
            "-y",
            str(meta_description.get_xyz()[1]),
            "-z",
            str(meta_description.get_xyz()[2]),
        ],
        output="screen"
    )

    launch_description.add_action(spawner)

    tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        namespace=meta_description.get_name(),
        arguments=[
            str(meta_description.get_xyz()[0]),
            str(meta_description.get_xyz()[1]),
            str(meta_description.get_xyz()[2]),
            "0.0", "0.0", "0.0",
            meta_description.get_parent_link(),
            meta_description.get_name()+"_link",
        ],
        output="screen"
    )

    launch_description.add_action(tf_publisher)

    return [launch_description]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("meta_description_file_path"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
