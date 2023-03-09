# # Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# # Add license

# from launch import LaunchDescription

# from launch.actions import (
#     IncludeLaunchDescription,
#     DeclareLaunchArgument,
#     OpaqueFunction,
#     GroupAction,
# )

# from launch_ros.actions import PushRosNamespace
# from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
# from launch_ros.substitutions import FindPackageShare
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# from romea_common_bringup import device_link_name
# from romea_rtls_transceiver_bringup import TransceiverMetaDescription


# def get_robot_namespace(context):
#     return LaunchConfiguration("robot_namespace").perform(context)


# def get_meta_description(context):
#     rtls_meta_description_filename = LaunchConfiguration(
#         "meta_description_filename"
#     ).perform(context)

#     return TransceiverMetaDescription(rtls_meta_description_filename)


# def launch_setup(context, *args, **kwargs):

#     robot_namespace = get_robot_namespace(context)
#     meta_description = get_meta_description(context)

#     if not meta_description.has_driver_configuration:
#         return []

#     tranceiver_name = meta_description.get_name()
#     transceiver_namespace = str(meta_description.get_namespace() or "")

#     # driver = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource(
#     #         [
#     #             PathJoinSubstitution(
#     #                 [
#     #                     FindPackageShare("romea_rtls_bringup"),
#     #                     "launch",
#     #                     "drivers/" + meta_description.get_driver_pkg() + ".launch.py",
#     #                 ]
#     #             )
#     #         ]
#     #     ),
#     #     launch_arguments={
#     #         "device": meta_description.get_driver_device(),
#     #         "baudrate": str(meta_description.get_driver_baudrate()),
#     #         "rate": str(meta_description.get_rate()),
#     #         "frame_id": device_link_name(robot_namespace, tranceiver_name),
#     #     }.items(),
#     # )

#     return [
#         GroupAction(
#             actions=[
#                 PushRosNamespace(robot_namespace),
#                 PushRosNamespace(transceiver_namespace),
#                 PushRosNamespace(tranceiver_name),
#                 driver,
#             ]
#         )
#     ]


# def generate_launch_description():

#     declared_arguments = []

#     declared_arguments.append(DeclareLaunchArgument("meta_description_filename"))

#     declared_arguments.append(
#         DeclareLaunchArgument("robot_namespace", default_value="")
#     )

#     return LaunchDescription(
#         declared_arguments + [OpaqueFunction(function=launch_setup)]
#     )
