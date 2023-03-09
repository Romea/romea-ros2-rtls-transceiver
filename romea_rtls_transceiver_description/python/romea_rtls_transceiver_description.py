# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import xacro

from ament_index_python.packages import get_package_share_directory


def urdf(prefix, name, type, configuration, pan_id, id, mode, parent_link, xyz, ros_namespace):

    xacro_file = (
        get_package_share_directory("romea_rtls_transceiver_description")
        + "/urdf/"
        + type
        + "_"
        + configuration
        + ".xacro.urdf"
    )

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "name": name,
            "mode": mode,
            "pan_id": str(pan_id),
            "id": str(id),
            "parent_link": parent_link,
            "xyz": " ".join(map(str, xyz)),
            "ros_namespace": ros_namespace
        },
    )
    return urdf_xml.toprettyxml()
