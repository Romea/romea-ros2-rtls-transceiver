# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import xacro

from ament_index_python.packages import get_package_share_directory


def transceiver_urdf(
        prefix, mode, name,
        type, configuration, pan_id, id, control_mode,
        parent_link, xyz, ros_namespace):

    xacro_file = (
        get_package_share_directory("romea_rtls_transceiver_description")
        + "/urdf/transceiver_"
        + type
        + "_"
        + configuration
        + ".xacro.urdf"
    )

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "name": name,
            "control_mode": control_mode,
            "pan_id": str(pan_id),
            "id": str(id),
            "parent_link": parent_link,
            "xyz": " ".join(map(str, xyz)),
            "ros_namespace": ros_namespace
        },
    )
    return urdf_xml.toprettyxml()


def anchor_urdf(name, type, configuration, pan_id, id):

    xacro_file = (
        get_package_share_directory("romea_rtls_transceiver_description")
        + "/urdf/anchor_"
        + type
        + "_"
        + configuration
        + ".xacro.urdf"
    )

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "name": name,
            "pan_id": str(pan_id),
            "id": str(id),
        },
    )
    return urdf_xml.toprettyxml()
