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



import pytest
import xml.etree.ElementTree as ET
from romea_rtls_transceiver_description import transceiver_urdf



pytest.fixture(scope="module")
def urdf_xml():
    prefix = "robot_"
    name = "transceiver0"
    type = "decawave"
    conf = "4GHz_6M8bit"
    mode = "10"
    pan_id = "0"
    id = "0 "
    parent_link = "base_link"
    xyz = [1.0, 2.0, 3.0]
    ros_namespace = "ns"
    return ET.fromstring(transceiver_urdf(prefix, name,
                                          type, conf, pan_id, id, mode,
                                          parent_link, xyz,
                                          ros_namespace))


def test_transceiver_name(urdf_xml):
    assert urdf_xml.find("link").get("name") == "robot_transceiver0_link"


def test_transceiver_position(urdf_xml):
    assert urdf_xml.find("joint/origin").get("xyz") == "1.0 2.0 3.0"


def test_transceiver_parent_link(urdf_xml):
    assert urdf_xml.find("joint/parent").get("link") == "robot_base_link"


def test_plugin_namespace(urdf_xml):
    assert urdf_xml.find("gazebo/plugin/ros/namespace").text == "ns"

# def test_gps_rate(urdf_xml):
#     assert urdf_xml.find("gazebo/sensor/update_rate").text == "10"
