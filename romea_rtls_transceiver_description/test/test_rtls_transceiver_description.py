# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license


import pytest
import xml.etree.ElementTree as ET
from romea_rtls_transceiver_description import urdf


@pytest.fixture(scope="module")
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
    return ET.fromstring(urdf(prefix, name, type, conf, pan_id, id, mode, parent_link, xyz))


def test_transceiver_name(urdf_xml):
    assert urdf_xml.find("link").get("name") == "robot_transceiver0_link"


def test_transceiver_position(urdf_xml):
    assert urdf_xml.find("joint/origin").get("xyz") == "1.0 2.0 3.0"


def test_transceiver_parent_link(urdf_xml):
    assert urdf_xml.find("joint/parent").get("link") == "robot_base_link"


# def test_gps_rate(urdf_xml):
#     assert urdf_xml.find("gazebo/sensor/update_rate").text == "10"
