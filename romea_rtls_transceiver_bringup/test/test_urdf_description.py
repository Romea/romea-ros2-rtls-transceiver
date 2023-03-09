# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import os
import pytest
import subprocess

from ament_index_python import get_package_prefix

import xml.etree.ElementTree as ET


@pytest.fixture(scope="module")
def urdf():

    exe = (
        get_package_prefix("romea_rtls_transceiver_bringup")
        + "/lib/romea_rtls_transceiver_bringup/urdf_description.py"
    )

    meta_description_filename = os.path.join(os.getcwd(), "test_rtls_transceiver_bringup.yaml")

    return ET.fromstring(
        subprocess.check_output(
            [
                exe,
                "robot_namespace:robot",
                "meta_description_filename:" + meta_description_filename,
            ],
            encoding="utf-8",
        )
    )


def test_rtls_transceiver_name(urdf):
    assert urdf.find("link").get("name") == "robot_transceiver_link"


def test_rtls_transceiver_position(urdf):
    assert urdf.find("joint/origin").get("xyz") == "1.0 2.0 3.0"


def test_rtls_transceiver_parent_link(urdf):
    assert urdf.find("joint/parent").get("link") == "robot_base_link"


def test_plugin_namespace(urdf):
    assert urdf.find("gazebo/plugin/ros/namespace").text == "/robot/rtls/transceiver"
