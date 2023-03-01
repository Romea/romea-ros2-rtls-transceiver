# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license


import os
import pytest

from romea_rtls_transceiver_bringup import RTLSTransceiverMetaDescription


@pytest.fixture(scope="module")
def meta_description():
    meta_description_filename = os.path.join(os.getcwd(), "test_rtls_transceiver_bringup.yaml")
    return RTLSTransceiverMetaDescription(meta_description_filename)


def test_get_name(meta_description):
    assert meta_description.get_name() == "transceiver"


def test_has_driver_configuration(meta_description):
    assert meta_description.has_driver_configuration() is True


def test_get_driver_pkg(meta_description):
    assert meta_description.get_driver_pkg() == "romea_trackbod_driver"


def test_get_driver_device(meta_description):
    assert meta_description.get_driver_device() == "/dev/ttyACM0"


def test_get_driver_baudrate(meta_description):
    assert meta_description.get_driver_baudrate() == 921600


def test_get_type(meta_description):
    assert meta_description.get_type() == "decawave"


def test_get_communication(meta_description):
    assert meta_description.get_communication() == "4GHz_6M8bit"


def test_get_id(meta_description):
    assert meta_description.get_id() == 0


def test_get_pan_id(meta_description):
    assert meta_description.get_pan_id() == 1


def test_get_mode(meta_description):
    assert meta_description.get_mode() == "slave"


def test_get_parent_link(meta_description):
    assert meta_description.get_parent_link() == "base_link"


def test_get_xyz(meta_description):
    assert meta_description.get_xyz() == [1.0, 2.0, 3.0]
