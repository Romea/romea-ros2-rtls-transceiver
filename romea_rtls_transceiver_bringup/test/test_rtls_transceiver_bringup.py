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



import os
import pytest

from romea_rtls_transceiver_bringup import RTLSTransceiverMetaDescription



pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(os.getcwd(), "test_rtls_transceiver_bringup.yaml")
    return RTLSTransceiverMetaDescription(meta_description_file_path)


def test_get_name(meta_description):
    assert meta_description.get_name() == "transceiver"


def test_get_namespace(meta_description):
    assert meta_description.get_namespace() == "rtls"


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
