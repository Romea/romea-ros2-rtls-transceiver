# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

from romea_common_bringup import MetaDescription
import romea_rtls_transceiver_description


class RTLSTransceiverMetaDescription:
    def __init__(self, meta_description_filename):
        self.meta_description = MetaDescription("rtls_transceiver", meta_description_filename)

    def get_name(self):
        return self.meta_description.get("name")

    def has_driver_configuration(self):
        return self.meta_description.exists("driver")

    def get_driver_pkg(self):
        return self.meta_description.get("pkg", "driver")

    def get_driver_device(self):
        return self.meta_description.get("device", "driver")

    def get_driver_baudrate(self):
        return self.meta_description.get("baudrate", "driver")

    def get_type(self):
        return self.meta_description.get("type", "configuration")

    def get_communication(self):
        return self.meta_description.get("communication", "configuration")

    def get_id(self):
        return self.meta_description.get("id", "configuration")

    def get_pan_id(self):
        return self.meta_description.get("pan_id", "configuration")

    def get_mode(self):
        return self.meta_description.get("mode", "configuration")

    def get_parent_link(self):
        return self.meta_description.get("parent_link", "geometry")

    def get_xyz(self):
        return self.meta_description.get("xyz", "geometry")


def urdf_description(prefix, meta_description_filename):

    meta_description = RTLSTransceiverMetaDescription(meta_description_filename)

    return romea_rtls_transceiver_description.urdf(
        prefix,
        meta_description.get_name(),
        meta_description.get_type(),
        meta_description.get_communication(),
        meta_description.get_pan_id(),
        meta_description.get_id(),
        meta_description.get_mode(),
        meta_description.get_parent_link(),
        meta_description.get_xyz(),
    )
