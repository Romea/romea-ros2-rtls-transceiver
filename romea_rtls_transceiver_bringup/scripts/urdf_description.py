#!/usr/bin/env python3

# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

from romea_common_bringup import robot_urdf_prefix
from romea_rtls_transceiver_bringup import urdf_description
import sys

if __name__ == "__main__":

    argv = sys.argv

    parameters = {}
    for argument in argv[1:]:
        name, value = argument.split(":")
        parameters[name] = value

    if not parameters["robot_namespace"]:
        prefix = ""
    else:
        prefix = parameters["robot_namespace"] + "_"

    prefix = robot_urdf_prefix(parameters["robot_namespace"])
    meta_description_filename = parameters["meta_description_filename"]
    print(urdf_description(prefix, meta_description_filename))
