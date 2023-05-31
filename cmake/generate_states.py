#!/usr/bin/env python3
# coding: utf-8
"""
This is derived from the original code published https://github.com/AutonomyLab/bebop_autonomy and released under BSD-3 clause license


BSD 3-Clause License

Copyright (c) 2023, Jeremy Fix

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

# Standard imports
import sys
from copy import deepcopy
import logging
import os
import subprocess
import re
import pystache

# External imports
import xml.etree.ElementTree as et

# From XML types to ROS primitive types
ROS_TYPE_MAP = {
    "bool": "bool",
    "u8": "uint8",
    "i8": "int8",
    "u16": "uint16",
    "i16": "int16",
    "u32": "uint32",
    "i32": "int32",
    "u64": "uint64",
    "i64": "int64",
    "float": "float32",
    "double": "float64",
    "string": "string",
    "enum": "enum",
}

# From XML types to BebopSDK union defined in ARCONTROLLER_Dictionary.h
BEBOP_TYPE_MAP = {
    "bool": "U8",
    "u8": "U8",
    "i8": "I8",
    "u16": "U16",
    "i16": "I16",
    "u32": "U32",
    "i32": "I32",
    "u64": "U64",
    "i64": "I64",
    "float": "Float",
    "double": "Double",
    "string": "String",
    "enum": "I32",
}


def is_state_tag(name):
    return (not name.find("State") == -1) and (name.find("Settings") == -1)


def cap_word(text):
    return text.lower().title()


def strip_text(text):
    return (
        re.sub("\s\s+", " ", text.strip().replace("\n", "").replace("\r", ""))
        .replace('"', "")
        .replace("'", "")
    )


def generate_states(xml_filename, template_dir):
    rend = pystache.Renderer()

    project = os.path.basename(xml_filename).split(".")[0]
    with open(xml_filename, "r") as f:
        xml = f.read()
    xml_root = et.fromstring(xml)

    # iterate all <class> tags
    logging.info("Iterating all State <class> tags ...")

    generator = os.path.basename(__file__)
    generator_git_hash = subprocess.check_output(
        ["git", "rev-parse", "--short", "HEAD"]
    ).strip()

    d_cpp = dict(
        {
            # "url": xml_url,
            # "project": project,
            # "date": today(),
            "generator": generator,
            "generator_git_hash": generator_git_hash,
            "queue_size": 10,  # 5Hz
            "frame_id": "base_link",
            "cpp_class": list(),
        }
    )
    d_msg = dict()

    for cl in xml_root.iter("class"):
        if not is_state_tag(cl.attrib["name"]):
            continue

        # Iterate all cmds
        # Generate one .msg and one C++ class for each of them
        for cmd in cl.iter("cmd"):
            # .msg
            msg_name = cap_word(project) + cl.attrib["name"] + cmd.attrib["name"]

            comment_el = cmd.find("comment")
            msg_file_comment = ""
            if comment_el is not None:
                msg_file_comment = comment_el.attrib["desc"]

            d = dict(
                {
                    # "url": xml_url,
                    "msg_filename": msg_name,
                    # "date": today(),
                    # "generator": generator,
                    # "generator_git_hash": generator_git_hash,
                    "msg_file_comment": strip_text(msg_file_comment),
                    "msg_field": list(),
                }
            )

            # C++ class
            cpp_class_dict_key = rend.render_path(
                f"{template_dir}/dictionary_key.mustache",
                {
                    "project": project.upper(),
                    "class": cl.attrib["name"].upper(),
                    "cmd": cmd.attrib["name"].upper(),
                },
            )
            # cmd.attrib["name"] and cl.attrib["name"] are already in CamelCase
            cpp_class_name = msg_name
            cpp_class_instance_name = (
                project.lower()
                + "_"
                + cl.attrib["name"].lower()
                + "_"
                + cmd.attrib["name"].lower()
                + "_ptr"
            )
            cpp_class_param_name = (
                "states/enable_"
                + cl.attrib["name"].lower()
                + "_"
                + cmd.attrib["name"].lower()
            )
            topic_name = (
                "states/" + project + "/" + cl.attrib["name"] + "/" + cmd.attrib["name"]
            )

            arg_list = []
            for arg in cmd.iter("arg"):
                # .msg
                f_name = arg.attrib["name"]
                f_type = ROS_TYPE_MAP[arg.attrib.get("type", "bool")]
                f_comment = strip_text(arg.text)
                f_enum_list = list()
                if f_type == "enum":
                    f_type = "uint8"
                    counter = 0
                    for enum in arg.iter("enum"):
                        f_enum_list.append(
                            {
                                "constant_name": f_name + "_" + enum.attrib["name"],
                                "constant_value": counter,
                                "constant_comment": strip_text(enum.text),
                            }
                        )
                        counter += 1

                d["msg_field"].append(
                    {
                        "msg_field_type": f_type,
                        "msg_field_name": f_name,
                        "msg_field_comment": f_comment,
                        "msg_field_enum": deepcopy(f_enum_list),
                    }
                )

                # C++ class
                arg_list.append(
                    {
                        "cpp_class_arg_key": cpp_class_dict_key
                        + "_"
                        + arg.attrib["name"].upper(),
                        "cpp_class_arg_name": f_name,
                        "cpp_class_arg_sdk_type": BEBOP_TYPE_MAP[
                            arg.attrib.get("type", "bool")
                        ],
                    }
                )

            d_msg[msg_name] = deepcopy(d)

            # C++ class
            d_cpp["cpp_class"].append(
                {
                    "cpp_class_name": cpp_class_name,
                    "cpp_class_comment": strip_text(msg_file_comment),
                    "cpp_class_instance_name": cpp_class_instance_name,
                    "cpp_class_param_name": cpp_class_param_name,
                    "topic_name": topic_name,
                    "latched": "true",
                    "cpp_class_msg_type": msg_name,
                    "key": cpp_class_dict_key,
                    "cpp_class_arg": deepcopy(arg_list),
                }
            )

    logging.info("... Done iterating, writing results to file")

    # Generating the target directories
    msg_dir = "msg"
    if not os.path.exists(msg_dir):
        os.makedirs(msg_dir)
    include_dir = "include/ros2_bebop_driver"
    if not os.path.exists(include_dir):
        os.makedirs(include_dir)

    # .msg write
    for k, d in d_msg.items():
        msg_filename = f"{msg_dir}/{k}.msg"
        logging.info("Writing %s" % (msg_filename,))
        with open(msg_filename, "w") as msg_file:
            msg_file.write(rend.render_path(f"{template_dir}/msg.mustache", d))

    header_file_name = f"{include_dir}/{project.lower()}_state_callbacks.h"
    logging.info("Writing %s" % (header_file_name,))
    with open(header_file_name, "w") as header_file:
        header_file.write(
            rend.render_path(f"{template_dir}/state_callbacks.h.mustache", d_cpp)
        )

    include_file_name = f"{include_dir}/{project.lower()}_state_callback_includes.h"
    logging.info("Writing %s" % (include_file_name,))
    with open(include_file_name, "w") as include_file:
        include_file.write(
            rend.render_path(
                f"{template_dir}/state_callback_includes.h.mustache", d_cpp
            )
        )

    with open(f"{include_dir}/callbacks_common.h", "w") as header_file:
        header_file.write(
            rend.render_path(f"{template_dir}/callbacks_common.h.mustache", d_cpp)
        )

    rst_file_name = "%s_states_param_topic.rst" % (project.lower(),)
    logging.info("Writing %s" % (rst_file_name,))
    with open(rst_file_name, "w") as rst_file:
        rst_file.write(
            rend.render_path(f"{template_dir}/states_param_topic.rst.mustache", d_cpp)
        )


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage : {sys.argv[0]} <xmlfile> <templatedir>")
        raise RuntimeError("Missing input xml file to be processed")
    generate_states(sys.argv[1], sys.argv[2])
