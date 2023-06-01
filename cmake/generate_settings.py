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
from copy import deepcopy
import logging
import os
import re
import sys

# External imports
import xml.etree.ElementTree as et
import pystache

blacklist_settings_keys = set(["wifiSecurity"])
# From XML types to Dynamic Reconfigure Types
DYN_TYPE_MAP = {
    "bool": "bool_t",
    "u8": "int_t",
    "i8": "int_t",
    "u16": "int_t",
    "i16": "int_t",
    "u32": "int_t",
    "i32": "int_t",
    "u64": "int_t",
    "i64": "int_t",
    "float": "double_t",
    "double": "double_t",
    "string": "str_t",
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
C_TYPE_MAP = {
    "bool": "bool",
    "u8": "int32_t",
    "i8": "int32_t",
    "u16": "int32_t",
    "i16": "int32_t",
    "u32": "int32_t",
    "i32": "int32_t",
    "u64": "int32_t",
    "i64": "int32_t",
    "float": "double",  # for rosparam
    "double": "double",
    "string": "std::string",
    "enum": "int32_t",
}


def cap_word(text):
    return text.lower().title()


def is_settings_tag(name):
    return (not name.find("Settings") == -1) and (name.find("State") == -1)


def strip_text(text):
    return (
        re.sub("\s\s+", " ", text.strip().replace("\n", "").replace("\r", ""))
        .replace('"', "")
        .replace("'", "")
        .replace("\\n", "")
    )


def guess_min_max(arg_comment):
    min_max_regex = re.compile("\[([0-9\.\-]+)\:([0-9\.\-]+)\]")
    m = min_max_regex.search(arg_comment)
    if m:
        logging.info("  ... [min:max]")
        return [float(m.group(1)), float(m.group(2))]
    elif arg_comment.lower().find("m/s2") != -1:
        logging.info("  ... acc (m/s2)")
        return [0.0, 5.0]
    elif arg_comment.lower().find("m/s") != -1:
        logging.info("  ... speed (m/s)")
        return [0.0, 10.0]
    elif (arg_comment.lower().find("in meters") != -1) or (
        arg_comment.lower().find("in m") != -1
    ):
        logging.info("  ... meters")
        return [0, 160]
    elif arg_comment.lower().find("in degree/s") != -1:
        logging.info("  ... rotations speed degrees/s")
        return [0, 900.0]
    elif arg_comment.lower().find("in degree") != -1:
        logging.info("  ... degrees")
        return [-180.0, 180.0]
    elif (arg_comment.lower().find("1") != -1) and (
        arg_comment.lower().find("0") != -1
    ):
        logging.info("  ... bool")
        return [0, 1]
    elif arg_comment.lower().find("latitude") != -1:
        logging.info("  ... latitude")
        return [-90.0, 90.0]
    elif arg_comment.lower().find("longitude") != -1:
        logging.info("  ... longitude")
        return [-180.0, 180.0]
    elif arg_comment.lower().find("[rad/s]") != -1:
        logging.info("  ... angular speed (rad/s)")
        return [0.0, 5.0]
    elif arg_comment.lower().find("channel") != -1:
        logging.info("  ... unknown int")
        return [0, 50]
    elif arg_comment.lower().find("second") != -1:
        logging.info("  ... time (s)")
        return [0, 120]

    return []


def generate_settings(xml_filename, template_dir):
    rend = pystache.Renderer()
    project = os.path.basename(xml_filename).split(".")[0]
    with open(xml_filename, "r") as f:
        xml = f.read()
    xml_root = et.fromstring(xml)

    generator = os.path.basename(__file__)
    # generator_git_hash = subprocess.check_output(
    #     ["git", "rev-parse", "--short", "HEAD"]
    # ).strip()

    # make sure that the name of the config file matches the third argument
    # of gen.generate()
    d_cfg = dict(
        {
            "cfg_filename": "Bebop%s.cfg" % (project.title(),),
            # "url": xml_url,
            "project": project.title(),
            # "date": today(),
            "generator": generator,
            # "generator_git_hash": generator_git_hash,
            "cfg_class": list(),
            "cpp_class": list(),
        }
    )

    for cl in xml_root.iter("class"):
        if not is_settings_tag(cl.attrib["name"]):
            continue

        # At the moment the XML file is not 100% consistent between Settings and SettingsChanged and inner Commands
        # 1. Check if `class["name"]State` exists
        if not xml_root.findall(
            ".//class[@name='%s']" % (cl.attrib["name"] + "State",)
        ):
            logging.warning("No State Class for %s " % (cl.attrib["name"],))
            continue

        # Iterate all cmds
        # generate one C++ class for each command
        cfg_class_d = {
            "cfg_class_name": cl.attrib["name"].lower(),
            "cfg_class_comment": strip_text(cl.text),
            "cfg_cmd": list(),
        }
        for cmd in cl.iter("cmd"):
            # 2. Check if `cmd["name"]Changed` exists
            if not xml_root.findall(
                ".//cmd[@name='%s']" % (cmd.attrib["name"] + "Changed",)
            ):
                logging.warning("No Changed CMD for %s " % (cmd.attrib["name"],))
                continue

            # blacklist
            if strip_text(cmd.attrib["name"]) in blacklist_settings_keys:
                logging.warning("Key %s is blacklisted!" % (cmd.attrib["name"],))
                continue

            comment_el = cmd.find("comment")
            cmd_comment = ""
            if not comment_el is None:
                cmd_comment = comment_el.attrib["desc"]

            # .cfg
            cfg_cmd_d = {"cfg_cmd_comment": strip_text(cmd_comment), "cfg_arg": list()}

            # C++
            # We are iterating classes with names ending in "Setting". For each of these classes
            # there exists a corresponding class with the same name + "State" (e.g PilotingSetting and PilottingSettingState)
            # The inner commands of the corresponding class are also follow a similar conention, they end in "CHANGED".
            # We create cfg files based on Settings, and ROS param updates based on SettingsChanged
            cpp_class_dict_key = rend.render_path(
                "templates/dictionary_key.mustache",
                {
                    "project": project.upper(),
                    "class": cl.attrib["name"].upper() + "STATE",
                    "cmd": cmd.attrib["name"].upper() + "CHANGED",
                },
            )
            # cmd.attrib["name"] and cl.attrib["name"] are already in CamelCase
            cpp_class_name = cl.attrib["name"] + cmd.attrib["name"]
            cpp_class_comment = strip_text(cmd_comment)
            cpp_class_instance_name = (
                project.lower()
                + "_"
                + cl.attrib["name"].lower()
                + "_"
                + cmd.attrib["name"].lower()
                + "_ptr"
            )
            cpp_class_params = list()

            counter = 0
            # generate one dyamic reconfigure variable per arg
            for arg in cmd.iter("arg"):
                # .cfg
                arg_name = (
                    cl.attrib["name"]
                    + cmd.attrib["name"]
                    + cap_word(arg.attrib["name"])
                )
                arg_type = DYN_TYPE_MAP[arg.attrib.get("type", "bool")]
                arg_comment = strip_text(arg.text)

                arg_enum_list = list()
                minmax_list = list()
                arg_default = 0
                arg_min = 0.0
                arg_max = 0.0
                counter = 0
                need_enum_cast = False
                if arg_type == "enum":
                    need_enum_cast = True
                    arg_type = "int_t"
                    for enum in arg.iter("enum"):
                        arg_enum_list.append(
                            {
                                "constant_name": arg_name + "_" + enum.attrib["name"],
                                "constant_value": counter,
                                "constant_comment": strip_text(enum.text),
                            }
                        )
                        counter += 1
                elif not arg_type == "str_t":
                    # No min/max values defined in XML, guessing the type and propose a value:
                    logging.info('Guessing type of "%s"' % (arg_name))
                    logging.info("  from: %s" % (arg_comment))
                    minmax_list = guess_min_max(arg_comment)
                    if len(minmax_list) == 2:
                        [arg_min, arg_max] = minmax_list
                        logging.info("  min: %s max: %s" % (arg_min, arg_max))
                    else:
                        logging.warning(
                            "  Can not guess [min:max] values for this arg, skipping it"
                        )

                    # We create a fake enum for int_t types that only accept bool values
                    # The source XML should have defined them as bool_t
                    # Since these are fake enums (no defines in SDK), we don't need int->enum casting
                    if arg_type == "int_t" and arg_min == 0 and arg_max == 1:
                        arg_enum_list.append(
                            {
                                "constant_name": arg_name + "_OFF",
                                "constant_value": 0,
                                "constant_comment": "Disabled",
                            }
                        )
                        arg_enum_list.append(
                            {
                                "constant_name": arg_name + "_ON",
                                "constant_value": 1,
                                "constant_comment": "Enabled",
                            }
                        )
                        counter = 2

                # either we found minmax or the arg is of type enum
                if len(minmax_list) or need_enum_cast or arg_type == "str_t":
                    # hack
                    if arg_type == "str_t":
                        arg_min = "''"
                        arg_max = "''"
                        arg_default = "''"

                    cfg_cmd_d["cfg_arg"].append(
                        {
                            "cfg_arg_type": arg_type,
                            "cfg_arg_name": arg_name,
                            "cfg_arg_comment": arg_comment,
                            "cfg_arg_default": arg_default,
                            "cfg_arg_min": arg_min,
                            "cfg_arg_max": arg_max,
                            # Render once trick: http://stackoverflow.com/a/10118092
                            "cfg_arg_enum": {"items": deepcopy(arg_enum_list)}
                            if len(arg_enum_list)
                            else [],
                            "enum_max": counter - 1,
                        }
                    )

                    # generate c enum type
                    if need_enum_cast:
                        enum_cast = "static_cast<eARCOMMANDS_%s_%s_%s_%s>" % (
                            project.upper(),
                            cl.attrib["name"].upper(),
                            cmd.attrib["name"].upper(),
                            arg.attrib["name"].upper(),
                        )
                    else:
                        enum_cast = ""

                    cpp_class_params.append(
                        {
                            "cpp_class_arg_key": cpp_class_dict_key
                            + "_"
                            + arg.attrib["name"].upper(),
                            "cpp_class_param_name": arg_name,
                            "cpp_class_comment": cpp_class_comment,
                            "cpp_class_param_enum_cast": enum_cast,
                            "cpp_class_param_type": C_TYPE_MAP[
                                arg.attrib.get("type", "bool")
                            ],
                            "cpp_class_param_sdk_type": BEBOP_TYPE_MAP[
                                arg.attrib.get("type", "bool")
                            ],
                        }
                    )

            # Skip cmds with no arguments
            if len(cfg_cmd_d["cfg_arg"]):
                cfg_class_d["cfg_cmd"].append(deepcopy(cfg_cmd_d))
                d_cfg["cpp_class"].append(
                    {
                        "cpp_class_dict_key": cpp_class_dict_key,
                        "cpp_class_name": cpp_class_name,
                        "cpp_class_instance_name": cpp_class_instance_name,
                        "cpp_class_params": deepcopy(cpp_class_params),
                    }
                )

        d_cfg["cfg_class"].append(deepcopy(cfg_class_d))

    logging.info("... Done iterating, writing results to file")

    # .cfg write
    cfg_file_name = d_cfg["cfg_filename"]
    logging.info("Writing %s" % (cfg_file_name,))
    with open(cfg_file_name, "w") as cfg_file:
        cfg_file.write(rend.render_path("templates/cfg.mustache", d_cfg))

    header_file_name = "%s_setting_callbacks.h" % (project.lower(),)
    logging.info("Writing %s" % (header_file_name,))
    with open(header_file_name, "w") as header_file:
        header_file.write(
            rend.render_path("templates/setting_callbacks.h.mustache", d_cfg)
        )

    include_file_name = "%s_setting_callback_includes.h" % (project.lower(),)
    logging.info("Writing %s" % (include_file_name,))
    with open(include_file_name, "w") as include_file:
        include_file.write(
            rend.render_path("templates/setting_callback_includes.h.mustache", d_cfg)
        )

    rst_file_name = "%s_settings_param.rst" % (project.lower(),)
    logging.info("Writing %s" % (rst_file_name,))
    with open(rst_file_name, "w") as rst_file:
        rst_file.write(rend.render_path("templates/settings_param.rst.mustache", d_cfg))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage : {sys.argv[0]} <xmlfile> <templatedir>")
        raise RuntimeError("Missing input xml file to be processed")
    generate_settings(sys.argv[1], sys.argv[2])
