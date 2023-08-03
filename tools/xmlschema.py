#!/usr/bin/env python3
# Copyright 2023 Open Source Robotics Foundation, Inc.
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

"""
Conversion script for SDF definitions to XML XSD files
"""

from xml.etree import ElementTree

import os

from typing import List, Dict, Tuple, Optional


# Mapping between "type" values found in SDF files to the corresponding
# XSD standard datatypes as defined by https://www.w3.org/TR/xmlschema11-2/
SDF_TYPES_TO_XSD_STD_TYPES = {
    "bool": "boolean",
    "char": "char",
    "int": "int",
    "double": "double",
    "float": "float",
    "string": "string",
    "unsigned int": "unsignedInt",
    "unsigned long": "unsignedLong",
}

# Mapping between "required" values found in SDF files to the corresponding
# minOccurs and maxOccurs found in XSD
SDF_REQUIRED_TO_MIN_MAX_OCCURS: Dict[str, Tuple[str, str]] = {
    "0": ("0", "1"),  # Required: 0,  (minOccurs: 0, maxOccurs: 1)
    "1": ("1", "1"),  # Required: 1,  (minOccurs: 1, maxOccurs: 1)
    "+": ("1", "unbounded"),  # Required: +,  (minOccurs: 1, maxOccurs: inf)
    "*": ("0", "unbounded"),  # Required: *,  (minOccurs: 0, maxOccurs: inf)
    "-1": ("0", "unbounded"),  # Required: -1, (minOccurs: 0, maxOccurs: inf)
}


def indent_lines(lines: List[str], indent: int) -> List[str]:
    """
    Indent a list of xml lines group of lines by a number (indent) of spaces

    """
    return [" " * indent + line for line in lines]


def get_attribute(element: ElementTree.Element, attrib: str) -> Optional[str]:
    """
    Retrieve XML attribute from an element
    """
    return element.attrib[attrib] if attrib in element.attrib else None


def is_std_type(sdf_type: str) -> bool:
    """
    Check if sdf_type is a known XSD standard type.
    Return true if the sdf_type is in the set of known types, false otherwise.
    """
    return sdf_type in SDF_TYPES_TO_XSD_STD_TYPES


def xsd_type_string(sdf_type: str) -> Optional[str]:
    """
    Check if xsd_type is a known XSD standard type.
    If it is, return 'xsd:' + type, None otherwise.
    """
    if is_std_type(sdf_type):
        xsd_type = SDF_TYPES_TO_XSD_STD_TYPES[sdf_type]
        return "xsd:" + xsd_type
    return None


def print_documentation(element: ElementTree.Element) -> List[str]:
    """
    Print the documentation associated with an element
    """
    lines = []
    description = element.find("description")
    if (
        description is not None
        and description.text is not None
        and len(description.text)
    ):
        lines.append("<xsd:annotation>")
        lines.append("  <xsd:documentation xml:lang='en'>")
        lines.append(f"    <![CDATA[{description.text}]]>")
        lines.append("  </xsd:documentation>")
        lines.append("</xsd:annotation>")
    return lines


def print_include(element: ElementTree.Element) -> List[str]:
    """
    Print include tag information
    """
    lines = []
    filename = get_attribute(element, "filename")
    if filename is not None:
        loc = "http://sdformat.org/schemas/"
        loc += filename.replace(".sdf", ".xsd")
        lines.append(f"<xsd:include schemaLocation='{loc}'/>")
    return lines


def print_include_ref(element: ElementTree.Element, sdf_root_dir: str) -> List[str]:
    """
    Print include tag reference information
    """
    lines = []
    filename = get_attribute(element, "filename")
    if filename is not None:
        sdf_path = os.path.join(sdf_root_dir, filename)
        if not os.path.exists(sdf_path):
            raise RuntimeError(f"Attempted to include non-existent file: {sdf_path}")

        include_tree = ElementTree.parse(sdf_path)
        root = include_tree.getroot()
        include_element_name = root.attrib["name"]
        lines.append(f"<xsd:element ref='{include_element_name}'/>")
    return lines


def print_plugin_element(element: ElementTree.Element) -> List[str]:
    """
    Separate handling of the 'plugin' element
    """
    lines = []
    # Short circuit for plugin.sdf copy_data element
    if "copy_data" in element.attrib:
        lines.append("<xsd:sequence>")
        lines.append(
            "  <xsd:any minOccurs='0' maxOccurs='unbounded' processContents='lax'/>"
        )
        lines.append("</xsd:sequence>")
    return lines


def print_element(element: ElementTree.Element) -> List[str]:
    """
    Print a child element of the sdf definition
    """
    lines = []

    elem_name = get_attribute(element, "name")
    elem_type = get_attribute(element, "type")
    elem_reqd = get_attribute(element, "required")

    if elem_type and is_std_type(elem_type):
        elem_type = xsd_type_string(elem_type)

    if not elem_reqd:
        raise RuntimeError("Cannot process element missing 'required' attribute")

    min_occurs, max_occurs = SDF_REQUIRED_TO_MIN_MAX_OCCURS[elem_reqd]
    lines.append(f"<xsd:choice  minOccurs='{min_occurs}' maxOccurs='{max_occurs}'>")

    if elem_type is None:
        lines.append(f"<xsd:element name='{elem_name}'>")
        lines.extend(indent_lines(print_documentation(element), 2))
        lines.append("  <xsd:complexType>")
        lines.append("    <xsd:choice maxOccurs='unbounded'>")

        for child_element in element.findall("element"):
            lines.extend(indent_lines(print_element(child_element), 6))

        lines.append("    </xsd:choice>")

        for attribute in element.findall("attribute"):
            lines.extend(indent_lines(print_attribute(attribute), 4))

        lines.append("  </xsd:complexType>")
    else:
        lines.append(f"<xsd:element name='{elem_name}' type='{elem_type}'>")
        lines.extend(indent_lines(print_documentation(element), 2))

    lines.append("</xsd:element>")
    lines.append("</xsd:choice>")
    return lines


def print_attribute(element: ElementTree.Element) -> List[str]:
    """
    Print an attribute of the sdf definition
    """
    lines = []

    elem_name = get_attribute(element, "name")
    elem_type = get_attribute(element, "type")
    elem_reqd = get_attribute(element, "required")
    elem_default = get_attribute(element, "default")

    if elem_type and is_std_type(elem_type):
        elem_type = xsd_type_string(elem_type)

    use = ""
    default = ""

    if elem_reqd == "1":
        use = "use='required'"
    elif elem_reqd == "0":
        use = "use='optional'"
        if elem_default is not None:
            default = f"default='{elem_default}'"

    lines.append(
        f"<xsd:attribute name='{elem_name}' type='{elem_type}' {use} {default}>"
    )
    lines.extend(indent_lines(print_documentation(element), 2))
    lines.append("</xsd:attribute>")
    return lines


def print_xsd(element: ElementTree.Element, sdf_root_dir: str) -> List[str]:
    """
    Print xsd for top level SDF element
    """
    lines = []

    elem_name = get_attribute(element, "name")
    elem_type = get_attribute(element, "type")

    elements = element.findall("element")
    attributes = element.findall("attribute")
    includes = element.findall("include")

    lines.extend(print_documentation(element))
    lines.append(
        "<xsd:include schemaLocation='http://sdformat.org/schemas/types.xsd'/>"
    )

    # Reference any includes in the SDF file
    for include in includes:
        lines.extend(print_include(include))

    if len(elements) or len(attributes) or len(includes):
        lines.append(f"<xsd:element name='{elem_name}'>")
        lines.append("  <xsd:complexType>")

        if elem_name != "plugin" and (len(elements) or len(includes)):
            lines.append("    <xsd:choice maxOccurs='unbounded'>")

        for child_element in elements:
            if "copy_data" in child_element.attrib:
                element_lines = print_plugin_element(child_element)
                lines.extend(indent_lines(element_lines, 4))
            else:
                element_lines = print_element(child_element)
                lines.extend(indent_lines(element_lines, 6))

        for include_element in includes:
            element_lines = print_include_ref(include_element, sdf_root_dir)
            lines.extend(indent_lines(element_lines, 6))

        if elem_name != "plugin" and (len(elements) or len(includes)):
            lines.append("    </xsd:choice>")

        for attribute_element in attributes:
            lines.extend(indent_lines(print_attribute(attribute_element), 4))

        lines.append("  </xsd:complexType>")
        lines.append("</xsd:element>")
    else:
        if elem_type and is_std_type(elem_type):
            elem_type = f' type={xsd_type_string(elem_type)}'
        else:
            elem_type = ""

        lines.append(f"<xsd:element name='{elem_name}'{elem_type} />")
    return lines


def process(input_file_sdf: str, sdf_dir: str) -> List[str]:
    """
    Produce an XSD file from an input SDF file
    """
    lines = []
    tree = ElementTree.parse(input_file_sdf)
    root = tree.getroot()
    lines.append("<?xml version='1.0' encoding='UTF-8'?>")
    lines.append("<xsd:schema xmlns:xsd='http://www.w3.org/2001/XMLSchema'>")
    lines.extend(indent_lines(print_xsd(root, sdf_dir), 2))
    lines.append("</xsd:schema>")
    return lines


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser("xmlschema.py")
    parser.add_argument("--input-file")
    parser.add_argument("--sdf-dir")
    parser.add_argument("--output-dir")
    args = parser.parse_args()

    input_file = os.path.abspath(args.input_file)
    output_lines = process(input_file, args.sdf_dir)
    fname = os.path.splitext(os.path.basename(args.input_file))[0]
    os.makedirs(args.output_dir, exist_ok=True)

    output_file = os.path.join(args.output_dir, f"{fname}.xsd")

    with open(output_file, "w", encoding="utf8") as f:
        f.write("\n".join(output_lines))
        f.write("\n")
