from xml.etree import ElementTree
from copy import deepcopy
from itertools import permutations, zip_longest

from pathlib import Path
import argparse
from typing import List


cmd_arg_parser = argparse.ArgumentParser(
    description="Create an XML schema from a SDFormat file."
)
cmd_arg_parser.add_argument(
    "-i",
    "--in",
    dest="source",
    metavar='FILE',
    required=True,
    type=str,
    help="SDF file inside of directory to compile.",
)
cmd_arg_parser.add_argument(
    "-s",
    "--sdf",
    dest="directory",
    required=True,
    type=str,
    help="Directory containing all the SDF files.",
)
cmd_arg_parser.add_argument(
    "-o",
    "--out",
    dest="target",
    default=".",
    type=str,
    help="Output directory for XSD file. Will be created if it doesn't exist.",
)
cmd_arg_parser.add_argument(
    "--ns-prefix",
    dest="ns_prefix",
    default="http://sdformat.org/schema",
    type=str,
    help="Prefix for generated xsd namespaces.",
)

args = cmd_arg_parser.parse_args()
source_dir = Path(args.directory)
source: Path = source_dir / args.source

root = ElementTree.parse(source).getroot()
declared = dict()
declared["imports"] = set()


def _to_simple_type(in_type: str) -> str:
"""
Converts the input SDF type string to a XSD type string

:param arg_type: input type
:raise: RuntimeError if no mapping exists
:returns: converted XSD string
"""
    known_types = {
        "unsigned int": "xs:unsignedInt",
        "unsigned long": "xs:unsignedLong",
        "bool": "xs:boolean",
        "string": "xs:string",
        "double": "xs:double",
        "int": "xs:int",
        "float": "xs:float",
        "char": "xs:char",
        "vector3": "types:vector3",
        "vector2d": "types:vector2d",
        "vector2i": "types:vector2i",
        "pose": "types:pose",
        "time": "types:time",
        "color": "types:color",
    }

    try:
        return known_types[in_type]
    except KeyError:
        raise RuntimeError(f"Unknown simple type: {in_type}") from None


# collect existing namespaces
namespaces = {
    "types": "http://sdformat.org/schemas/types.xsd",
    "xs": "http://www.w3.org/2001/XMLSchema",
}
for path in source_dir.iterdir():
    if not path.is_file():
        continue

    if not path.suffix == ".sdf":
        continue

    namespaces[path.stem] = f"{args.ns_prefix}/{path.stem}"
    ElementTree.register_namespace(path.stem, f"{args.ns_prefix}/{path.stem}")


def _to_qname(name: str) -> str:
    try:
        prefix, name = name.split(":")
    except ValueError:
        # no prefix
        return name
    else:
        return f"{{{namespaces[prefix]}}}{name}"


class Description:
    def __init__(self, element: ElementTree.Element):
        self.element = element

    def to_subtree(self):
        desc_text = self.element.text
        if desc_text is None or desc_text == "":
            return list(), None

        desc_text = self.element.text.strip()
        desc_text.replace("\n", " ")

        documentation = ElementTree.Element(_to_qname("xs:documentation"))
        documentation.text = desc_text
        documentation.set("xml:lang", "en")
        annotation = ElementTree.Element(_to_qname("xs:annotation"))
        annotation.append(documentation)
        return list(), annotation


class Element:
    def __init__(self, element: ElementTree.Element):
        self.element = element

    def to_basic(self):
        namespaces = list()
        el = ElementTree.Element(_to_qname("xs:element"))
        el.set("name", self.element.attrib["name"])
        if "type" in self.element.attrib:
            el.set("type", self.element.attrib["type"])
        if "default" in self.element.attrib:
            el.set("default", self.element.attrib["default"])

        return namespaces, el

    def to_subtree(self):
        namespaces = list()

        if "copy_data" in self.element.attrib:
            # special element for plugin.sdf to
            # declare that any children are allowed
            anyEl = ElementTree.Element(_to_qname("xs:any"))
            anyEl.set("minOccurs", "0")
            anyEl.set("maxOccurs", "unbounded")
            anyEl.set("processContents", "skip")

            docs = self.element.find("description")
            if docs is not None:
                doc_namespaces, doc_el = Description(docs).to_subtree()
                namespaces.extend(doc_namespaces)
                if doc_el:
                    anyEl.append(doc_el)

            return namespaces, anyEl

        el = ElementTree.Element(_to_qname("xs:element"))
        el.set("name", self.element.attrib["name"])

        docs = self.element.find("description")
        if docs is not None:
            doc_namespaces, doc_el = Description(docs).to_subtree()
            namespaces.extend(doc_namespaces)
            if doc_el:
                el.append(doc_el)

        num_children = len(self.element) - len(self.element.findall("description"))
        has_children = num_children > 0
        has_type = "type" in self.element.attrib
        has_ref = "ref" in self.element.attrib

        if has_ref:
            # I couldn't quite work out how this tag is supposed to work
            # it appears to refer to the file from which the root element
            # should be used to define this element. This seems equivalent
            # to <include> though, so I am at a loss.
            # This is a best guess implementation.

            other_file = source_dir / (self.element.attrib["ref"] + ".sdf")
            other_root = ElementTree.parse(other_file).getroot()
            name = other_root.attrib["name"]

            child_ns, child_el = ComplexType(self.element).to_subtree()
            el.set("type", f"{name}Type")

        elif has_type and has_children:
            child_ns, child_el = ComplexType(self.element).to_subtree()
            namespaces.extend(child_ns)
            el.append(child_el)
        elif has_children:
            child_ns, child_el = ComplexType(self.element).to_subtree()
            namespaces.extend(child_ns)
            el.append(child_el)
        elif has_type:
            child_type = _to_simple_type(self.element.attrib["type"])
            el.set("type", child_type)
            if child_type.startswith("types"):
                namespaces.append("types")
        else:
            el.set("type", _to_simple_type("string"))

        if "default" in self.element.attrib:
            el.set("default", self.element.attrib["default"])

        required_codes = {
            "0": ("0", "1"),
            "1": ("1", "1"),
            "+": ("1", "unbounded"),
            "*": ("0", "unbounded"),
            "-1": ("0", "0"),
        }
        min_occurs, max_occurs = required_codes[self.element.attrib["required"]]
        el.set("minOccurs", min_occurs)
        el.set("maxOccurs", max_occurs)

        return namespaces, el


class Include:
    def __init__(self, element: ElementTree.Element):
        self.element = element

    def to_subtree(self):
        namespaces = list()
        other_file = source_dir / self.element.attrib["filename"]
        other_root = ElementTree.parse(other_file).getroot()
        name = other_root.attrib["name"]

        el = ElementTree.Element(_to_qname("xs:element"))
        el.set("name", other_root.attrib["name"])

        el.set("type", f"{other_file.stem}:{name}Type")
        namespaces.append(f"{other_file.stem}")

        if "default" in other_root.attrib:
            el.set("default", other_root.attrib["default"])

        docs = other_root.find("description")
        if docs is not None:
            doc_namespaces, doc_el = Description(docs).to_subtree()
            namespaces.extend(doc_namespaces)
            if doc_el:
                el.append(doc_el)

        if self.element.attrib["required"] == "1":
            el.set("type", f"{other_file.stem}:{name}Type")

        # TODO: remove this line
        declared.setdefault("imports", set()).add(other_file.stem)

        required_codes = {
            "0": ("0", "1"),
            "1": ("1", "1"),
            "+": ("1", "unbounded"),
            "*": ("0", "unbounded"),
            "-1": ("0", "0"),
        }
        min_occurs, max_occurs = required_codes[self.element.attrib["required"]]
        el.set("minOccurs", min_occurs)
        el.set("maxOccurs", max_occurs)

        return namespaces, el


class Attribute:
    def __init__(self, element: ElementTree.Element):
        self.element = element

    def to_subtree(self):
        namespaces = list()
        el = ElementTree.Element(_to_qname("xs:attribute"))

        docs = self.element.find("description")
        if docs is not None:
            doc_namespaces, doc_el = Description(docs).to_subtree()
            namespaces.extend(doc_namespaces)
            if doc_el:
                el.append(doc_el)

        el.set("name", self.element.attrib["name"])

        el_type = _to_simple_type(self.element.attrib["type"])
        el.set("type", el_type)
        if el_type.startswith("types"):
            namespaces.append("types")

        required = "required" if self.element.attrib["required"] == "1" else "optional"
        el.set("use", required)

        if required == "optional" and "default" in self.element.attrib:
            el.set("default", self.element.attrib["default"])

        return namespaces, el


class ComplexType:
    def __init__(self, element: ElementTree.Element, name: str = None):
        self.element = element
        self.name = name

    def to_subtree(self):
        namespaces = list()
        elements = list()
        attributes = list()

        for attribute in self.element.findall("attribute"):
            child_ns, child_el = Attribute(attribute).to_subtree()
            attributes.append(child_el)
            namespaces += child_ns

        for child in self.element.findall("element"):
            child_ns, child_el = Element(child).to_subtree()
            elements.append(child_el)
            namespaces += child_ns

        for child in self.element.findall("include"):
            child_ns, child_el = Include(child).to_subtree()
            elements.append(child_el)
            namespaces += child_ns

        el = ElementTree.Element(_to_qname("xs:complexType"))

        if self.name:
            el.set("name", self.name)

        docs = self.element.find("description")
        if docs is not None:
            doc_namespaces, doc_el = Description(docs).to_subtree()
            namespaces.extend(doc_namespaces)
            if doc_el:
                el.append(doc_el)

        if elements and "type" in self.element.attrib:
            raise NotImplementedError(
                "Cant handle sub-elements for an element declaring a type."
            )
        elif "type" in self.element.attrib:
            el_type = _to_simple_type(self.element.attrib["type"])
            extension = ElementTree.Element(_to_qname("xs:extension"))
            extension.set("base", el_type)
            if el_type.startswith("types"):
                namespaces.append("types")
            extension.extend(attributes)
            simple_content = ElementTree.Element(_to_qname("xs:simpleContent"))
            simple_content.append(extension)
            el.append(simple_content)
        elif elements:
            # drop depreciated elements
            elements = [el for el in elements if not el.attrib["maxOccurs"] == "0"]
            unbounded_elements = [el for el in elements if el.attrib["maxOccurs"] == "unbounded"]

            if not unbounded_elements:
                seq_container = ElementTree.Element(_to_qname("xs:all"))
                seq_container.extend(elements)
                el.append(seq_container)
                el.extend(attributes)
            else:
                # XSD 1.1 allows maxOccurs="unbound" within xs:all, so 
                # we could drop this entire else block if we could upgrade
                # to XSD 1.1. (xmllint only supports XSD 1.0)
                
                # sort all elements into two categories: 
                # 1. maxOccurs <= 1 
                # 2. maxOccurs = unbounded and minOccurs = 0
                # the case maxOccurs = unbounded and minOccurs = 1 is refactored
                # into 1 required element in case 1 + 1 optional element in case 2
                # bounded_elements = [el for el in elements if not el.attrib["maxOccurs"] == "unbounded"]
                # optional_unbounded = list()
                # for unbounded_element in [el for el in elements if el.attrib["maxOccurs"] == "unbounded"]:
                #     unbounded_element:ElementTree.Element
                #     min_occurs = unbounded_element.attrib["minOccurs"]
                #     if min_occurs == "1":
                #         required_item = deepcopy(unbounded_element)
                #         required_item.set("maxOccurs", "1")
                #         bounded_elements.append(required_item)
                    
                #     # it will be ugly, but we can try to reduce clutter
                #     unbounded_element.attrib.pop("minOccurs")
                #     unbounded_element.attrib.pop("maxOccurs")
                #     optional_unbounded.append(unbounded_element)

                # infinity_choice = ElementTree.Element(_to_qname("xs:choice"))
                # infinity_choice.set("maxOccurs", "unbounded")
                # infinity_choice.extend(optional_unbounded)

                # # bounded elements may show up anywhere between unbounded optional elements.
                # # However we can't use <xs:all> here (XSD 1.0 limitation). Instead use a choice
                # # over sequences of unbounded choices with a permutation of bounded elements
                # # inbetween them.
                # container = ElementTree.Element(_to_qname("xs:choice"))
                # for sequence in permutations(bounded_elements):
                #     seq_container = ElementTree.Element(_to_qname("xs:sequence"))
                #     seq_container.append(infinity_choice)
                #     for pair in zip_longest(sequence, [], fillvalue=infinity_choice):
                #         seq_container.extend(pair)
                
                #     container.append(seq_container)

                # el.append(container)
                # el.extend(attributes)

                # above code appears to work, but the generated
                # XSD is multiple GB in size. Use this for now.
                seq_container = ElementTree.Element(_to_qname("xs:choice"))
                seq_container.set("maxOccurs", "unbounded")
                seq_container.extend(elements)
                el.append(seq_container)
                el.extend(attributes)

        else:
            el.extend(attributes)

        return namespaces, el


def setup_schema(used_ns: list, use_default_ns: bool = True) -> ElementTree.Element:
    xsd_schema = ElementTree.Element(_to_qname("xs:schema"))
    xsd_schema.set("version", "1.1")

    if use_default_ns:
        xsd_schema.set("xmlns", namespaces[source.stem])
        xsd_schema.set("targetNamespace", namespaces[source.stem])

    for name in set(used_ns):
        el = ElementTree.Element(_to_qname("xs:import"))
        el.set("namespace", namespaces[name])

        if name == "types":
            # types is a special class (not generated)
            el.set("schemaLocation", f"./types.xsd")
        else:
            el.set("schemaLocation", f"./{name}Type.xsd")

        xsd_schema.append(el)
        xsd_schema.set(f"xmlns:{name}", namespaces[name])

    return xsd_schema


out_dir = Path(args.target)
if not out_dir.exists():
    out_dir.mkdir(exist_ok=True, parents=True)

# write type file
used_ns, element = ComplexType(root, root.attrib["name"] + "Type").to_subtree()
xsd_schema = setup_schema(used_ns)
xsd_schema.append(element)
with open(out_dir / (source.stem + "Type.xsd"), "wb") as out_file:
    ElementTree.ElementTree(xsd_schema).write(out_file, encoding="UTF-8", xml_declaration=True)

# write element file
file_name = source.stem
tag_name = root.attrib["name"]
used_ns, element = Element(root).to_basic()
element.set("type", f"{file_name}:{tag_name}Type")
used_ns.append(file_name)
xsd_schema = setup_schema(used_ns, use_default_ns=False)
xsd_schema.append(element)
with open(out_dir / (source.stem + ".xsd"), "wb") as out_file:
    ElementTree.ElementTree(xsd_schema).write(out_file, encoding="UTF-8", xml_declaration=True)
