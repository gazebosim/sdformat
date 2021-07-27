from xml.etree import ElementTree

from pathlib import Path
import argparse
from dataclasses import dataclass
from typing import List


def _tabulate(input:str, offset:int=0, style:str=" "*4) -> str:
    """Given XML/XSD input, add indentation and return it"""
    formatted = ""
    for line in input.split("\n")[:-1]:
        formatted += style * offset + line + "\n"
    return formatted


cmd_arg_parser = argparse.ArgumentParser(description="Create an XML schema from a SDFormat file.")
cmd_arg_parser.add_argument("-i", "--in", dest="source", required=True, type=str, help="SDF file inside of directory to compile.")
cmd_arg_parser.add_argument("-s", "--sdf", dest="directory", required=True, type=str, help="Directory containing all the SDF files.")
cmd_arg_parser.add_argument("-o", "--out", dest="target", default=".", type=str, help="Output directory for xsd file. Will be created if it doesn't exit.")
cmd_arg_parser.add_argument("--ns-prefix", dest="ns_prefix", default="sdformat", type=str, help="Prefix for generated xsd namespaces.")

args = cmd_arg_parser.parse_args()
source_dir = Path(args.directory)
source:Path = source_dir / args.source
template_dir = Path(__file__).parent / "xsd_templates"

# collect existing namespaces
namespaces = {
    "types": "http://sdformat.org/schemas/types.xsd",
    "xs": "http://www.w3.org/2001/XMLSchema"
}
for path in source_dir.iterdir():
    if not path.is_file():
        continue

    if not path.suffix == ".sdf":
        continue

    namespaces[path.stem] = f"{args.ns_prefix}/{path.stem}"
    ElementTree.register_namespace(path.stem, f"{args.ns_prefix}/{path.stem}")


def _to_qname(name:str) -> str:
    try:
        prefix, name = name.split(":")
    except ValueError:
        # no prefix
        return name
    else:
        return f"{{{namespaces[prefix]}}}{name}"

@dataclass
class Element:
    name: str
    type: str
    required:str
    default:str=None
    description:str=None

    def to_basic(self):
        el = ElementTree.Element(_to_qname("xs:element"))
        el.set("name", self.name)
        el.set("type", self.type)
        if self.default:
            el.set("default", self.default)
        return el

    def to_etree_element(self):
        el = ElementTree.Element(_to_qname("xs:element"))
        el.set("name", self.name)
        el.set("type", self.type)
        if self.default:
            el.set("default", self.default)

        required_codes = {
            "0" : ("0", "1"),
            "1" : ("1", "1"),
            "+" : ("1", "unbounded"),
            "*" : ("0", "unbounded"),
            "-1" : ("0", "0")
        }
        min_occurs, max_occurs = required_codes[self.required]
        choice = ElementTree.Element(_to_qname("xs:choice"))
        choice.tail = "\n"
        choice.set("minOccurs", min_occurs)
        choice.set("maxOccurs", max_occurs)
        choice.append(el)

        return choice


@dataclass
class Attribute:
    name: str
    type: str
    required: str
    default: int
    description:str=None

    def to_etree_element(self):
        el = ElementTree.Element(_to_qname("xs:attribute"))
        el.tail = "\n"

        el.set("name", self.name)
        el.set("type", self.type)
        el.set("use", "required" if self.required == "1" else "optional")
        
        if self.default is not None:
            el.set("default", self.default)

        return el

@dataclass
class ComplexType:
    name: str
    element: ElementTree.Element

    def to_etree_element(self, declared):
        elements = list()
        attributes = list()

        for attribute in self.element.findall("attribute"):
            if "default" in attribute.attrib.keys():
                    default = attribute.attrib["default"]

            attributes.append(Attribute(
                name = attribute.attrib["name"],
                type = attribute.attrib["type"],
                required= attribute.attrib["required"],
                default=default
            ).to_etree_element())

        for child in self.element.findall("element"):
            if "copy_data" in child.attrib:
                # special element for plugin.sdf to allow
                # declare that any children are allowed
                anyEl = ElementTree.Element(_to_qname("xs:any"))
                anyEl.set("minOccurs", "0")
                anyEl.set("maxOccurs", "unbounded")
                anyEl.set("processContents", "skip")
                elements.append(anyEl)
                continue

            name = child.attrib["name"]
            elements.append(declared["elements"][name].to_etree_element())

        for child in self.element.findall("include"):
            child:ElementTree.Element
            other_file = source_dir / child.attrib["filename"]
            other_root = ElementTree.parse(other_file).getroot()
            name = other_root.attrib["name"]
            elements.append(declared["elements"][name].to_etree_element())

        el = ElementTree.Element(_to_qname("xs:complexType"))
        el.set("name", self.name)
        el.tail = "\n"

        if elements and "type" in self.element.attrib:
            pass
        elif "type" in self.element.attrib:
            extension = ElementTree.Element(_to_qname("xs:extension"))
            extension.set("base", self.element.attrib["type"])
            simple_content = ElementTree.Element(_to_qname("xs:simpleContent"))
            simple_content.append(extension)
            el.append(simple_content)
        elif elements:
            choice = ElementTree.Element(_to_qname("xs:choice"))
            choice.set("maxOccurs", "unbounded")
            choice.tail = "\n"
            choice.extend(elements)
            el.append(choice)

        el.extend(attributes)

        return el


@dataclass
class SimpleType:
    name: str
    namespace:str=None

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)

    def to_etree_element(self):
        name = self.name
        known_types = {
            "unsigned int": "xs:unsignedInt",
            "unsigned long": "xs:unsignedLong",
            "bool": "xs:boolean",
            "string":"xs:string",
            "double":"xs:double",
            "int":"xs:int",
            "float":"xs:float",
            "char":"xs:char",
            "vector3": "types:vector3",
            "vector2d": "types:vector2d",
            "vector2i": "types:vector2i",
            "pose": "types:pose",
            "time": "types:time",
            "color": "types:color",
        }

        try:
            referred = known_types[name]
        except KeyError:
            raise RuntimeError(f"Unknown primitive type: {name}") from None

        restriction = ElementTree.Element(_to_qname("xs:restriction"))
        restriction.set("base", referred)

        el = ElementTree.Element(_to_qname("xs:simpleType"))
        el.set("name", name)
        el.append(restriction)
        el.tail = "\n"

        return el

# parse file
# recurse elements of the file and track which xsd elements need to be generated
def expand(element:ElementTree.Element, declared:dict):
    if element.tag == "description":
        # not handled at this stage and may
        # contain tags (see particle_emitter)
        return

    for child in element:
        expand(child, declared)

    desc_element = element.find("description")
    description = desc_element.text if desc_element else None

    if element.tag == "element":
        if "copy_data" in element.attrib:
            # special element for plugin.sdf
            # essentially allows any element to occur here
            # we ignore it here, but insert <xs:any> while creating
            # the pluginType
            return

        name = element.attrib["name"]
        element_type = name + "Type"
        required = element.attrib["required"]
        
        num_children = len(element)
        if element.find("description") is not None:
            num_children = len(element) - 1

        has_children = num_children > 0
        has_type = "type" in element.attrib

        if has_children and has_type:
            # extens existing simple type
            # Example: pose in pose.sdf
            element_type = name + "Type"
            declared.setdefault("simple_types", set()).add(SimpleType(element.attrib["type"]))
            declared.setdefault("complex_types", dict())[element_type] = ComplexType(element_type, element)
        elif has_type:
            # redefinition of simple type
            # Example: (some) children of link.sdf
            element_type = element.attrib["type"]
            declared.setdefault("simple_types", set()).add(SimpleType(element_type))
        elif has_children:
            # new complex type
            # Example: world in world.sdf
            element_type = name + "Type"
            declared.setdefault("complex_types", dict())[element_type] = ComplexType(element_type, element)
        else:
            # element that wraps a string
            # Example: audio_sink in audio_sink.sdf
            element_type = name + "Type"
            # declared.setdefault("simple_types", set()).add(SimpleType("string"))
            declared.setdefault("complex_types", dict())[element_type] = ComplexType(name + "Type", element)
        
        
        default = element.attrib["default"] if "default" in element.attrib else None
        elements:dict = declared.setdefault("elements", dict())
        elements[name] = Element(
            name,
            type=element_type,
            required=required,
            default=default,
            description=description
        )
    elif element.tag == "include":
        other_file = source_dir / element.attrib["filename"]
        other_root = ElementTree.parse(other_file).getroot()
        name = other_root.attrib["name"]
        element_type = f"{other_file.stem}:{name}Type"
        required = element.attrib["required"]
        elements = declared.setdefault("elements", dict())
        elements[name] = Element(name, element_type, required)
        declared.setdefault("imports", set()).add(other_file.stem)
    elif element.tag == "attribute":
        element_type = element.attrib["type"]
        declared.setdefault("simple_types", set()).add(SimpleType(element_type))
    else:
        raise RuntimeError(f"Unknown SDF element encountered: {element.tag}")

root = ElementTree.parse(source).getroot()
declared = dict()
declared.setdefault("imports", set()).add(source.stem)
expand(root, declared)


xsd_root = ElementTree.Element(_to_qname("xs:schema"))
xsd_root.set("xmlns", namespaces[source.stem])
xsd_root.set("targetNamespace", namespaces[source.stem])
strings = dict()

for name in declared["imports"]:
    el = ElementTree.Element(_to_qname("xs:import"))
    el.set("namespace", namespaces[name])
    el.set("schemaLocation", f"./{name}Type.xsd")
    el.tail = "\n"
    
    xsd_root.append(el)
    xsd_root.set(f"xmlns:{name}", namespaces[name])

if "simple_types" in declared:
    el = ElementTree.Element(_to_qname("xs:import"))
    el.set("namespace", namespaces["types"])
    el.set("schemaLocation", f"./types.xsd")
    el.tail = "\n"
    
    xsd_root.append(el)
    xsd_root.set(f"xmlns:types", namespaces["types"])

    for simple_type in declared["simple_types"]:
        xsd_root.append(simple_type.to_etree_element())
else:
    strings["simple_types"] = list()

if "complex_types" in declared:
    for complex_type in declared["complex_types"].values():
        xsd_root.append(complex_type.to_etree_element(declared))
else:
    strings["complex_types"] = list()

out_dir = Path(args.target)

if not out_dir.exists():
    out_dir.mkdir(exist_ok=True, parents=True)
# write type file
with open(out_dir / (source.stem + "Type.xsd"), "w") as out_file:
    ElementTree.ElementTree(xsd_root).write(out_file, encoding="unicode")

# write element file
xsd_root = ElementTree.Element(_to_qname("xs:schema"))
# xsd_root.set("targetNamespace", namespaces[source.stem])
xsd_root.set("xmlns", namespaces[source.stem])
name = source.stem
el = ElementTree.Element(_to_qname("xs:import"))
el.set("namespace", namespaces[name])
el.set("schemaLocation", f"./{name}.xsd")
el.tail = "\n"
xsd_root.append(el)
# xsd_root.set(f"xmlns:{name}", namespaces[name])
xsd_root.append(declared["elements"][root.attrib["name"]].to_basic())
with open(out_dir / (source.stem + ".xsd"), "w") as out_file:
    ElementTree.ElementTree(xsd_root).write(out_file, encoding="unicode")
