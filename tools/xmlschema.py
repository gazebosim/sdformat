from xml.etree import ElementTree
from pathlib import Path
import argparse
from dataclasses import dataclass
from typing import List

cmd_arg_parser = argparse.ArgumentParser(description="Create an XML schema from a SDFormat file.")
cmd_arg_parser.add_argument("-i", "--in", dest="source", required=True, type=str, help="SDF file inside of directory to compile.")
cmd_arg_parser.add_argument("-s", "--sdf", dest="directory", required=True, type=str, help="Directory containing all the SDF files.")
cmd_arg_parser.add_argument("-o", "--out", dest="target", default=".", type=str, help="Output directory for xsd file. Will be created if it doesn't exit.")
cmd_arg_parser.add_argument("--ns-prefix", dest="ns_prefix", default="sdformat", type=str, help="Prefix for generated xsd namespaces.")

args = cmd_arg_parser.parse_args()
source_dir = Path(args.directory)
source:Path = source_dir / args.source
template_dir = Path(__file__).parent / "xsd_templates"
xsd_file_template:str = (template_dir / "file.xsd").read_text()


def _tabulate(input:str, offset:int=0) -> str:
    formatted = ""
    for line in input.split("\n")[:-1]:
        formatted += (" " * 4) * offset + line + "\n"
    return formatted

# collect existing namespaces
namespaces = {"types": f"{args.ns_prefix}/types"}
for path in source_dir.iterdir():
    if not path.is_file():
        continue

    if not path.suffix == ".sdf":
        continue

    namespaces[path.stem] = f"{args.ns_prefix}/{path.stem}"

@dataclass
class Element:
    name: str
    type: str
    required:str
    default:str=None
    description:str=None

    def to_xsd(self):
        return f"<xs:element name='{self.name}' type='{self.type}' />"

    def to_typedef(self):
        """The string used inside a ComplexType to refer to this element"""

        required_codes = {
            "0" : ("0", "1"),
            "1" : ("1", "1"),
            "+" : ("1", "unbounded"),
            "*" : ("0", "unbounded"),
            "-1" : ("0", "0")
        }
        min_occurs, max_occurs = required_codes[self.required]

        if self.description:
            template = (template_dir / "element_with_comment.xsd").read_text()
        else:
            template = (template_dir / "element.xsd").read_text()
        template = template.format(
            min_occurs=min_occurs,
            max_occurs=max_occurs,
            name = self.name,
            type = self.type,
            default = f"default='{self.default}'" if self.default is not None else "",
            description = self.description,
        )

        return _tabulate(template, 2)

@dataclass
class Attribute:
    name: str
    type: str
    required: str
    default: int
    description:str=None

    def to_typedef(self):
        template = (template_dir / "attribute.xsd").read_text()
        template = template.format(
            name = self.name,
            type = self.type,
            required = "required" if self.required == "1" else "optional",
            default = self.default
        )

        return _tabulate(template, 1)

@dataclass
class ComplexType:
    name: str
    element: ElementTree.Element

    def to_xsd(self, declared):
        attributes = list()
        elements = list()

        for attribute in self.element.findall("attribute"):
            if "default" in attribute.attrib.keys():
                    default = attribute.attrib["default"]

            attributes.append(Attribute(
                name = attribute.attrib["name"],
                type = attribute.attrib["type"],
                required= attribute.attrib["required"],
                default=default
            ).to_typedef())

        for child in self.element.findall("element"):
            if "copy_data" in child.attrib:
                # special element for plugin.sdf to allow
                # declare that any children are allowed
                any_xsd = "<xs:any minOccurs='0' maxOccurs='unbounded' processContents='skip'/>\n"
                elements.append(_tabulate(any_xsd, 2))
                continue

            name = child.attrib["name"]
            elements.append(declared["elements"][name].to_typedef())

        for child in self.element.findall("include"):
            child:ElementTree.Element
            other_file = source_dir / child.attrib["filename"]
            other_root = ElementTree.parse(other_file).getroot()
            name = other_root.attrib["name"]
            elements.append(declared["elements"][name].to_typedef())

        if "type" in self.element.attrib:
            if len(elements) > 0:
                raise RuntimeError("The compiler cant generate this type.")
            
            template = (template_dir / "expansion_type.xsd").read_text()
            template = template.format(
                name = self.name,
                type = self.element.attrib["type"],
                attributes = "\n".join(attributes)
            )

        else:
            elements = "\n".join(elements)
            attributes = "\n".join(attributes)

            template = (template_dir / "type.xsd").read_text()
            template = template.format(
                name = self.name,
                elements = elements,
                attributes = attributes
            )

        return template

@dataclass
class SimpleType:
    name: str
    namespace:str=None

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)

    def to_xsd(self):
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

        return f"<xs:simpleType name='{name}'><xs:restriction base='{referred}' /></xs:simpleType>"

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


strings = dict()

for name in declared["imports"]:
    ns_elements = strings.setdefault("namespaces", [])
    ns_elements.append(f"    xmlns:{name}='{namespaces[name]}'")
    imp_elements = strings.setdefault("imports", [])
    imp_elements.append(f"    <xs:import namespace='{namespaces[name]}' schemaLocation='./{name}.xsd'/>")

if "simple_types" in declared:
    for simple_type in declared["simple_types"]:
        elements = strings.setdefault("simple_types", [])
        elements.append(simple_type.to_xsd())
else:
    strings["simple_types"] = list()

if "complex_types" in declared:
    for complex_type in declared["complex_types"].values():
        elements = strings.setdefault("complex_types", [])
        elements.append(complex_type.to_xsd(declared))
else:
    strings["complex_types"] = list()



# write the file to disk
substitutions = {
    "file_namespace": namespaces[source.stem],
    "namespaces": "\n".join(strings["namespaces"]),
    "imports":"\n".join(strings["imports"]),
    "element":declared["elements"][root.attrib["name"]].to_xsd(),
    "simple_types":"\n".join(strings["simple_types"]),
    "complex_types":"\n".join(strings["complex_types"])
}

out_dir = Path(args.target)
if not out_dir.exists():
    out_dir.mkdir(exist_ok=True, parents=True)

with open(out_dir / (source.stem + ".xsd"), "w") as out_file:
    print(xsd_file_template.format(**substitutions), file=out_file)
