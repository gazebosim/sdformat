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

root = ElementTree.parse(source).getroot()
declared = dict()
declared["imports"] = set()


def _to_simple_type(in_type:str) -> str:
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
        return known_types[in_type]
    except KeyError:
        raise RuntimeError(f"Unknown simple type: {in_type}") from None

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
class Description:
    element: ElementTree.Element

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

@dataclass
class Element:
    element: ElementTree.Element

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

            other_file = source_dir / (self.element.attrib["ref"]+".sdf")
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
            "0" : ("0", "1"),
            "1" : ("1", "1"),
            "+" : ("1", "unbounded"),
            "*" : ("0", "unbounded"),
            "-1" : ("0", "0")
        }
        min_occurs, max_occurs = required_codes[self.element.attrib["required"]]
        choice = ElementTree.Element(_to_qname("xs:choice"))
        choice.set("minOccurs", min_occurs)
        choice.set("maxOccurs", max_occurs)
        choice.append(el)

        return namespaces, el


@dataclass
class Include:
    element: ElementTree.Element

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

        #TODO: remove this line
        declared.setdefault("imports", set()).add(other_file.stem)


        required_codes = {
            "0" : ("0", "1"),
            "1" : ("1", "1"),
            "+" : ("1", "unbounded"),
            "*" : ("0", "unbounded"),
            "-1" : ("0", "0")
        }
        min_occurs, max_occurs = required_codes[self.element.attrib["required"]]
        choice = ElementTree.Element(_to_qname("xs:choice"))
        choice.set("minOccurs", min_occurs)
        choice.set("maxOccurs", max_occurs)
        choice.append(el)
        
        return namespaces, choice


@dataclass
class Attribute:
    element: ElementTree.Element

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

@dataclass
class ComplexType:
    element: ElementTree.Element
    name: str = None

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
            raise NotImplementedError("Cant handle sub-elements for an element declaring a type.")
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
            choice = ElementTree.Element(_to_qname("xs:choice"))
            choice.set("maxOccurs", "unbounded")
            choice.extend(elements)
            el.append(choice)
            el.extend(attributes)
        else:
            el.extend(attributes)

        return namespaces, el

def setup_schema(used_ns:list, use_default_ns:bool=True) -> ElementTree.Element:
    xsd_schema = ElementTree.Element(_to_qname("xs:schema"))

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
used_ns, element = ComplexType(root, root.attrib["name"]+"Type").to_subtree()
xsd_schema = setup_schema(used_ns)
xsd_schema.append(element)
with open(out_dir / (source.stem + "Type.xsd"), "w") as out_file:
    ElementTree.ElementTree(xsd_schema).write(out_file, encoding="unicode")

# write element file
file_name = source.stem
tag_name = root.attrib["name"]
used_ns, element = Element(root).to_basic()
element.set("type", f"{file_name}:{tag_name}Type")
used_ns.append(file_name)
xsd_schema = setup_schema(used_ns, use_default_ns=False)
xsd_schema.append(element)
with open(out_dir / (source.stem + ".xsd"), "w") as out_file:
    ElementTree.ElementTree(xsd_schema).write(out_file, encoding="unicode")
