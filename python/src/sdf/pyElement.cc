/*
 * Copyright (C) 2023 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pyElement.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <string>

#include "pyParam.hh"
#include "pybind11_helpers.hh"
#include "sdf/Element.hh"
#include "sdf/Param.hh"
#include "sdf/PrintConfig.hh"
#include "sdf/config.hh"

using namespace pybind11::literals;
namespace py = pybind11;
namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
namespace python
{

/////////////////////////////////////////////////
// The following functions have been excluded from the bindings because they're
// mainly used internally by the parser or there are better alternatives.
// - GetCopyChildren (Used by the parser for handling unknown elements)
// - SetCopyChildren (Used by the parser for handling unknown elements)
// - SetReferenceSDF (Used only by the parser)
// - ReferenceSDF (Used only by the parser)
// - PrintDescription (Use GetDescription() and print)
// - PrintValues (Because ToString is available)
// - PrintDocLeftPane (Helper function for generating documentation)
// - PrintDocRightPane (Helper function for generating documentation)
// - HasUniqueChildNames (Used for error checking by the parser)
// - CountNamedElements (Used for error checking by the parser)
// - GetElement (FindElement and AddElement should be used instead)
// - GetElementImpl (Use FindElement instead)
// - GetElementTypeNames (Used for error checking by the parser)
// - NameUniquenessExceptions (Used for error checking by the parser)
void defineElement(py::object module)
{
  using PyClassElement = py::class_<Element, ElementPtr>;

  auto elemClass = PyClassElement(module, "Element");
  elemClass.def(py::init<>())
      .def("clone", ErrorWrappedCast<>(&Element::Clone, py::const_),
           "Create a copy of this Element.")
      .def("copy", ErrorWrappedCast<ElementPtr>(&Element::Copy),
           "Copy values from an Element.")
      .def("get_parent", &Element::GetParent,
           "Get a pointer to this Element's parent.")
      .def("set_parent", &Element::SetParent, "Set the parent of this Element.")
      .def("set_name", &Element::SetName, "Set the name of the Element")
      .def("get_name", &Element::GetName, "Get the Element's name.")
      .def("set_required", &Element::SetRequired, "Set the requirement type.")
      .def("get_required", &Element::GetRequired, "Get the requirement string.")
      .def("set_explicitly_set_in_file", &Element::SetExplicitlySetInFile,
           "Set if the element and children where set or default in the "
           "original file")
      .def("get_explicitly_set_in_file", &Element::GetExplicitlySetInFile,
           "Return if the element was been explicitly set in the file")
      .def("to_string",
           ErrorWrappedCast<const std::string &, const PrintConfig &>(
               &Element::ToString, py::const_),
           "Convert the element values to a string representation.", "prefix"_a,
           "config"_a = PrintConfig())
      .def(
          "to_string",
          ErrorWrappedCast<const std::string &, bool, bool,
                           const PrintConfig &>(&Element::ToString, py::const_),
          "Convert the element values to a string representation.", "prefix"_a,
          "include_default_elements"_a, "include_default_attribute"_a,
          "config"_a = PrintConfig())
      .def(
          "add_attribute",
          [](Element &_self, const std::string &_key, const std::string &_type,
             const std::string &_defaultvalue, bool _required,
             const std::string &_description = "")
          {
            Errors errors;
            _self.AddAttribute(_key, _type, _defaultvalue, _required, errors,
                               _description);
            ThrowIfErrors(errors);
          },
          "Add an attribute value.", "key"_a, "type"_a, "default_value"_a,
          "required"_a, "description"_a = "")
      .def(
          "add_value",
          [](Element &_self, const std::string &_type,
             const std::string &_defaultValue, bool _required,
             const std::string &_description = "")
          {
            Errors errors;
            _self.AddValue(_type, _defaultValue, _required, errors,
                           _description);
            ThrowIfErrors(errors);
          },
          "Add a value to this Element", "type"_a, "default_value"_a,
          "required"_a, "description"_a = "")
      .def(
          "add_value",
          [](Element &_self, const std::string &_type,
             const std::string &_defaultValue, bool _required,
             const std::string &_minValue, const std::string &_maxValue,
             const std::string &_description = "")
          {
            Errors errors;
            _self.AddValue(_type, _defaultValue, _required, _minValue,
                           _maxValue, errors, _description);
            ThrowIfErrors(errors);
          },
          "Add a value to this Element", "type"_a, "default_value"_a,
          "required"_a, "min_value"_a, "max_value"_a, "description"_a = "")
      .def("get_attribute",
           py::overload_cast<const std::string &>(&Element::GetAttribute,
                                                  py::const_),
           "Get the param of an attribute.")
      .def("get_attribute_count", &Element::GetAttributeCount,
           "Get the number of attributes.")
      .def("get_attributes", &Element::GetAttributes,
           "Get all the attribute params.")
      .def("get_attribute",
           py::overload_cast<unsigned int>(&Element::GetAttribute, py::const_),
           "Get the param of an attribute.")
      .def("get_element_description_count",
           &Element::GetElementDescriptionCount,
           "Get the number of element descriptions.")
      .def("get_element_description",
           py::overload_cast<unsigned int>(&Element::GetElementDescription,
                                           py::const_),
           "Get an element description using an index")
      .def("get_element_description",
           py::overload_cast<const std::string &>(
               &Element::GetElementDescription, py::const_),
           "Get an element description using a key")
      .def("has_element_description", &Element::HasElementDescription,
           "Return true if an element description exists.")
      .def("has_attribute", &Element::HasAttribute,
           "Return true if an attribute exists.")
      .def("get_attribute_set", &Element::GetAttributeSet,
           "Return true if the attribute was set (i.e. not default value)")
      .def("remove_attribute", &Element::RemoveAttribute,
           "Remove an attribute.")
      .def("remove_all_attributes", &Element::RemoveAllAttributes,
           "Removes all attributes.")
      .def("get_value", &Element::GetValue,
           "Get the param of the elements value")
      // get_any is not supported since std::any is not supported in
      // pybind11.
      // get and set are defined below
      .def("has_element", &Element::HasElement,
           "Return true if the named element exists.")
      .def("get_first_element", &Element::GetFirstElement,
           "Get the first child element")
      .def("get_next_element", &Element::GetNextElement,
           "Get the first child Get the next sibling of this element.")
      .def("find_element",
           py::overload_cast<const std::string &>(&Element::FindElement),
           "Return a pointer to the child element with the provided name.")
      .def("add_element",
           ErrorWrappedCast<const std::string &>(&Element::AddElement),
           "Add a value to this Element")
      .def("insert_element",
           py::overload_cast<ElementPtr>(&Element::InsertElement),
           "Add an element object.")
      .def("insert_element",
           py::overload_cast<ElementPtr, bool>(&Element::InsertElement),
           "Add an element object, and optionally set the given element's "
           "parent to this object")
      .def("remove_from_parent", &Element::RemoveFromParent,
           "Remove this element from its parent.")
      .def("remove_child", ErrorWrappedCast<ElementPtr>(&Element::RemoveChild),
           "Remove a child element.")
      .def("clear_elements", &Element::ClearElements,
           "Remove all child elements.")
      .def("clear", &Element::Clear,
           "Remove all child elements and reset file path and original "
           "version.")
      .def("update", ErrorWrappedCast<>(&Element::Update),
           "Call the Update() callback on each element, as well as the "
           "embedded Param.")
      .def("reset", &Element::Reset,
           "Call reset on each element and element description before "
           "deleting all of them.  Also clear out the embedded Param.")
      .def("set_include_element", &Element::SetIncludeElement,
           "Set the `<include>` element that was used to load this element")
      .def("get_include_element", &Element::GetIncludeElement,
           "Get the `<include>` element that was used to load this element.")
      .def("set_file_path", &Element::SetFilePath,
           "Set the path to the SDF document where this element came from.")
      .def("file_path", &Element::FilePath,
           "Get the path to the SDF document where this element came from")
      .def("set_line_number", &Element::SetLineNumber,
           "Set the line number of this element within the SDF document.")
      .def("line_number", &Element::LineNumber,
           "Get the line number of this element within the SDF document.")
      .def("set_xml_path", &Element::SetXmlPath,
           "Set the XML path of this element.")
      .def("xml_path", &Element::XmlPath, "Get the XML path of this element.")
      .def("set_original_version", &Element::SetOriginalVersion,
           "Set the spec version that this was originally parsed from.")
      .def("original_version", &Element::OriginalVersion,
           "Get the spec version that this was originally parsed from.")
      .def("get_description", &Element::GetDescription,
           "Get a text description of the element.")
      .def("set_description", &Element::SetDescription,
           "Set a text description for the element.")
      .def("add_element_description", &Element::AddElementDescription,
           "Add a new element description");

  // Definitions for `Get<T>`, and `Set<T>` which will bind to `get_bool`, 
  // `get_int`, etc.
  forEachParamType(
      [&elemClass](auto &&arg)
      {
        using T = std::decay_t<decltype(arg)>;
        const std::string getFuncName = "get_" + computeSuffix<T>();
        const std::string setFuncName = "set_" + computeSuffix<T>();
        elemClass
            .def(getFuncName.c_str(),
                 ErrorWrappedCast<const std::string &>(&Element::Get<T>,
                                                       py::const_),
                 "Get the value of a key. This function assumes the _key "
                 "exists.",
                 "key"_a = "")
            .def(getFuncName.c_str(),
                 ErrorWrappedCast<const std::string &, const T &>(
                     &Element::Get<T>, py::const_),
                 "Get the value of a key.")
            .def(setFuncName.c_str(),
                 ErrorWrappedCast<const T &>(&Element::Set<T>),
                 "Get the value of a key. This function assumes the _key "
                 "exists.");
      });
}
}  // namespace python
}  // namespace SDF_VERSION_NAMESPACE
}  // namespace sdf
