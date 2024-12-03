/*
 * Copyright 2015 Open Source Robotics Foundation
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
 *
 */

#include <algorithm>
#include <sstream>
#include <string>

#include "sdf/Assert.hh"
#include "sdf/Element.hh"
#include "sdf/Filesystem.hh"
#include "Utils.hh"

using namespace sdf;

/////////////////////////////////////////////////
Element::Element()
  : dataPtr(new ElementPrivate)
{
  this->dataPtr->copyChildren = false;
  this->dataPtr->referenceSDF = "";
  this->dataPtr->explicitlySetInFile = true;
}

/////////////////////////////////////////////////
Element::~Element()
{
}

/////////////////////////////////////////////////
ElementPtr Element::GetParent() const
{
  return this->dataPtr->parent.lock();
}

/////////////////////////////////////////////////
void Element::SetParent(const ElementPtr _parent)
{
  this->dataPtr->parent = _parent;

  // If this element doesn't have a path, get it from the parent
  if (nullptr != _parent && (this->FilePath().empty() ||
      this->FilePath() == std::string(kSdfStringSource)))
  {
    this->SetFilePath(_parent->FilePath());
  }

  // If this element doesn't have an original version, get it from the parent
  if (nullptr != _parent && this->OriginalVersion().empty())
  {
    this->SetOriginalVersion(_parent->OriginalVersion());
  }
}

/////////////////////////////////////////////////
void Element::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
const std::string &Element::GetName() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Element::SetRequired(const std::string &_req)
{
  this->dataPtr->required = _req;
}

/////////////////////////////////////////////////
const std::string &Element::GetRequired() const
{
  return this->dataPtr->required;
}

/////////////////////////////////////////////////
void Element::SetCopyChildren(bool _value)
{
  this->dataPtr->copyChildren = _value;
}

/////////////////////////////////////////////////
void Element::SetExplicitlySetInFile(const bool _value)
{
  this->dataPtr->explicitlySetInFile = _value;

  ElementPtr_V::const_iterator eiter;
  for (eiter = this->dataPtr->elements.begin();
       eiter != this->dataPtr->elements.end(); ++eiter)
  {
    (*eiter)->SetExplicitlySetInFile(_value);
  }
}

/////////////////////////////////////////////////
bool Element::GetExplicitlySetInFile() const
{
  return this->dataPtr->explicitlySetInFile;
}

/////////////////////////////////////////////////
bool Element::GetCopyChildren() const
{
  return this->dataPtr->copyChildren;
}

/////////////////////////////////////////////////
void Element::SetReferenceSDF(const std::string &_value)
{
  this->dataPtr->referenceSDF = _value;
}

/////////////////////////////////////////////////
std::string Element::ReferenceSDF() const
{
  return this->dataPtr->referenceSDF;
}

/////////////////////////////////////////////////
void Element::AddValue(const std::string &_type,
                       const std::string &_defaultValue,
                       bool _required,
                       const std::string &_description)
{
  sdf::Errors errors;
  this->dataPtr->value = this->CreateParam(this->dataPtr->name,
      _type, _defaultValue, _required, errors, _description);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void Element::AddValue(const std::string &_type,
                       const std::string &_defaultValue,
                       bool _required,
                       sdf::Errors &_errors,
                       const std::string &_description)
{
  this->dataPtr->value = this->CreateParam(this->dataPtr->name,
      _type, _defaultValue, _required, _errors, _description);
}

/////////////////////////////////////////////////
void Element::AddValue(const std::string &_type,
                       const std::string &_defaultValue,
                       bool _required,
                       const std::string &_minValue,
                       const std::string &_maxValue,
                       const std::string &_description)
{
  sdf::Errors errors;
  this->AddValue(_type, _defaultValue, _required, _minValue, _maxValue,
                 errors, _description);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void Element::AddValue(const std::string &_type,
                       const std::string &_defaultValue,
                       bool _required,
                       const std::string &_minValue,
                       const std::string &_maxValue,
                       sdf::Errors &_errors,
                       const std::string &_description)
{
  this->dataPtr->value =
      std::make_shared<Param>(this->dataPtr->name, _type, _defaultValue,
                              _required, _minValue, _maxValue, _errors,
                              _description);
  SDF_ASSERT(this->dataPtr->value->SetParentElement(shared_from_this()),
      "Cannot set parent Element of value to itself.");
}

/////////////////////////////////////////////////
ParamPtr Element::CreateParam(const std::string &_key,
                              const std::string &_type,
                              const std::string &_defaultValue,
                              bool _required,
                              sdf::Errors &_errors,
                              const std::string &_description)
{
  ParamPtr param = std::make_shared<Param>(
      _key, _type, _defaultValue, _required, _errors, _description);
  SDF_ASSERT(param->SetParentElement(shared_from_this()),
      "Cannot set parent Element of created Param to itself.");
  return param;
}

/////////////////////////////////////////////////
void Element::AddAttribute(const std::string &_key,
                           const std::string &_type,
                           const std::string &_defaultValue,
                           bool _required,
                           const std::string &_description)
{
  sdf::Errors errors;
  this->AddAttribute(_key, _type, _defaultValue, _required,
                     errors, _description);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void Element::AddAttribute(const std::string &_key,
                           const std::string &_type,
                           const std::string &_defaultValue,
                           bool _required,
                           sdf::Errors &_errors,
                           const std::string &_description)
{
  this->dataPtr->attributes.push_back(
      this->CreateParam(_key, _type, _defaultValue,
                        _required, _errors, _description));
}

/////////////////////////////////////////////////
ElementPtr Element::Clone() const
{
  sdf::Errors errors;
  ElementPtr elem = this->Clone(errors);
  sdf::throwOrPrintErrors(errors);
  return elem;
}

/////////////////////////////////////////////////
ElementPtr Element::Clone(sdf::Errors &_errors) const
{
  ElementPtr clone(new Element);
  clone->dataPtr->description = this->dataPtr->description;
  clone->dataPtr->name = this->dataPtr->name;
  clone->dataPtr->required = this->dataPtr->required;
  clone->dataPtr->copyChildren = this->dataPtr->copyChildren;
  clone->dataPtr->referenceSDF = this->dataPtr->referenceSDF;
  clone->dataPtr->path = this->dataPtr->path;
  clone->dataPtr->lineNumber = this->dataPtr->lineNumber;
  clone->dataPtr->xmlPath = this->dataPtr->xmlPath;
  clone->dataPtr->originalVersion = this->dataPtr->originalVersion;
  clone->dataPtr->explicitlySetInFile = this->dataPtr->explicitlySetInFile;

  Param_V::const_iterator aiter;
  for (aiter = this->dataPtr->attributes.begin();
       aiter != this->dataPtr->attributes.end(); ++aiter)
  {
    auto clonedAttribute = (*aiter)->Clone();
    SDF_ASSERT(clonedAttribute->SetParentElementNoReparse(clone),
        "Cannot set parent Element of cloned attribute Param to cloned "
        "Element.");
    clone->dataPtr->attributes.push_back(clonedAttribute);
  }

  ElementPtr_V::const_iterator eiter;
  for (eiter = this->dataPtr->elementDescriptions.begin();
      eiter != this->dataPtr->elementDescriptions.end(); ++eiter)
  {
    clone->dataPtr->elementDescriptions.push_back((*eiter)->Clone(_errors));
  }

  for (eiter = this->dataPtr->elements.begin();
       eiter != this->dataPtr->elements.end(); ++eiter)
  {
    clone->dataPtr->elements.push_back((*eiter)->Clone(_errors));
    clone->dataPtr->elements.back()->SetParent(clone);
  }

  if (this->dataPtr->value)
  {
    clone->dataPtr->value = this->dataPtr->value->Clone();
    SDF_ASSERT(clone->dataPtr->value->SetParentElementNoReparse(clone),
        "Cannot set parent Element of cloned value Param to cloned Element.");
  }

  if (this->dataPtr->includeElement)
  {
    clone->dataPtr->includeElement =
        this->dataPtr->includeElement->Clone(_errors);
  }

  return clone;
}

/////////////////////////////////////////////////
void Element::Copy(const ElementPtr _elem)
{
  sdf::Errors errors;
  this->Copy(_elem, errors);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void Element::Copy(const ElementPtr _elem, sdf::Errors &_errors)
{

  this->dataPtr->name = _elem->GetName();
  this->dataPtr->description = _elem->GetDescription();
  this->dataPtr->required = _elem->GetRequired();
  this->dataPtr->copyChildren = _elem->GetCopyChildren();
  this->dataPtr->referenceSDF = _elem->ReferenceSDF();
  this->dataPtr->originalVersion = _elem->OriginalVersion();
  this->dataPtr->path = _elem->FilePath();
  this->dataPtr->lineNumber = _elem->LineNumber();
  this->dataPtr->xmlPath = _elem->XmlPath();
  this->dataPtr->explicitlySetInFile = _elem->GetExplicitlySetInFile();

  for (Param_V::iterator iter = _elem->dataPtr->attributes.begin();
       iter != _elem->dataPtr->attributes.end(); ++iter)
  {
    if (!this->HasAttribute((*iter)->GetKey()))
    {
      this->dataPtr->attributes.push_back((*iter)->Clone());
    }
    ParamPtr param = this->GetAttribute((*iter)->GetKey());
    (*param) = (**iter);
    SDF_ASSERT(param->SetParentElement(shared_from_this()),
        "Cannot set parent Element of copied attribute Param to itself.");
  }

  if (_elem->GetValue())
  {
    if (!this->dataPtr->value)
    {
      this->dataPtr->value = _elem->GetValue()->Clone();
    }
    else
    {
      *(this->dataPtr->value) = *(_elem->GetValue());
    }
    SDF_ASSERT(this->dataPtr->value->SetParentElement(shared_from_this()),
        "Cannot set parent Element of copied value Param to itself.");
  }

  this->dataPtr->elementDescriptions.clear();
  for (ElementPtr_V::const_iterator iter =
       _elem->dataPtr->elementDescriptions.begin();
       iter != _elem->dataPtr->elementDescriptions.end(); ++iter)
  {
    this->dataPtr->elementDescriptions.push_back((*iter)->Clone(_errors));
  }

  this->dataPtr->elements.clear();
  for (ElementPtr_V::iterator iter = _elem->dataPtr->elements.begin();
       iter != _elem->dataPtr->elements.end(); ++iter)
  {
    ElementPtr elem;
    elem = (*iter)->Clone(_errors);
    elem->Copy(*iter, _errors);
    elem->SetParent(shared_from_this());
    this->dataPtr->elements.push_back(elem);
  }

  if (_elem->dataPtr->includeElement)
  {
    if (!this->dataPtr->includeElement)
    {
      this->dataPtr->includeElement =
          _elem->dataPtr->includeElement->Clone(_errors);
    }
    else
    {
      this->dataPtr->includeElement->Copy(_elem->dataPtr->includeElement,
                                          _errors);
    }
  }
}

/////////////////////////////////////////////////
void Element::PrintDescription(const std::string &_prefix) const
{
  sdf::Errors errors;
  this->PrintDescription(errors, _prefix);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void Element::PrintDescription(sdf::Errors &_errors,
                               const std::string &_prefix) const
{
  std::cout << _prefix << "<element name ='" << this->dataPtr->name
            << "' required ='" << this->dataPtr->required << "'";

  if (this->dataPtr->value)
  {
    std::cout << " type ='" << this->dataPtr->value->GetTypeName()
              << "'"
              << " default ='"
              << this->dataPtr->value->GetDefaultAsString(_errors)
              << "'";
    auto minValue = this->dataPtr->value->GetMinValueAsString(_errors);
    if (minValue.has_value())
    {
      std::cout << " min ='" << *minValue << "'";
    }

    auto maxValue = this->dataPtr->value->GetMaxValueAsString(_errors);
    if (maxValue.has_value())
    {
      std::cout << " max ='" << *maxValue << "'";
    }
  }

  std::cout << ">\n";

  std::cout << _prefix << "  <description><![CDATA["
            << this->dataPtr->description
            << "]]></description>\n";

  Param_V::iterator aiter;
  for (aiter = this->dataPtr->attributes.begin();
      aiter != this->dataPtr->attributes.end(); ++aiter)
  {
    std::cout << _prefix << "  <attribute name ='"
              << (*aiter)->GetKey() << "' type ='" << (*aiter)->GetTypeName()
              << "' default ='" << (*aiter)->GetDefaultAsString(_errors)
              << "' required ='" << (*aiter)->GetRequired() << "'>\n";
    std::cout << _prefix << "    <description><![CDATA["
              << (*aiter)->GetDescription()
              << "]]></description>\n";
    std::cout << _prefix << "  </attribute>\n";
  }

  if (this->GetCopyChildren())
  {
    std::cout << _prefix << "  <element copy_data ='true' required ='*'/>\n";
  }


  std::string refSDF = this->ReferenceSDF();
  if (!refSDF.empty())
  {
    std::cout << _prefix << "  <element ref ='" << refSDF
              << "' required ='*'/>\n";
  }

  ElementPtr_V::iterator eiter;
  for (eiter = this->dataPtr->elementDescriptions.begin();
      eiter != this->dataPtr->elementDescriptions.end(); ++eiter)
  {
    (*eiter)->PrintDescription(_errors, _prefix + "  ");
  }

  std::cout << _prefix << "</element>\n";
}

/////////////////////////////////////////////////
void Element::PrintDocRightPane(std::string &_html, int _spacing,
                                int &_index) const
{
  std::ostringstream stream;
  ElementPtr_V::iterator eiter;

  int start = _index++;

  std::string childHTML;
  for (eiter = this->dataPtr->elementDescriptions.begin();
      eiter != this->dataPtr->elementDescriptions.end(); ++eiter)
  {
    (*eiter)->PrintDocRightPane(childHTML, _spacing + 4, _index);
  }

  stream << "<a name=\"" << this->dataPtr->name << start
         << "\">&lt" << this->dataPtr->name << "&gt</a>";

  stream << "<div style='padding-left:" << _spacing << "px;'>\n";

  stream << "<div style='background-color: #ffffff'>\n";

  stream << "<font style='font-weight:bold'>Description: </font>";
  if (!this->dataPtr->description.empty())
  {
    stream << this->dataPtr->description << "<br>\n";
  }
  else
  {
    stream << "none<br>\n";
  }

  stream << "<font style='font-weight:bold'>Required: </font>"
         << this->dataPtr->required << "&nbsp;&nbsp;&nbsp;\n";

  stream << "<font style='font-weight:bold'>Type: </font>";
  if (this->dataPtr->value)
  {
    stream << this->dataPtr->value->GetTypeName()
           << "&nbsp;&nbsp;&nbsp;\n"
           << "<font style='font-weight:bold'>Default: </font>"
           << this->dataPtr->value->GetDefaultAsString() << '\n';
  }
  else
  {
    stream << "n/a\n";
  }

  stream << "</div>";

  if (this->dataPtr->attributes.size() > 0)
  {
    stream << "<div style='background-color: #dedede; padding-left:10px; "
           << "display:inline-block;'>\n";
    stream << "<font style='font-weight:bold'>Attributes</font><br>";

    Param_V::iterator aiter;
    for (aiter = this->dataPtr->attributes.begin();
        aiter != this->dataPtr->attributes.end(); ++aiter)
    {
      stream << "<div style='display: inline-block;padding-bottom: 4px;'>\n";

      stream << "<div style='float:left; width: 80px;'>\n";
      stream << "<font style='font-style: italic;'>" << (*aiter)->GetKey()
        << "</font>: ";
      stream << "</div>\n";

      stream << "<div style='float:left; padding-left: 4px; width: 300px;'>\n";

      if (!(*aiter)->GetDescription().empty())
      {
          stream << (*aiter)->GetDescription() << "<br>\n";
      }
      else
      {
          stream << "no description<br>\n";
      }

      stream << "<font style='font-weight:bold'>Type: </font>"
             << (*aiter)->GetTypeName() << "&nbsp;&nbsp;&nbsp;"
             << "<font style='font-weight:bold'>Default: </font>"
             << (*aiter)->GetDefaultAsString() << "<br>";
      stream << "</div>\n";

      stream << "</div>\n";
    }
    stream << "</div>\n";
    stream << "<br>\n";
  }

  _html += stream.str();
  _html += childHTML;
  _html += "</div>\n";
}

/////////////////////////////////////////////////
void Element::PrintDocLeftPane(std::string &_html, int _spacing,
                               int &_index) const
{
  std::ostringstream stream;
  ElementPtr_V::iterator eiter;

  int start = _index++;

  std::string childHTML;
  for (eiter = this->dataPtr->elementDescriptions.begin();
      eiter != this->dataPtr->elementDescriptions.end(); ++eiter)
  {
    (*eiter)->PrintDocLeftPane(childHTML, _spacing + 4, _index);
  }

  stream << "<a id='" << start << "' onclick='highlight(" << start
         << ");' href=\"#" << this->dataPtr->name << start
         << "\">&lt" << this->dataPtr->name << "&gt</a>";

  stream << "<div style='padding-left:" << _spacing << "px;'>\n";

  _html += stream.str();
  _html += childHTML;
  _html += "</div>\n";
}

/////////////////////////////////////////////////
void Element::PrintValuesImpl(sdf::Errors &_errors,
                              const std::string &_prefix,
                              bool _includeDefaultElements,
                              bool _includeDefaultAttributes,
                              const PrintConfig &_config,
                              std::ostringstream &_out) const
{
  if (_config.PreserveIncludes() && this->GetIncludeElement() != nullptr)
  {
    _out << this->GetIncludeElement()->ToString(_errors, _prefix, _config);
  }
  else if (this->GetExplicitlySetInFile() || _includeDefaultElements)
  {
    _out << _prefix << "<" << this->dataPtr->name;

    this->dataPtr->PrintAttributes(
        _errors, _includeDefaultAttributes, _config, _out);

    if (this->dataPtr->elements.size() > 0)
    {
      _out << ">\n";
      ElementPtr_V::const_iterator eiter;
      for (eiter = this->dataPtr->elements.begin();
           eiter != this->dataPtr->elements.end(); ++eiter)
      {
        (*eiter)->ToString(_errors,
                           _out,
                           _prefix + "  ",
                           _includeDefaultElements,
                           _includeDefaultAttributes,
                           _config);
      }
      _out << _prefix << "</" << this->dataPtr->name << ">\n";
    }
    else
    {
      if (this->dataPtr->value)
      {
        _out << ">" << this->dataPtr->value->GetAsString(_errors, _config)
             << "</" << this->dataPtr->name << ">\n";
      }
      else
      {
        _out << "/>\n";
      }
    }
  }
}

/////////////////////////////////////////////////
void ElementPrivate::PrintAttributes(bool _includeDefaultAttributes,
                                     const PrintConfig &_config,
                                     std::ostringstream &_out) const
{
  sdf::Errors errors;
  this->PrintAttributes(errors, _includeDefaultAttributes, _config,
                        _out);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void ElementPrivate::PrintAttributes(sdf::Errors &_errors,
                                     bool _includeDefaultAttributes,
                                     const PrintConfig &_config,
                                     std::ostringstream &_out) const
{
  // Attribute exceptions are used in the event of a non-default PrintConfig
  // which modifies the Attributes of this Element that are printed out. The
  // modifications to an Attribute by a PrintConfig will overwrite the original
  // existing Attribute when this Element is printed.
  std::set<std::string> attributeExceptions;
  if (this->name == "pose")
  {
    if (_config.RotationInDegrees() || _config.RotationSnapToDegrees())
    {
      attributeExceptions.insert("degrees");
      _out << " " << "degrees='true'";

      attributeExceptions.insert("rotation_format");
      _out << " " << "rotation_format='euler_rpy'";
    }
  }

  Param_V::const_iterator aiter;
  for (aiter = this->attributes.begin();
       aiter != this->attributes.end(); ++aiter)
  {
    // Only print attribute values if they were set
    // TODO(anyone): GetRequired is added here to support up-conversions where
    // a new required attribute with a default value is added. We would have
    // better separation of concerns if the conversion process set the
    // required attributes with their default values.
    if ((*aiter)->GetSet() || (*aiter)->GetRequired() ||
        _includeDefaultAttributes ||
        ((*aiter)->GetKey().find(':') != std::string::npos))
    {
      const std::string key = (*aiter)->GetKey();
      const auto it = attributeExceptions.find(key);
      if (it == attributeExceptions.end())
      {
        _out << " " << key << "='"
             << (*aiter)->GetAsString(_errors, _config) << "'";
      }
    }
  }
}

/////////////////////////////////////////////////
void Element::PrintValues(std::string _prefix,
                          const PrintConfig &_config) const
{
  sdf::Errors errors;
  PrintValues(errors, _prefix, true, false, _config);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void Element::PrintValues(sdf::Errors &_errors, std::string _prefix,
                          const PrintConfig &_config) const
{
  std::ostringstream ss;
  PrintValuesImpl(_errors, _prefix, true, false, _config, ss);
  std::cout << ss.str();
}

/////////////////////////////////////////////////
void Element::PrintValues(const std::string &_prefix,
                          bool _includeDefaultElements,
                          bool _includeDefaultAttributes,
                          const PrintConfig &_config) const
{
  sdf::Errors errors;
  PrintValues(errors,
              _prefix,
              _includeDefaultElements,
              _includeDefaultAttributes,
              _config);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void Element::PrintValues(sdf::Errors &_errors,
                          const std::string &_prefix,
                          bool _includeDefaultElements,
                          bool _includeDefaultAttributes,
                          const PrintConfig &_config) const
{
  std::ostringstream ss;
  PrintValuesImpl(_errors,
                  _prefix,
                  _includeDefaultElements,
                  _includeDefaultAttributes,
                  _config,
                  ss);
  std::cout << ss.str();
}

/////////////////////////////////////////////////
std::string Element::ToString(const std::string &_prefix,
                              const PrintConfig &_config) const
{
  sdf::Errors errors;
  std::string out = this->ToString(errors, _prefix, _config);
  sdf::throwOrPrintErrors(errors);
  return out;
}

/////////////////////////////////////////////////
std::string Element::ToString(sdf::Errors &_errors,
                              const std::string &_prefix,
                              const PrintConfig &_config) const
{
  return this->ToString(_errors, _prefix, true, false, _config);
}

/////////////////////////////////////////////////
std::string Element::ToString(const std::string &_prefix,
                              bool _includeDefaultElements,
                              bool _includeDefaultAttributes,
                              const PrintConfig &_config) const
{
  sdf::Errors errors;
  std::ostringstream out;
  this->ToString(errors,
                 out,
                 _prefix,
                 _includeDefaultElements,
                 _includeDefaultAttributes,
                 _config);
  sdf::throwOrPrintErrors(errors);
  return out.str();
}

/////////////////////////////////////////////////
std::string Element::ToString(sdf::Errors &_errors,
                              const std::string &_prefix,
                              bool _includeDefaultElements,
                              bool _includeDefaultAttributes,
                              const PrintConfig &_config) const
{
  std::ostringstream out;
  this->ToString(_errors,
                 out,
                 _prefix,
                 _includeDefaultElements,
                 _includeDefaultAttributes,
                 _config);
  return out.str();
}

/////////////////////////////////////////////////
void Element::ToString(sdf::Errors &_errors,
                       std::ostringstream &_out,
                       const std::string &_prefix,
                       bool _includeDefaultElements,
                       bool _includeDefaultAttributes,
                       const PrintConfig &_config) const
{
  PrintValuesImpl(_errors,
                  _prefix,
                  _includeDefaultElements,
                  _includeDefaultAttributes,
                  _config,
                  _out);
}

/////////////////////////////////////////////////
bool Element::HasAttribute(const std::string &_key) const
{
  return this->GetAttribute(_key) != nullptr;
}

/////////////////////////////////////////////////
bool Element::GetAttributeSet(const std::string &_key) const
{
  bool result = false;
  ParamPtr p = this->GetAttribute(_key);
  if (p)
  {
    result = p->GetSet();
  }

  return result;
}

/////////////////////////////////////////////////
void Element::RemoveAttribute(const std::string &_key)
{
  Param_V::const_iterator iter;
  for (iter = this->dataPtr->attributes.begin();
      iter != this->dataPtr->attributes.end(); ++iter)
  {
    if ((*iter)->GetKey() == _key)
    {
      this->dataPtr->attributes.erase(iter);
      break;
    }
  }
}

/////////////////////////////////////////////////
void Element::RemoveAllAttributes()
{
  this->dataPtr->attributes.clear();
}

/////////////////////////////////////////////////
ParamPtr Element::GetAttribute(const std::string &_key) const
{
  Param_V::const_iterator iter;
  for (iter = this->dataPtr->attributes.begin();
      iter != this->dataPtr->attributes.end(); ++iter)
  {
    if ((*iter)->GetKey() == _key)
    {
      return (*iter);
    }
  }
  return ParamPtr();
}

/////////////////////////////////////////////////
size_t Element::GetAttributeCount() const
{
  return this->dataPtr->attributes.size();
}

/////////////////////////////////////////////////
const Param_V &Element::GetAttributes() const
{
  return this->dataPtr->attributes;
}

/////////////////////////////////////////////////
ParamPtr Element::GetAttribute(unsigned int _index) const
{
  ParamPtr result;
  if (_index < this->dataPtr->attributes.size())
  {
    result = this->dataPtr->attributes[_index];
  }

  return result;
}

/////////////////////////////////////////////////
size_t Element::GetElementDescriptionCount() const
{
  return this->dataPtr->elementDescriptions.size();
}

/////////////////////////////////////////////////
ElementPtr Element::GetElementDescription(unsigned int _index) const
{
  ElementPtr result;
  if (_index < this->dataPtr->elementDescriptions.size())
  {
    result = this->dataPtr->elementDescriptions[_index];
  }
  return result;
}

/////////////////////////////////////////////////
ElementPtr Element::GetElementDescription(const std::string &_key) const
{
  ElementPtr_V::const_iterator iter;
  for (iter = this->dataPtr->elementDescriptions.begin();
       iter != this->dataPtr->elementDescriptions.end(); ++iter)
  {
    if ((*iter)->GetName() == _key)
    {
      return (*iter);
    }
  }

  return ElementPtr();
}

/////////////////////////////////////////////////
ParamPtr Element::GetValue() const
{
  return this->dataPtr->value;
}

/////////////////////////////////////////////////
bool Element::HasElement(const std::string &_name) const
{
  return this->GetElementImpl(_name) != ElementPtr();
}

/////////////////////////////////////////////////
ElementPtr Element::GetElementImpl(const std::string &_name) const
{
  ElementPtr_V::const_iterator iter;
  for (iter = this->dataPtr->elements.begin();
       iter != this->dataPtr->elements.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
    {
      return (*iter);
    }
  }

  return ElementPtr();
}

/////////////////////////////////////////////////
ElementPtr Element::GetFirstElement() const
{
  if (this->dataPtr->elements.empty())
  {
    return ElementPtr();
  }
  else
  {
    return this->dataPtr->elements.front();
  }
}

/////////////////////////////////////////////////
ElementPtr Element::GetNextElement(const std::string &_name) const
{
  auto parent = this->dataPtr->parent.lock();
  if (parent)
  {
    ElementPtr_V::const_iterator iter;
    iter = std::find(parent->dataPtr->elements.begin(),
        parent->dataPtr->elements.end(), shared_from_this());

    if (iter == parent->dataPtr->elements.end())
    {
      return ElementPtr();
    }

    ++iter;
    if (iter == parent->dataPtr->elements.end())
    {
      return ElementPtr();
    }
    else if (_name.empty())
    {
      return *(iter);
    }
    else
    {
      for (; iter != parent->dataPtr->elements.end(); ++iter)
      {
        if ((*iter)->GetName() == _name)
        {
          return (*iter);
        }
      }
    }
  }

  return ElementPtr();
}

/////////////////////////////////////////////////
std::set<std::string> Element::GetElementTypeNames() const
{
  std::set<std::string> result;
  auto elem = this->GetFirstElement();
  while (elem)
  {
    std::string typeName = elem->GetName();
    result.insert(typeName);
    elem = elem->GetNextElement();
  }
  return result;
}

/////////////////////////////////////////////////
bool Element::HasUniqueChildNames(const std::string &_type) const
{
  return this->HasUniqueChildNames(_type, {});
}

/////////////////////////////////////////////////
bool Element::HasUniqueChildNames(
    sdf::Errors &_errors,
    const std::string &_type) const
{
  return this->HasUniqueChildNames(_errors, _type, {});
}

/////////////////////////////////////////////////
bool Element::HasUniqueChildNames(
    const std::string &_type,
    const std::vector<std::string> &_ignoreElements) const
{
  sdf::Errors errors;
  bool result = this->HasUniqueChildNames(errors, _type, _ignoreElements);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
bool Element::HasUniqueChildNames(
    sdf::Errors &_errors,
    const std::string &_type,
    const std::vector<std::string> &_ignoreElements) const
{
  auto namedElementsCount = this->CountNamedElements(
      _errors, _type, _ignoreElements);
  for (auto &iter : namedElementsCount)
  {
    if (iter.second > 1)
    {
      return false;
    }
  }
  return true;
}

/////////////////////////////////////////////////
std::map<std::string, std::size_t> Element::CountNamedElements(
    const std::string &_type) const
{
  return this->CountNamedElements(_type, {});
}

/////////////////////////////////////////////////
std::map<std::string, std::size_t> Element::CountNamedElements(
    sdf::Errors &_errors,
    const std::string &_type) const
{
  return this->CountNamedElements(_errors, _type, {});
}

/////////////////////////////////////////////////
std::map<std::string, std::size_t> Element::CountNamedElements(
    const std::string &_type,
    const std::vector<std::string> &_ignoreElements) const
{
  sdf::Errors errors;
  auto result = this->CountNamedElements(errors, _type, _ignoreElements);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
std::map<std::string, std::size_t> Element::CountNamedElements(
    sdf::Errors &_errors,
    const std::string &_type,
    const std::vector<std::string> &_ignoreElements) const
{
  std::map<std::string, std::size_t> result;

  sdf::ElementPtr elem;
  if (_type.empty())
  {
    elem = this->GetFirstElement();
  }
  else
  {
    elem = this->GetElementImpl(_type);
  }

  while (elem)
  {
    auto ignoreIt = std::find(_ignoreElements.begin(), _ignoreElements.end(),
                              elem->GetName());
    if (elem->HasAttribute("name") && ignoreIt == _ignoreElements.end())
    {
      // Get("name") returns attribute value if it exists before checking
      // for the value of a child element <name>, so it's safe to use
      // here since we've checked HasAttribute("name").
      std::string childNameAttributeValue = elem->Get<std::string>(
          _errors, "name");
      if (result.find(childNameAttributeValue) == result.end())
      {
        result[childNameAttributeValue] = 1;
      }
      else
      {
        ++result[childNameAttributeValue];
      }
    }

    elem = elem->GetNextElement(_type);
  }

  return result;
}

/////////////////////////////////////////////////
ElementPtr Element::GetElement(const std::string &_name)
{
  sdf::Errors errors;
  ElementPtr result = this->GetElement(_name, errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
ElementPtr Element::GetElement(const std::string &_name, sdf::Errors &_errors)
{
  ElementPtr result = this->GetElementImpl(_name);
  if (result == ElementPtr())
  {
    result = this->AddElement(_name, _errors);
  }

  return result;
}

/////////////////////////////////////////////////
ElementPtr Element::FindElement(const std::string &_name)
{
  return this->GetElementImpl(_name);
}

/////////////////////////////////////////////////
ElementConstPtr Element::FindElement(const std::string &_name) const
{
  return this->GetElementImpl(_name);
}

/////////////////////////////////////////////////
void Element::InsertElement(ElementPtr _elem)
{
  this->dataPtr->elements.push_back(_elem);
}

/////////////////////////////////////////////////
void Element::InsertElement(ElementPtr _elem,  bool _setParentToSelf)
{
  if (_setParentToSelf)
    _elem->SetParent(shared_from_this());
  this->dataPtr->elements.push_back(_elem);
}

/////////////////////////////////////////////////
bool Element::HasElementDescription(const std::string &_name) const
{
  return this->GetElementDescription(_name) != ElementPtr();
}

/////////////////////////////////////////////////
ElementPtr Element::AddElement(const std::string &_name)
{
  sdf::Errors errors;
  ElementPtr elem = this->AddElement(_name, errors);
  sdf::throwOrPrintErrors(errors);
  return elem;
}

/////////////////////////////////////////////////
ElementPtr Element::AddElement(const std::string &_name, sdf::Errors &_errors)
{
  // if this element is a reference sdf and does not have any element
  // descriptions then get them from its parent
  auto parent = this->dataPtr->parent.lock();
  if (!this->dataPtr->referenceSDF.empty() &&
      this->dataPtr->elementDescriptions.empty() && parent &&
      parent->GetName() == this->dataPtr->name)
  {
    for (unsigned int i = 0; i < parent->GetElementDescriptionCount(); ++i)
    {
      this->dataPtr->elementDescriptions.push_back(
        parent->GetElementDescription(i)->Clone(_errors));
    }
  }

  ElementPtr_V::const_iterator iter, iter2;
  for (iter = this->dataPtr->elementDescriptions.begin();
      iter != this->dataPtr->elementDescriptions.end(); ++iter)
  {
    if ((*iter)->dataPtr->name == _name)
    {
      ElementPtr elem = (*iter)->Clone(_errors);
      elem->SetParent(shared_from_this());
      this->dataPtr->elements.push_back(elem);

      // Add all child elements.
      for (iter2 = elem->dataPtr->elementDescriptions.begin();
           iter2 != elem->dataPtr->elementDescriptions.end(); ++iter2)
      {
        // Add only required child element
        if ((*iter2)->GetRequired() == "1")
        {
          elem->AddElement((*iter2)->dataPtr->name, _errors);
        }
      }
      return this->dataPtr->elements.back();
    }
  }

  _errors.push_back({ErrorCode::ELEMENT_ERROR,
      "Missing element description for [" + _name + "]\n"});
  return ElementPtr();
}

/////////////////////////////////////////////////
void Element::Clear()
{
  this->ClearElements();
  this->dataPtr->originalVersion.clear();
  this->dataPtr->path.clear();
  this->dataPtr->lineNumber = std::nullopt;
  this->dataPtr->xmlPath.clear();
}

/////////////////////////////////////////////////
void Element::ClearElements()
{
  for (sdf::ElementPtr_V::iterator iter = this->dataPtr->elements.begin();
      iter != this->dataPtr->elements.end(); ++iter)
  {
    (*iter)->ClearElements();
  }

  this->dataPtr->elements.clear();
}


/////////////////////////////////////////////////
void Element::Update()
{
  sdf::Errors errors;
  this->Update(errors);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void Element::Update(sdf::Errors &_errors)
{
  for (sdf::Param_V::iterator iter = this->dataPtr->attributes.begin();
      iter != this->dataPtr->attributes.end(); ++iter)
  {
    (*iter)->Update(_errors);
  }

  for (sdf::ElementPtr_V::iterator iter = this->dataPtr->elements.begin();
      iter != this->dataPtr->elements.end(); ++iter)
  {
    (*iter)->Update(_errors);
  }

  if (this->dataPtr->value)
  {
    this->dataPtr->value->Update(_errors);
  }
}

/////////////////////////////////////////////////
void Element::Reset()
{
  for (ElementPtr_V::iterator iter = this->dataPtr->elements.begin();
      iter != this->dataPtr->elements.end(); ++iter)
  {
    if (*iter)
    {
      (*iter)->Reset();
    }
    (*iter).reset();
  }

  for (ElementPtr_V::iterator iter = this->dataPtr->elementDescriptions.begin();
      iter != this->dataPtr->elementDescriptions.end(); ++iter)
  {
    if (*iter)
    {
      (*iter)->Reset();
    }
    (*iter).reset();
  }
  this->dataPtr->elements.clear();
  this->dataPtr->elementDescriptions.clear();

  this->dataPtr->value.reset();

  this->dataPtr->parent.reset();
}

/////////////////////////////////////////////////
void Element::AddElementDescription(ElementPtr _elem)
{
  this->dataPtr->elementDescriptions.push_back(_elem);
}

/////////////////////////////////////////////////
void Element::SetIncludeElement(sdf::ElementPtr _includeElem)
{
  this->dataPtr->includeElement = _includeElem;
}

/////////////////////////////////////////////////
sdf::ElementPtr Element::GetIncludeElement() const
{
  return this->dataPtr->includeElement;
}

/////////////////////////////////////////////////
void Element::SetFilePath(const std::string &_path)
{
  this->dataPtr->path = _path;
}

/////////////////////////////////////////////////
const std::string &Element::FilePath() const
{
  return this->dataPtr->path;
}

/////////////////////////////////////////////////
void Element::SetLineNumber(int _lineNumber)
{
  this->dataPtr->lineNumber = _lineNumber;
}

/////////////////////////////////////////////////
std::optional<int> Element::LineNumber() const
{
  return this->dataPtr->lineNumber;
}

/////////////////////////////////////////////////
void Element::SetXmlPath(const std::string &_path)
{
  this->dataPtr->xmlPath = _path;
}

/////////////////////////////////////////////////
const std::string &Element::XmlPath() const
{
  return this->dataPtr->xmlPath;
}

/////////////////////////////////////////////////
void Element::SetOriginalVersion(const std::string &_version)
{
  this->dataPtr->originalVersion = _version;
}

/////////////////////////////////////////////////
const std::string &Element::OriginalVersion() const
{
  return this->dataPtr->originalVersion;
}

/////////////////////////////////////////////////
std::string Element::GetDescription() const
{
  return this->dataPtr->description;
}

/////////////////////////////////////////////////
void Element::SetDescription(const std::string &_desc)
{
  this->dataPtr->description = _desc;
}

/////////////////////////////////////////////////
void Element::RemoveFromParent()
{
  auto parent = this->dataPtr->parent.lock();
  if (parent)
  {
    ElementPtr_V::iterator iter;
    iter = std::find(parent->dataPtr->elements.begin(),
        parent->dataPtr->elements.end(), shared_from_this());

    if (iter != parent->dataPtr->elements.end())
    {
      parent->dataPtr->elements.erase(iter);
      parent.reset();
    }
  }
}

/////////////////////////////////////////////////
void Element::RemoveChild(ElementPtr _child)
{
  sdf::Errors errors;
  RemoveChild(_child, errors);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void Element::RemoveChild(ElementPtr _child, sdf::Errors &)
{
  SDF_ASSERT(_child, "Cannot remove a nullptr child pointer");

  ElementPtr_V::iterator iter;
  iter = std::find(this->dataPtr->elements.begin(),
                   this->dataPtr->elements.end(), _child);

  if (iter != this->dataPtr->elements.end())
  {
    _child->SetParent(ElementPtr());
    this->dataPtr->elements.erase(iter);
  }
}

/////////////////////////////////////////////////
std::any Element::GetAny(const std::string &_key) const
{
  sdf::Errors errors;
  std::any result = this->GetAny(errors, _key);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
std::any Element::GetAny(sdf::Errors &_errors, const std::string &_key) const
{
  std::any result;
  if (_key.empty() && this->dataPtr->value)
  {
    if (!this->dataPtr->value->GetAny(result, _errors))
    {
        _errors.push_back({ErrorCode::ELEMENT_ERROR,
            "Couldn't get element [" + this->GetName() + "] as std::any\n"});
    }
  }
  else if (!_key.empty())
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
    {
      if (!this->GetAttribute(_key)->GetAny(result, _errors))
      {
        _errors.push_back({ErrorCode::ELEMENT_ERROR,
            "Couldn't get attribute [" + _key + "] as std::any\n"});
      }
    }
    else
    {
      ElementPtr tmp = this->GetElementImpl(_key);
      if (tmp != ElementPtr())
      {
        result = tmp->GetAny(_errors);
      }
      else
      {
        tmp = this->GetElementDescription(_key);
        if (tmp != ElementPtr())
        {
          result = tmp->GetAny(_errors);
        }
        else
        {
          _errors.push_back({ErrorCode::ELEMENT_ERROR,
              "Unable to find value for key [" + _key + "]\n"});
        }
      }
    }
  }
  return result;
}

//////////////////////////////////////////////////
std::vector<std::string> Element::NameUniquenessExceptions()
{
  // We make exception for "plugin" when checking for name uniqueness.
  return {"plugin"};
}
