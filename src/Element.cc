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

#include "sdf/Assert.hh"
#include "sdf/Element.hh"

using namespace sdf;

/////////////////////////////////////////////////
Element::Element()
{
  this->copyChildren = false;
  this->nestedSDF = false;
}

/////////////////////////////////////////////////
Element::~Element()
{
  this->parent.reset();
  for (Param_V::iterator iter = this->attributes.begin();
      iter != this->attributes.end(); ++iter)
  {
    (*iter).reset();
  }
  this->attributes.clear();

  for (ElementPtr_V::iterator iter = this->elements.begin();
      iter != this->elements.end(); ++iter)
  {
    (*iter).reset();
  }

  for (ElementPtr_V::iterator iter = this->elementDescriptions.begin();
      iter != this->elementDescriptions.end(); ++iter)
  {
    (*iter).reset();
  }
  this->elements.clear();
  this->elementDescriptions.clear();

  this->value.reset();

  // this->Reset();
}

/////////////////////////////////////////////////
ElementPtr Element::GetParent() const
{
  return this->parent;
}

/////////////////////////////////////////////////
void Element::SetParent(const ElementPtr _parent)
{
  this->parent = _parent;
}

/////////////////////////////////////////////////
void Element::SetName(const std::string &_name)
{
  this->name = _name;
}

/////////////////////////////////////////////////
const std::string &Element::GetName() const
{
  return this->name;
}

/////////////////////////////////////////////////
void Element::SetRequired(const std::string &_req)
{
  this->required = _req;
}

/////////////////////////////////////////////////
const std::string &Element::GetRequired() const
{
  return this->required;
}

/////////////////////////////////////////////////
void Element::SetCopyChildren(bool _value)
{
  this->copyChildren = _value;
}

/////////////////////////////////////////////////
bool Element::GetCopyChildren() const
{
  return this->copyChildren;
}

/////////////////////////////////////////////////
void Element::SetNestedSDF(bool _value)
{
  this->nestedSDF = _value;
}

/////////////////////////////////////////////////
bool Element::GetNestedSDF() const
{
  return this->nestedSDF;
}

/////////////////////////////////////////////////
void Element::AddValue(const std::string &_type,
    const std::string &_defaultValue, bool _required,
    const std::string &_description)
{
  this->value = this->CreateParam(this->name, _type, _defaultValue, _required,
      _description);
}

/////////////////////////////////////////////////
boost::shared_ptr<Param> Element::CreateParam(const std::string &_key,
    const std::string &_type, const std::string &_defaultValue, bool _required,
    const std::string &_description)
{
  return boost::shared_ptr<Param>(
        new Param(_key, _type, _defaultValue, _required, _description));
}

/////////////////////////////////////////////////
void Element::AddAttribute(const std::string &_key, const std::string &_type,
    const std::string &_defaultValue, bool _required,
    const std::string &_description)
{
  this->attributes.push_back(
      this->CreateParam(_key, _type, _defaultValue, _required, _description));
}

/////////////////////////////////////////////////
ElementPtr Element::Clone() const
{
  ElementPtr clone(new Element);
  clone->description = this->description;
  clone->name = this->name;
  clone->required = this->required;
  // clone->parent = this->parent;
  clone->copyChildren = this->copyChildren;
  clone->nestedSDF = this->nestedSDF;
  clone->includeFilename = this->includeFilename;

  Param_V::const_iterator aiter;
  for (aiter = this->attributes.begin();
      aiter != this->attributes.end(); ++aiter)
  {
    clone->attributes.push_back((*aiter)->Clone());
  }

  ElementPtr_V::const_iterator eiter;
  for (eiter = this->elementDescriptions.begin();
      eiter != this->elementDescriptions.end(); ++eiter)
  {
    clone->elementDescriptions.push_back((*eiter)->Clone());
  }

  for (eiter = this->elements.begin(); eiter != this->elements.end(); ++eiter)
  {
    clone->elements.push_back((*eiter)->Clone());
    clone->elements.back()->parent = clone;
  }

  if (this->value)
    clone->value = this->value->Clone();

  return clone;
}

/////////////////////////////////////////////////
void Element::Copy(const ElementPtr _elem)
{
  this->name = _elem->GetName();
  this->description = _elem->GetDescription();
  this->required = _elem->GetRequired();
  this->copyChildren = _elem->GetCopyChildren();
  this->nestedSDF = _elem->GetNestedSDF();
  this->includeFilename = _elem->includeFilename;

  for (Param_V::iterator iter = _elem->attributes.begin();
       iter != _elem->attributes.end(); ++iter)
  {
    if (!this->HasAttribute((*iter)->GetKey()))
      this->attributes.push_back((*iter)->Clone());
    ParamPtr param = this->GetAttribute((*iter)->GetKey());
    (*param) = (**iter);
  }

  if (_elem->GetValue())
  {
    if (!this->value)
      this->value = _elem->GetValue()->Clone();
    else
      *(this->value) = *(_elem->GetValue());
  }

  this->elementDescriptions.clear();
  for (ElementPtr_V::const_iterator iter = _elem->elementDescriptions.begin();
       iter != _elem->elementDescriptions.end(); ++iter)
  {
    this->elementDescriptions.push_back((*iter)->Clone());
  }

  this->elements.clear();
  for (ElementPtr_V::iterator iter = _elem->elements.begin();
       iter != _elem->elements.end(); ++iter)
  {
    ElementPtr elem = (*iter)->Clone();
    elem->Copy(*iter);
    elem->parent = shared_from_this();
    this->elements.push_back(elem);
  }
}

/////////////////////////////////////////////////
void Element::PrintDescription(const std::string &_prefix)
{
  std::cout << _prefix << "<element name ='" << this->name
            << "' required ='" << this->required << "'>\n";

  std::cout << _prefix << "  <description>" << this->description
            << "</description>\n";

  Param_V::iterator aiter;
  for (aiter = this->attributes.begin();
      aiter != this->attributes.end(); ++aiter)
  {
    std::cout << _prefix << "  <attribute name ='"
              << (*aiter)->GetKey() << "' type ='" << (*aiter)->GetTypeName()
              << "' default ='" << (*aiter)->GetDefaultAsString()
              << "' required ='" << (*aiter)->GetRequired() << "'>\n";
    std::cout << _prefix << "    <description>" << (*aiter)->GetDescription()
              << "</description>\n";
    std::cout << _prefix << "  </attribute>\n";
  }

  if (this->GetCopyChildren())
    std::cout << _prefix << "  <element copy_data ='true' required ='*'/>\n";

  if (this->GetNestedSDF())
    std::cout << _prefix << "  <element nested_sdf ='true' required ='*'/>\n";

  ElementPtr_V::iterator eiter;
  for (eiter = this->elementDescriptions.begin();
      eiter != this->elementDescriptions.end(); ++eiter)
  {
    (*eiter)->PrintDescription(_prefix + "  ");
  }

  std::cout << _prefix << "</element>\n";
}

/////////////////////////////////////////////////
void Element::PrintDocRightPane(std::string &_html, int _spacing, int &_index)
{
  std::ostringstream stream;
  ElementPtr_V::iterator eiter;

  int start = _index++;

  std::string childHTML;
  for (eiter = this->elementDescriptions.begin();
      eiter != this->elementDescriptions.end(); ++eiter)
  {
    (*eiter)->PrintDocRightPane(childHTML, _spacing + 4, _index);
  }

  stream << "<a name=\"" << this->name << start
         << "\">&lt" << this->name << "&gt</a>";

  stream << "<div style='padding-left:" << _spacing << "px;'>\n";

  stream << "<div style='background-color: #ffffff'>\n";

  stream << "<font style='font-weight:bold'>Description: </font>";
  if (!this->description.empty())
    stream << this->description << "<br>\n";
  else
    stream << "none<br>\n";

  stream << "<font style='font-weight:bold'>Required: </font>"
         << this->required << "&nbsp;&nbsp;&nbsp;\n";

  stream << "<font style='font-weight:bold'>Type: </font>";
  if (this->value)
  {
    stream << this->value->GetTypeName()
           << "&nbsp;&nbsp;&nbsp;\n"
           << "<font style='font-weight:bold'>Default: </font>"
           << this->value->GetDefaultAsString() << '\n';
  }
  else
    stream << "n/a\n";

  stream << "</div>";

  if (this->attributes.size() > 0)
  {
    stream << "<div style='background-color: #dedede; padding-left:10px; "
           << "display:inline-block;'>\n";
    stream << "<font style='font-weight:bold'>Attributes</font><br>";

    Param_V::iterator aiter;
    for (aiter = this->attributes.begin();
        aiter != this->attributes.end(); ++aiter)
    {
      stream << "<div style='display: inline-block;padding-bottom: 4px;'>\n";

      stream << "<div style='float:left; width: 80px;'>\n";
      stream << "<font style='font-style: italic;'>" << (*aiter)->GetKey()
        << "</font>: ";
      stream << "</div>\n";

      stream << "<div style='float:left; padding-left: 4px; width: 300px;'>\n";

      if (!(*aiter)->GetDescription().empty())
          stream << (*aiter)->GetDescription() << "<br>\n";
      else
          stream << "no description<br>\n";

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
void Element::PrintDocLeftPane(std::string &_html, int _spacing, int &_index)
{
  std::ostringstream stream;
  ElementPtr_V::iterator eiter;

  int start = _index++;

  std::string childHTML;
  for (eiter = this->elementDescriptions.begin();
      eiter != this->elementDescriptions.end(); ++eiter)
  {
    (*eiter)->PrintDocLeftPane(childHTML, _spacing + 4, _index);
  }

  stream << "<a id='" << start << "' onclick='highlight(" << start
         << ");' href=\"#" << this->name << start
         << "\">&lt" << this->name << "&gt</a>";

  stream << "<div style='padding-left:" << _spacing << "px;'>\n";

  _html += stream.str();
  _html += childHTML;
  _html += "</div>\n";
}

/////////////////////////////////////////////////
void Element::PrintValues(std::string _prefix)
{
  std::cout << _prefix << "<" << this->name;

  Param_V::iterator aiter;
  for (aiter = this->attributes.begin();
       aiter != this->attributes.end(); ++aiter)
  {
    std::cout << " " << (*aiter)->GetKey() << "='"
      << (*aiter)->GetAsString() << "'";
  }

  if (this->elements.size() > 0)
  {
    std::cout << ">\n";
    ElementPtr_V::iterator eiter;
    for (eiter = this->elements.begin();
        eiter != this->elements.end(); ++eiter)
    {
      (*eiter)->PrintValues(_prefix + "  ");
    }
    std::cout << _prefix << "</" << this->name << ">\n";
  }
  else
  {
    if (this->value)
    {
      std::cout << ">" << this->value->GetAsString()
        << "</" << this->name << ">\n";
    }
    else
    {
      std::cout << "/>\n";
    }
  }
}

/////////////////////////////////////////////////
std::string Element::ToString(const std::string &_prefix) const
{
  std::ostringstream out;
  this->ToString(_prefix, out);
  return out.str();
}

/////////////////////////////////////////////////
void Element::ToString(const std::string &_prefix,
                       std::ostringstream &_out) const
{
  if (this->includeFilename.empty())
  {
    _out << _prefix << "<" << this->name;

    Param_V::const_iterator aiter;
    for (aiter = this->attributes.begin();
        aiter != this->attributes.end(); ++aiter)
    {
      _out << " " << (*aiter)->GetKey() << "='"
           << (*aiter)->GetAsString() << "'";
    }

    if (this->elements.size() > 0)
    {
      _out << ">\n";
      ElementPtr_V::const_iterator eiter;
      for (eiter = this->elements.begin();
          eiter != this->elements.end(); ++eiter)
      {
        (*eiter)->ToString(_prefix + "  ", _out);
      }
      _out << _prefix << "</" << this->name << ">\n";
    }
    else
    {
      if (this->value)
      {
        _out << ">" << this->value->GetAsString()
             << "</" << this->name << ">\n";
      }
      else
      {
        _out << "/>\n";
      }
    }
  }
  else
  {
    _out << _prefix << "<include filename='"
         << this->includeFilename << "'/>\n";
  }
}

/////////////////////////////////////////////////
bool Element::HasAttribute(const std::string &_key)
{
  return this->GetAttribute(_key) != NULL;
}

/////////////////////////////////////////////////
bool Element::GetAttributeSet(const std::string &_key)
{
  bool result = false;
  ParamPtr p = this->GetAttribute(_key);
  if (p)
    result = p->GetSet();

  return result;
}

/////////////////////////////////////////////////
ParamPtr Element::GetAttribute(const std::string &_key)
{
  Param_V::const_iterator iter;
  for (iter = this->attributes.begin();
      iter != this->attributes.end(); ++iter)
  {
    if ((*iter)->GetKey() == _key)
      return (*iter);
  }
  return ParamPtr();
}

/////////////////////////////////////////////////
unsigned int Element::GetAttributeCount() const
{
  return this->attributes.size();
}

/////////////////////////////////////////////////
ParamPtr Element::GetAttribute(unsigned int _index) const
{
  ParamPtr result;
  if (_index < this->attributes.size())
    result = this->attributes[_index];

  return result;
}

/////////////////////////////////////////////////
unsigned int Element::GetElementDescriptionCount() const
{
  return this->elementDescriptions.size();
}

/////////////////////////////////////////////////
ElementPtr Element::GetElementDescription(unsigned int _index) const
{
  ElementPtr result;
  if (_index < this->elementDescriptions.size())
    result = this->elementDescriptions[_index];
  return result;
}

/////////////////////////////////////////////////
ElementPtr Element::GetElementDescription(const std::string &_key) const
{
  ElementPtr_V::const_iterator iter;
  for (iter = this->elementDescriptions.begin();
       iter != this->elementDescriptions.end(); ++iter)
  {
    if ((*iter)->GetName() == _key)
      return (*iter);
  }

  return ElementPtr();
}

/////////////////////////////////////////////////
ParamPtr Element::GetValue()
{
  return this->value;
}

/////////////////////////////////////////////////
bool Element::HasElement(const std::string &_name) const
{
  ElementPtr_V::const_iterator iter;
  for (iter = this->elements.begin(); iter != this->elements.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
      return true;
  }

  return false;
}

/////////////////////////////////////////////////
ElementPtr Element::GetElementImpl(const std::string &_name) const
{
  ElementPtr_V::const_iterator iter;
  for (iter = this->elements.begin(); iter != this->elements.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
      return (*iter);
  }

  // gzdbg << "Unable to find element [" << _name << "] return empty\n";
  return ElementPtr();
}

/////////////////////////////////////////////////
ElementPtr Element::GetFirstElement() const
{
  if (this->elements.empty())
    return ElementPtr();
  else
    return this->elements.front();
}

/////////////////////////////////////////////////
ElementPtr Element::GetNextElement(const std::string &_name) const
{
  if (this->parent)
  {
    ElementPtr_V::const_iterator iter;
    iter = std::find(this->parent->elements.begin(),
        this->parent->elements.end(), shared_from_this());

    if (iter == this->parent->elements.end())
    {
      return ElementPtr();
    }

    ++iter;
    if (iter == this->parent->elements.end())
      return ElementPtr();
    else if (_name.empty())
      return *(iter);
    else
    {
      for (; iter != this->parent->elements.end(); ++iter)
      {
        if ((*iter)->GetName() == _name)
          return (*iter);
      }
    }
  }

  return ElementPtr();
}

/////////////////////////////////////////////////
ElementPtr Element::GetElement(const std::string &_name)
{
  if (this->HasElement(_name))
    return this->GetElementImpl(_name);
  else
    return this->AddElement(_name);
}

/////////////////////////////////////////////////
void Element::InsertElement(ElementPtr _elem)
{
  this->elements.push_back(_elem);
}

/////////////////////////////////////////////////
bool Element::HasElementDescription(const std::string &_name)
{
  bool result = false;
  ElementPtr_V::const_iterator iter;
  for (iter = this->elementDescriptions.begin();
       iter != this->elementDescriptions.end(); ++iter)
  {
    if ((*iter)->name == _name)
    {
      result = true;
      break;
    }
  }

  return result;
}

/////////////////////////////////////////////////
ElementPtr Element::AddElement(const std::string &_name)
{
  ElementPtr_V::const_iterator iter, iter2;
  for (iter = this->elementDescriptions.begin();
      iter != this->elementDescriptions.end(); ++iter)
  {
    if ((*iter)->name == _name)
    {
      ElementPtr elem = (*iter)->Clone();
      elem->SetParent(shared_from_this());
      this->elements.push_back(elem);

      // Add all child elements.
      for (iter2 = elem->elementDescriptions.begin();
           iter2 != elem->elementDescriptions.end(); ++iter2)
      {
        // Add only required child element
        if ((*iter2)->GetRequired() == "1")
        {
          elem->AddElement((*iter2)->name);
        }
      }

      return this->elements.back();
    }
  }
  sdferr << "Missing element description for [" << _name << "]\n";
  return ElementPtr();
}

/////////////////////////////////////////////////
void Element::ClearElements()
{
  for (sdf::ElementPtr_V::iterator iter = this->elements.begin();
      iter != this->elements.end(); ++iter)
  {
    (*iter)->ClearElements();
  }

  this->elements.clear();
}

/////////////////////////////////////////////////
void Element::Update()
{
  for (sdf::Param_V::iterator iter = this->attributes.begin();
      iter != this->attributes.end(); ++iter)
  {
    (*iter)->Update();
  }

  for (sdf::ElementPtr_V::iterator iter = this->elements.begin();
      iter != this->elements.end(); ++iter)
  {
    (*iter)->Update();
  }

  if (this->value)
    this->value->Update();
}

/////////////////////////////////////////////////
void Element::Reset()
{
  for (ElementPtr_V::iterator iter = this->elements.begin();
      iter != this->elements.end(); ++iter)
  {
    if (*iter)
      (*iter)->Reset();
    (*iter).reset();
  }

  for (ElementPtr_V::iterator iter = this->elementDescriptions.begin();
      iter != this->elementDescriptions.end(); ++iter)
  {
    if (*iter)
      (*iter)->Reset();
    (*iter).reset();
  }
  this->elements.clear();
  this->elementDescriptions.clear();

  this->value.reset();

  this->parent.reset();
}

/////////////////////////////////////////////////
void Element::AddElementDescription(ElementPtr _elem)
{
  this->elementDescriptions.push_back(_elem);
}

/////////////////////////////////////////////////
void Element::SetInclude(const std::string &_filename)
{
  this->includeFilename = _filename;
}

/////////////////////////////////////////////////
std::string Element::GetInclude() const
{
  return this->includeFilename;
}

/////////////////////////////////////////////////
std::string Element::GetDescription() const
{
  return this->description;
}

/////////////////////////////////////////////////
void Element::SetDescription(const std::string &_desc)
{
  this->description = _desc;
}

/////////////////////////////////////////////////
void Element::RemoveFromParent()
{
  if (this->parent)
  {
    ElementPtr_V::iterator iter;
    iter = std::find(this->parent->elements.begin(),
        this->parent->elements.end(), shared_from_this());

    if (iter != this->parent->elements.end())
    {
      this->parent->elements.erase(iter);
      this->parent.reset();
    }
  }
}


/////////////////////////////////////////////////
bool Element::GetValueBool(const std::string &_key)
{
  return this->Get<bool>(_key);
}

/////////////////////////////////////////////////
int Element::GetValueInt(const std::string &_key)
{
  return this->Get<int>(_key);
}

/////////////////////////////////////////////////
float Element::GetValueFloat(const std::string &_key)
{
  return this->Get<float>(_key);
}

/////////////////////////////////////////////////
double Element::GetValueDouble(const std::string &_key)
{
  return this->Get<double>(_key);
}

/////////////////////////////////////////////////
unsigned int Element::GetValueUInt(const std::string &_key)
{
  return this->Get<unsigned int>(_key);
}

/////////////////////////////////////////////////
char Element::GetValueChar(const std::string &_key)
{
  return this->Get<char>(_key);
}

/////////////////////////////////////////////////
std::string Element::GetValueString(const std::string &_key)
{
  return this->Get<std::string>(_key);
}

/////////////////////////////////////////////////
sdf::Vector3 Element::GetValueVector3(const std::string &_key)
{
  return this->Get<sdf::Vector3>(_key);
}

/////////////////////////////////////////////////
sdf::Vector2d Element::GetValueVector2d(const std::string &_key)
{
  return this->Get<sdf::Vector2d>(_key);
}

/////////////////////////////////////////////////
sdf::Quaternion Element::GetValueQuaternion(const std::string &_key)
{
  return this->Get<sdf::Quaternion>(_key);
}

/////////////////////////////////////////////////
sdf::Pose Element::GetValuePose(const std::string &_key)
{
  return this->Get<sdf::Pose>(_key);
}

/////////////////////////////////////////////////
sdf::Color Element::GetValueColor(const std::string &_key)
{
  return this->Get<sdf::Color>(_key);
}

/////////////////////////////////////////////////
sdf::Time Element::GetValueTime(const std::string &_key)
{
  return this->Get<sdf::Time>(_key);
}

/////////////////////////////////////////////////
boost::any Element::GetAny(const std::string &_key)
{
  boost::any result;
  if (_key.empty() && this->value)
  {
    if (!this->value->GetAny(result))
    {
      sdferr << "Couldn't get element [" << this->GetName()
             << "] as boost::any\n";
    }
  }
  else if (!_key.empty())
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
    {
      if (!this->GetAttribute(_key)->GetAny(result))
        sdferr << "Couldn't get attribute [" << _key << "] as boost::any\n";
    }
    else if (this->HasElement(_key))
      result = this->GetElementImpl(_key)->GetAny();
    else if (this->HasElementDescription(_key))
      result = this->GetElementDescription(_key)->GetAny();
    else
      sdferr << "Unable to find value for key [" << _key << "]\n";
  }
  return result;
}

/////////////////////////////////////////////////
void Element::RemoveChild(ElementPtr _child)
{
  SDF_ASSERT(_child, "Cannot remove a NULL child pointer");

  ElementPtr_V::iterator iter;
  iter = std::find(this->elements.begin(),
                   this->elements.end(), _child);

  if (iter != this->elements.end())
  {
    _child->SetParent(ElementPtr());
    this->elements.erase(iter);
  }
}

