/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include <boost/filesystem.hpp>
#include <map>
#include <list>

#include "sdf/parser.hh"
#include "sdf/Assert.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/sdf_config.h"

using namespace sdf;
std::string SDF::version = SDF_VERSION;

typedef std::list<boost::filesystem::path> PathList;
typedef std::map<std::string, PathList> URIPathMap;

URIPathMap g_uriPathMap;

boost::function<std::string (const std::string &)> g_findFileCB;

/////////////////////////////////////////////////
void sdf::setFindCallback(
    boost::function<std::string (const std::string &)> _cb)
{
  g_findFileCB = _cb;
}

/////////////////////////////////////////////////
std::string sdf::findFile(const std::string &_filename, bool _searchLocalPath,
    bool _useCallback)
{
  boost::filesystem::path path = _filename;

  // Check to see if _filename is URI. If so, resolve the URI path.
  for (URIPathMap::iterator iter = g_uriPathMap.begin();
       iter != g_uriPathMap.end(); ++iter)
  {
    // Check to see if the URI in the global map is the first part of the
    // given filename
    if (_filename.find(iter->first) == 0)
    {
      std::string suffix = _filename;
      boost::replace_first(suffix, iter->first, "");

      // Check each path in the list.
      for (PathList::iterator pathIter = iter->second.begin();
           pathIter != iter->second.end(); ++pathIter)
      {
        // Return the path string if the path + suffix exists.
        if (boost::filesystem::exists((*pathIter) / suffix))
          return ((*pathIter) / suffix).string();
      }
    }
  }

  // Next check the install path.
  path = boost::filesystem::path(SDF_SHARE_PATH) / _filename;
  if (boost::filesystem::exists(path))
    return path.string();

  // Next check the versioned install path.
  path = boost::filesystem::path(SDF_SHARE_PATH) / "sdformat" /
    sdf::SDF::version / _filename;
  if (boost::filesystem::exists(path))
    return path.string();

  // Next check SDF_PATH environment variable
  char *pathCStr = getenv("SDF_PATH");
  if (pathCStr)
  {
    std::vector<std::string> paths;
    boost::split(paths, pathCStr, boost::is_any_of(":"));
    for (std::vector<std::string>::iterator iter = paths.begin();
         iter != paths.end(); ++iter)
    {
      path = boost::filesystem::path(*iter) / _filename;
      if (boost::filesystem::exists(path))
        return path.string();
    }
  }

  // Next check to see if the given file exists.
  path = boost::filesystem::path(_filename);
  if (boost::filesystem::exists(path))
    return path.string();


  // Finally check the local path, if the flag is set.
  if (_searchLocalPath)
  {
    path = boost::filesystem::current_path() / _filename;

    if (boost::filesystem::exists(path))
      return path.string();
  }

  // If we still haven't found the file, use the registered callback if the
  // flag has been set
  if (_useCallback)
  {
    if (!g_findFileCB)
    {
      sdferr << "Tried to use callback in sdf::findFile(), but the callback "
        "is empty.  Did you call sdf::setFindCallback()?";
      return std::string();
    }
    else
      return g_findFileCB(_filename);
  }

  return std::string();
}

/////////////////////////////////////////////////
void sdf::addURIPath(const std::string &_uri, const std::string &_path)
{
  // Split _path on colons.
  std::list<std::string> parts;
  boost::split(parts, _path, boost::is_any_of(":"));

  // Add each part of the colon separated path to the global URI map.
  for (std::list<std::string>::iterator iter = parts.begin();
       iter != parts.end(); ++iter)
  {
    boost::filesystem::path path = *iter;

    // Only add valid paths
    if (!(*iter).empty() && boost::filesystem::exists(path) &&
        boost::filesystem::is_directory(path))
    {
      g_uriPathMap[_uri].push_back(path);
    }
  }
}

/////////////////////////////////////////////////
Element::Element()
{
  this->copyChildren = false;
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
              << "' required ='" << (*aiter)->GetRequired() << "'/>\n";
  }

  if (this->GetCopyChildren())
    std::cout << _prefix << "  <element copy_data ='true' required ='*'/>\n";

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

/////////////////////////////////////////////////
SDF::SDF()
: root(new Element)
{
}

/////////////////////////////////////////////////
void SDF::PrintDescription()
{
  this->root->PrintDescription("");
}

/////////////////////////////////////////////////
void SDF::PrintValues()
{
  this->root->PrintValues("");
}

/////////////////////////////////////////////////
void SDF::PrintDoc()
{
  std::string html, html2;
  int index = 0;
  this->root->PrintDocLeftPane(html, 10, index);

  index = 0;
  this->root->PrintDocRightPane(html2, 10, index);

  std::cout << "<!DOCTYPE HTML>\n"
  << "<html>\n"
  << "<head>\n"
  << "  <link href='style.css' rel='stylesheet' type='text/css'>\n"
  << "  <script type='text/javascript' src='jquery.js'></script>\n"
  << "  <script type='text/javascript' src='splitter-152.js'></script>\n"
  << "  <script type='text/javascript'>\n"
  << "    var prevId = 0;\n"
  << "  function highlight(id) {\n"
  << "    var elem = document.getElementById(prevId);\n"
  << "    elem.style.background = '#ffffff';\n"
  << "    elem.style.color = '#da7800';\n"
  << "    elem = document.getElementById(id);\n"
  << "    elem.style.background = '#da7800';\n"
  << "    elem.style.color = '#ffffff';\n"
  << "    prevId = id;\n"
  << "  }\n"
  << "  $().ready(function() {\n"
  << "    $('#my_splitter').splitter({\n"
  << "      splitVertical: true,\n"
  << "      outline: true,\n"
  << "      sizeLeft: true,\n"
  << "      resizeTo: window,\n"
  << "      accessKey: 'I'\n"
  << "    });\n"
  << "  });\n"
  << "  </script>\n"
  << "  <style type='text/css' media='all'>\n"
  << "  #my_splitter {\n"
  << "      height: 500px;\n"
  << "      width: 100%;\n"
  << "      border: 1px solid #aaa;\n"
  << "  }\n"
  << "  #left_pane {\n"
  << "    min-width:320px;\n"
  << "  }\n"
  << "  #right_pane {\n"
  << "    min-width:500px;\n"
  << "  }\n"
  << "  </style>\n"
  << "</head>\n<body>\n";

  std::cout << "<div style='padding:4px'>\n"
            << "<h1>SDF " << SDF::version << "</h1>\n";

  std::cout << "<p>The Robot Modeling Language (SDF) is an XML file "
            << "format used to describe all the elements in a simulation "
            << "environment.\n</p>";
  std::cout << "<h3>Usage</h3>\n";
  std::cout << "<blockquote>";
  std::cout << "<ul><li><b>Left Panel:</b> List of all the SDF elements.</li>";
  std::cout << "<li><b>Right Panel:</b> Descriptions of all the SDF "
            << "elements.</li>";
  std::cout << "<li><b>Selection:</b> Clicking an element in the Left Panel "
            << "moves the corresponding description to the top of the Right "
            << "Panel.</li>";
  std::cout << "<li><b>Search:</b> Use your web-browser's built in 'Find' "
            << "function to locate a specific element."
            << "</li></ul>";
  std::cout << "</blockquote>";

  std::cout << "</br>\n";

  std::cout << "<h3>Meta-Tags</h3>\n";
  std::cout << "<blockquote>";
  std::cout << "Meta-tags are processed by the parser before the final "
            << "SDF file is generated.";
  std::cout << "<ul>";

  std::cout << "<li><b>&ltinclude&gt</b>: Include an SDF model file "
            << "within the current SDF file."
            << "<ul style='margin-left:12px'>"
            << "<li><b>&lt;uri&gt;</b>: URI of SDF model file to include.</li>"
            << "<li><b>&lt;name&gt;</b>: Name of the included SDF model.</li>"
            << "<li><b>&lt;pose&gt;</b>: Pose of the included SDF model, "
            << "specified as &lt;pose&gt;x y z roll pitch yaw&lt;/pose&gt;, "
            << "with x, y, and z representing a position in meters, and roll, "
            << "pitch, and yaw representing Euler angles in radians.</li>"
            << "</ul>"
            << "</li>";

  std::cout << "</ul>";
  std::cout << "</blockquote>";


  std::cout << "</div>\n";

  std::cout << "<div id='my_splitter'>\n";

  std::cout << "<div id='left_pane'>\n";
  std::cout << html;
  std::cout << "</div>\n";

  std::cout << "<div id='right_pane'>\n";
  std::cout << html2;
  std::cout << "</div>\n";

  std::cout << "</div>\n";

  std::cout << "\
    </body>\
    </html>\n";
}

/////////////////////////////////////////////////
void SDF::Write(const std::string &_filename)
{
  std::string string = this->root->ToString("");

  std::ofstream out(_filename.c_str(), std::ios::out);

  if (!out)
  {
    sdferr << "Unable to open file[" << _filename << "] for writing\n";
    return;
  }
  out << string;
  out.close();
}

/////////////////////////////////////////////////
std::string SDF::ToString() const
{
  std::ostringstream stream;

  stream << "<?xml version='1.0'?>\n";
  if (this->root->GetName() != "sdf")
    stream << "<sdf version='" << SDF::version << "'>\n";

  stream << this->root->ToString("");

  if (this->root->GetName() != "sdf")
    stream << "</sdf>";

  return stream.str();
}

/////////////////////////////////////////////////
void SDF::SetFromString(const std::string &_sdfData)
{
  sdf::initFile("root.sdf", this->root);
  if (!sdf::readString(_sdfData, this->root))
  {
    sdferr << "Unable to parse sdf string[" << _sdfData << "]\n";
  }
}
