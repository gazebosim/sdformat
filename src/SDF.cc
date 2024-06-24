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

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <functional>
#include <list>
#include <map>
#include <string>
#include <vector>

#include "sdf/parser.hh"
#include "sdf/Assert.hh"
#include "sdf/Console.hh"
#include "sdf/Error.hh"
#include "sdf/Filesystem.hh"
#include "sdf/SDFImpl.hh"
#include "SDFImplPrivate.hh"
#include "sdf/sdf_config.h"
#include "EmbeddedSdf.hh"
#include "Utils.hh"

#include <gz/utils/Environment.hh>

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
// TODO(azeey) This violates the Google style guide. Change to a function that
// returns the version string when possible.
std::string SDF::version = SDF_VERSION;  // NOLINT(runtime/string)

std::string sdfSharePath()
{
#ifdef SDF_SHARE_PATH
  if (std::string(SDF_SHARE_PATH) != "/")
    return SDF_SHARE_PATH;
#endif
  return "";
}

/////////////////////////////////////////////////
void setFindCallback(std::function<std::string(const std::string &)> _cb)
{
  ParserConfig::GlobalConfig().SetFindCallback(_cb);
}

/////////////////////////////////////////////////
std::string findFile(
    const std::string &_filename, bool _searchLocalPath, bool _useCallback)
{
  sdf::Errors errors;
  std::string result = findFile(errors, _filename, _searchLocalPath,
                                _useCallback, ParserConfig::GlobalConfig());
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
std::string findFile(
    sdf::Errors &_errors, const std::string &_filename, bool _searchLocalPath,
    bool _useCallback)
{
  return findFile(
      _errors, _filename, _searchLocalPath,
      _useCallback, ParserConfig::GlobalConfig());
}

/////////////////////////////////////////////////
std::string findFile(const std::string &_filename, bool _searchLocalPath,
                          bool _useCallback, const ParserConfig &_config)
{
  sdf::Errors errors;
  std::string result = findFile(errors, _filename, _searchLocalPath,
                                _useCallback, _config);
  sdf::throwOrPrintErrors(errors);
  return result;
}


/////////////////////////////////////////////////
std::string findFile(sdf::Errors &_errors, const std::string &_filename,
                     bool _searchLocalPath, bool _useCallback,
                     const ParserConfig &_config)
{
  // Check to see if _filename is URI. If so, resolve the URI path.
  for (const auto &[uriScheme, paths] : _config.URIPathMap())
  {
    // Check to see if the URI in the global map is the first part of the
    // given filename
    if (_filename.find(uriScheme) == 0)
    {
      std::string suffix = _filename;
      size_t index = suffix.find(uriScheme);
      if (index != std::string::npos)
      {
        suffix.replace(index, uriScheme.length(), "");
      }

      // Check each path in the list.
      for (const auto &path : paths)
      {
        // Return the path string if the path + suffix exists.
        std::string pathSuffix = sdf::filesystem::append(path, suffix);
        if (sdf::filesystem::exists(pathSuffix))
        {
          return pathSuffix;
        }
      }
    }
  }

  // Strip scheme, if any
  std::string filename = _filename;
  std::string sep("://");
  size_t idx = _filename.find(sep);
  if (idx != std::string::npos)
  {
    filename = filename.substr(idx + sep.length());
  }

  // First check the local path, if the flag is set.
  if (_searchLocalPath)
  {
    std::string path = sdf::filesystem::append(sdf::filesystem::current_path(),
                                               filename);
    if (sdf::filesystem::exists(path))
    {
      return path;
    }
  }

  // Next check the install path.
  std::string path = sdf::filesystem::append(sdfSharePath(), filename);
  if (sdf::filesystem::exists(path))
  {
    return path;
  }

  // Next check the versioned install path.
  path = sdf::filesystem::append(sdfSharePath(),
    "sdformat" + std::string(SDF_MAJOR_VERSION_STR),
    sdf::SDF::Version(), filename);

  if (sdf::filesystem::exists(path))
  {
    return path;
  }

  // Finally check to see if the given file exists.
  path = filename;
  if (sdf::filesystem::exists(path))
  {
    return path;
  }

  std::string sdfPathEnv;
  if(gz::utils::env("SDF_PATH", sdfPathEnv))
  {
    std::vector<std::string> paths = sdf::split(sdfPathEnv, ":");
    for (std::vector<std::string>::iterator iter = paths.begin();
         iter != paths.end(); ++iter)
    {
      path = sdf::filesystem::append(*iter, filename);
      if (sdf::filesystem::exists(path))
      {
        return path;
      }
    }
  }

  // If we still haven't found the file, use the registered callback if the
  // flag has been set
  if (_useCallback)
  {
    if (!_config.FindFileCallback())
    {
      _errors.push_back({sdf::ErrorCode::FILE_NOT_FOUND,
        "Tried to use callback in sdf::findFile(), but the callback "
        "is empty.  Did you call sdf::setFindCallback()?"});
      return std::string();
    }
    else
    {
      return _config.FindFileCallback()(_filename);
    }
  }

  return std::string();
}

/////////////////////////////////////////////////
void addURIPath(const std::string &_uri, const std::string &_path)
{
  ParserConfig::GlobalConfig().AddURIPath(_uri, _path);
}

/////////////////////////////////////////////////
SDF::SDF()
  : dataPtr(new SDFPrivate)
{
}

/////////////////////////////////////////////////
SDF::~SDF()
{
}

/////////////////////////////////////////////////
void SDF::PrintDescription()
{
  sdf::Errors errors;
  this->PrintDescription(errors);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void SDF::PrintDescription(sdf::Errors &_errors)
{
  this->Root()->PrintDescription(_errors, "");
}

/////////////////////////////////////////////////
void SDF::PrintValues(const PrintConfig &_config)
{
  sdf::Errors errors;
  this->PrintValues(errors, _config);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void SDF::PrintValues(sdf::Errors &_errors, const PrintConfig &_config)
{
  this->Root()->PrintValues(_errors, "", _config);
}

/////////////////////////////////////////////////
void SDF::PrintDoc()
{
  std::string html, html2;
  int index = 0;
  this->Root()->PrintDocLeftPane(html, 10, index);

  index = 0;
  this->Root()->PrintDocRightPane(html2, 10, index);

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
            << "<h1>SDF " << SDF::Version() << "</h1>\n";

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

  std::cout << "    </body>    </html>\n";
}

/////////////////////////////////////////////////
void SDF::Write(const std::string &_filename)
{
  sdf::Errors errors;
  this->Write(errors, _filename);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void SDF::Write(sdf::Errors &_errors, const std::string &_filename)
{
  std::string string = this->Root()->ToString(_errors, "");

  std::ofstream out(_filename.c_str(), std::ios::out);

  if (!out)
  {
    _errors.push_back({sdf::ErrorCode::FILE_NOT_FOUND,
      "Unable to open file[" + _filename + "] for writing."});
    return;
  }
  out << string;
  out.close();
}

/////////////////////////////////////////////////
std::string SDF::ToString(const PrintConfig &_config) const
{
  sdf::Errors errors;
  std::string result = this->ToString(errors, _config);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
std::string SDF::ToString(sdf::Errors &_errors,
                          const PrintConfig &_config) const
{
  std::ostringstream stream;

  stream << "<?xml version='1.0'?>\n";
  if (this->Root()->GetName() != "sdf")
  {
    stream << "<sdf version='" << SDF::Version() << "'>\n";
  }

  stream << this->Root()->ToString(_errors, "", _config);

  if (this->Root()->GetName() != "sdf")
  {
    stream << "</sdf>";
  }

  return stream.str();
}

/////////////////////////////////////////////////
void SDF::SetFromString(const std::string &_sdfData)
{
  sdf::Errors errors;
  this->SetFromString(errors, _sdfData);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void SDF::SetFromString(sdf::Errors &_errors,
                        const std::string &_sdfData)
{
  sdf::initFile("root.sdf", this->Root());
  if (!sdf::readString(_sdfData, this->Root(), _errors))
  {
    _errors.push_back({sdf::ErrorCode::PARSING_ERROR,
      "Unable to parse sdf string[" + _sdfData + "]"});
  }
}

/////////////////////////////////////////////////
void SDF::Clear()
{
  this->dataPtr->root->Clear();
  this->dataPtr->path.clear();
  this->dataPtr->originalVersion.clear();
}

/////////////////////////////////////////////////
ElementPtr SDF::Root() const
{
  return this->dataPtr->root;
}

/////////////////////////////////////////////////
void SDF::SetRoot(const ElementPtr _root)
{
  this->dataPtr->root = _root;
}

/////////////////////////////////////////////////
void SDF::Root(const ElementPtr _root)
{
  this->SetRoot(_root);
}

/////////////////////////////////////////////////
std::string SDF::FilePath() const
{
  return this->dataPtr->path;
}

/////////////////////////////////////////////////
void SDF::SetFilePath(const std::string &_path)
{
  this->dataPtr->path = _path;
  this->dataPtr->root->SetFilePath(_path);
}

/////////////////////////////////////////////////
void SDF::SetOriginalVersion(const std::string &_version)
{
  this->dataPtr->originalVersion = _version;
  this->dataPtr->root->SetOriginalVersion(_version);
}

/////////////////////////////////////////////////
const std::string &SDF::OriginalVersion() const
{
  return this->dataPtr->originalVersion;
}

/////////////////////////////////////////////////
std::string SDF::Version()
{
  return version;
}

/////////////////////////////////////////////////
void SDF::Version(const std::string &_version)
{
  version = _version;
}

/////////////////////////////////////////////////
ElementPtr SDF::WrapInRoot(const ElementPtr &_sdf)
{
  sdf::Errors errors;
  ElementPtr result = SDF::WrapInRoot(_sdf, errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
ElementPtr SDF::WrapInRoot(const ElementPtr &_sdf, sdf::Errors &_errors)
{
  ElementPtr root(new Element);
  root->SetName("sdf");
  std::stringstream v;
  v << Version();
  root->AddAttribute("version", "string", v.str(), true, _errors, "version");
  root->InsertElement(_sdf->Clone());
  return root;
}

/////////////////////////////////////////////////
const std::string &SDF::EmbeddedSpec(
    const std::string &_filename, const bool _quiet)
{
  sdf::Errors errors;
  const std::string &result = EmbeddedSpec(_filename, errors);
  if (!_quiet)
  {
    sdf::throwOrPrintErrors(errors);
  }
  return result;
}

/////////////////////////////////////////////////
const std::string &SDF::EmbeddedSpec(
    const std::string &_filename, sdf::Errors &_errors)
{
  try
  {
    const std::string pathname = SDF::Version() + "/" + _filename;
    return GetEmbeddedSdf().at(pathname);
  }
  catch(const std::out_of_range &)
  {
      _errors.push_back({sdf::ErrorCode::FILE_NOT_FOUND,
          "Unable to find SDF filename[" + _filename + "] with " +
          "version " + SDF::Version()});
  }

  // An empty SDF string is returned if a query into the embeddedSdf map fails.
  static const std::string emptySdfString;
  return emptySdfString;
}
}
}
