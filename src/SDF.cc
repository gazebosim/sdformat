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
#include "sdf/Filesystem.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/SDFImplPrivate.hh"
#include "sdf/sdf_config.h"

using namespace sdf;

typedef std::list<std::string> PathList;
typedef std::map<std::string, PathList> URIPathMap;

static URIPathMap g_uriPathMap;

static std::function<std::string(const std::string &)> g_findFileCB;

std::string SDF::version = SDF_VERSION;

/////////////////////////////////////////////////
void sdf::setFindCallback(std::function<std::string(const std::string &)> _cb)
{
  g_findFileCB = _cb;
}

/////////////////////////////////////////////////
std::string sdf::findFile(const std::string &_filename, bool _searchLocalPath,
                          bool _useCallback)
{
  std::string path = _filename;

  // Check to see if _filename is URI. If so, resolve the URI path.
  for (URIPathMap::iterator iter = g_uriPathMap.begin();
       iter != g_uriPathMap.end(); ++iter)
  {
    // Check to see if the URI in the global map is the first part of the
    // given filename
    if (_filename.find(iter->first) == 0)
    {
      std::string suffix = _filename;
      size_t index = suffix.find(iter->first);
      if (index != std::string::npos)
      {
        suffix.replace(index, iter->first.length(), "");
      }

      // Check each path in the list.
      for (PathList::iterator pathIter = iter->second.begin();
           pathIter != iter->second.end(); ++pathIter)
      {
        // Return the path string if the path + suffix exists.
        std::string pathSuffix = sdf::filesystem::append(*pathIter, suffix);
        if (sdf::filesystem::exists(pathSuffix))
        {
          return pathSuffix;
        }
      }
    }
  }

  // Next check the install path.
  path = sdf::filesystem::append(SDF_SHARE_PATH, _filename);
  if (sdf::filesystem::exists(path))
  {
    return path;
  }

  // Next check the versioned install path.
  path = sdf::filesystem::append(SDF_SHARE_PATH, "sdformat",
                                 sdf::SDF::Version(), _filename);
  if (sdf::filesystem::exists(path))
  {
    return path;
  }

  // Next check SDF_PATH environment variable
  const char *pathCStr = std::getenv("SDF_PATH");

  if (pathCStr)
  {
    std::vector<std::string> paths = sdf::split(pathCStr, ":");
    for (std::vector<std::string>::iterator iter = paths.begin();
         iter != paths.end(); ++iter)
    {
      path = sdf::filesystem::append(*iter, _filename);
      if (sdf::filesystem::exists(path))
      {
        return path;
      }
    }
  }

  // Next check to see if the given file exists.
  path = _filename;
  if (sdf::filesystem::exists(path))
  {
    return path;
  }

  // Finally check the local path, if the flag is set.
  if (_searchLocalPath)
  {
    path = sdf::filesystem::append(sdf::filesystem::current_path(), _filename);
    if (sdf::filesystem::exists(path))
    {
      return path;
    }
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
    {
      return g_findFileCB(_filename);
    }
  }

  return std::string();
}

/////////////////////////////////////////////////
void sdf::addURIPath(const std::string &_uri, const std::string &_path)
{
  // Split _path on colons.
  std::vector<std::string> parts = sdf::split(_path, ":");

  // Add each part of the colon separated path to the global URI map.
  for (std::vector<std::string>::iterator iter = parts.begin();
       iter != parts.end(); ++iter)
  {
    // Only add valid paths
    if (!(*iter).empty() && sdf::filesystem::is_directory(*iter))
    {
      g_uriPathMap[_uri].push_back(*iter);
    }
  }
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
  this->Root()->PrintDescription("");
}

/////////////////////////////////////////////////
void SDF::PrintValues()
{
  this->Root()->PrintValues("");
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

  std::cout << "\
    </body>\
    </html>\n";
}

/////////////////////////////////////////////////
void SDF::Write(const std::string &_filename)
{
  std::string string = this->Root()->ToString("");

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
  if (this->Root()->GetName() != "sdf")
  {
    stream << "<sdf version='" << SDF::Version() << "'>\n";
  }

  stream << this->Root()->ToString("");

  if (this->Root()->GetName() != "sdf")
  {
    stream << "</sdf>";
  }

  return stream.str();
}

/////////////////////////////////////////////////
void SDF::SetFromString(const std::string &_sdfData)
{
  sdf::initFile("root.sdf", this->Root());
  if (!sdf::readString(_sdfData, this->Root()))
  {
    sdferr << "Unable to parse sdf string[" << _sdfData << "]\n";
  }
}

/////////////////////////////////////////////////
ElementPtr SDF::Root() const
{
  return this->dataPtr->root;
}

/////////////////////////////////////////////////
void SDF::Root(const ElementPtr _root)
{
  this->dataPtr->root = _root;
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
  ElementPtr root(new Element);
  root->SetName("sdf");
  std::stringstream v;
  v << Version();
  root->AddAttribute("version", "string", v.str(), true, "version");
  root->InsertElement(_sdf->Clone());
  return root;
}


