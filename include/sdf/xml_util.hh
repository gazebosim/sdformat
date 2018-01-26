#ifndef _SDFORMAT_XML_UTIL_HH_
#define _SDFORMAT_XML_UTIL_HH_

#include <string>

#include <tinyxml2.h>

// Perform a deep copy of _src. The return node will be created in _doc.
tinyxml2::XMLNode* DeepClone(tinyxml2::XMLDocument* _doc, const tinyxml2::XMLNode* _src);

// Trim white space at beginning and end of a string
std::string TrimString(const std::string &s);

#endif
