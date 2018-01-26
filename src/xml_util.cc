#include "sdf/xml_util.hh"

#include <algorithm>

#include "sdf/Console.hh"

tinyxml2::XMLNode* DeepClone(tinyxml2::XMLDocument* _doc, const tinyxml2::XMLNode* _src)
{
  if (_src == NULL)
  {
    sdferr << "Pointer to XML node _src is NULL\n";
    return NULL;
  }

  tinyxml2::XMLNode *copy = _src->ShallowClone(_doc);
  if (copy == NULL)
  {
    sdferr << "Failed to clone node " << _src->Value() << "\n";
    return NULL;
  }

  for (const tinyxml2::XMLNode *node = _src->FirstChild(); node != NULL;
       node = node->NextSibling())
  {
    auto* child_copy = DeepClone(_doc, node);
    if(child_copy == NULL)
    {
      sdferr << "Failed to clone child " << node->Value() << "\n";
      return NULL;
    }
    copy->InsertEndChild(child_copy);
  }

  return copy;
}

namespace {

std::string &TrimStringLeft(std::string &s)
{
  s.erase(s.begin(),find_if_not(s.begin(),s.end(),[](int c){return isspace(c);}));
  return s;
}

std::string &TrimStringRight(std::string &s)
{
  s.erase(find_if_not(s.rbegin(),s.rend(),[](int c){return isspace(c);}).base(), s.end());
  return s;
}

}

std::string TrimString(const std::string &s)
{
  std::string t=s;
  return TrimStringLeft(TrimStringRight(t));
}
