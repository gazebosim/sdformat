/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef TOML_PARSER_HH_
#define TOML_PARSER_HH_

#include <map>
#include <string>
#include <variant>

#include "sdf/Param.hh"
// This is a very basic toml parser for use in the interface_api integration
// test and is not capable of parsing the full toml syntax. Specifically, this
// parser does not support arrays and inline tables. It also only knows of only
// a small set of value types.
namespace toml
{
std::string appendPrefix(const std::string &_input)
{
  return _input;
}

template <typename T, typename... Args>
std::string appendPrefix(
    const std::string &_prefix, const T &_key, Args... others)
{
  if (_prefix.empty())
    return appendPrefix(_key, others...);
  return appendPrefix(_prefix + '.' + _key, others...);
}

std::string keyType(const std::string &_key)
{
  if (_key == "pose")
    return "pose";
  else if (_key == "static")
    return "bool";

  return "string";
}

// A struct is needed to create a recursive std::variant based data structure
struct Value
{
  using KeyValue = std::map<std::string, Value>;
  using VariantType = std::variant<KeyValue, sdf::Param>;
  VariantType data;

  public: KeyValue &Map()
  {
    return std::get<KeyValue>(this->data);
  }

  public: template <typename T>
  T ParamGet(const T &_defaultVal = T {})
  {
    T out{_defaultVal};
    if (auto param = std::get_if<sdf::Param>(&this->data))
    {
      param->Get(out);
    }
    return out;
  }

  public: Value &operator[](const std::string &_key)
  {
    auto keys = sdf::split(_key, ".");
    Value *curValue = &(this->Map()[keys[0]]);

    for (std::size_t i = 1; i < keys.size(); ++i)
    {
      if (auto kval = std::get_if<KeyValue>(&curValue->data))
      {
        curValue = &kval->operator[](keys[i]);
      }
      else
      {
        throw std::runtime_error("Unable to find key [" + _key + "]");
      }
    }
    return *curValue;
  }

  public: void PrintValue(std::ostream &os, const std::string &_key) const
  {
    std::visit(
        [&](auto &&arg)
        {
          using T = std::decay_t<decltype(arg)>;
          // cppcheck-suppress syntaxError
          if constexpr (std::is_same_v<T, sdf::Param>)
          {
            os << _key << " = " << arg << std::endl;
          }
          else if constexpr (std::is_same_v<T, Value::KeyValue>)
          {
            for (const auto &[key, val] : arg)
            {
              val.PrintValue(os, appendPrefix(_key, key));
            }
          }
        },
        data);
  }

  friend std::ostream& operator<<(std::ostream& os, const Value &_value)
  {
    _value.PrintValue(os, "");
    return os;
  }
};

struct Document: public Value
{
  Document()
      : Value({Value::KeyValue()})
  {
  }
};

Document parseToml(const std::string &_filePath, sdf::Errors &_errors)
{
  std::fstream fs;
  fs.open(_filePath);
  if (!fs.good())
  {
    _errors.emplace_back(sdf::ErrorCode::FILE_READ,
        "File [" + _filePath + "] could not be read.");
    return {};
  }

  Document doc;

  auto readValue = [](const std::string &_inp)
  {
    const std::string trimmed = sdf::trim(_inp);
    // Find the quotes
    auto begInd = trimmed.find('"');
    auto endInd = trimmed.rfind('"');
    return trimmed.substr(begInd + 1, endInd - 1);
  };
  auto readTableName = [](const std::string &_inp)
  {
    const std::string trimmed = sdf::trim(_inp);
    // Find the quotes
    auto begInd = trimmed.find('[');
    auto endInd = trimmed.rfind(']');
    return trimmed.substr(begInd + 1, endInd - 1);
  };

  std::string curPrefix = "";
  std::string line;
  while (std::getline(fs, line))
  {
    sdf::trim(line);
    if (line.empty() || line[0] == '#' )
    {
      continue;
    }
    else
    {
      std::size_t eqInd = line.find('=');
      if (eqInd != std::string::npos)
      {
        const std::string key = sdf::trim(line.substr(0, eqInd));
        const std::string value = readValue(line.substr(eqInd + 1));

        sdf::Param param(key, keyType(key), "", true);
        param.SetFromString(value);

        doc[appendPrefix(curPrefix, key)] = {param};
      }
      else if (line.find('[') != std::string::npos)
      {
        const std::string tableName = readTableName(line);
        if (tableName.empty())
        {
          throw std::runtime_error("Empty table name encountered");
        }
        else
        {
          curPrefix = tableName;
        }
      }
    }
  }

  fs.close();
  return doc;
}
}  // namespace toml

#endif
