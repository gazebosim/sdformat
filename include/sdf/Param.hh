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

#ifndef SDFORMAT_PARAM_HH_
#define SDFORMAT_PARAM_HH_

// See: https://bugreports.qt-project.org/browse/QTBUG-22829
#ifndef Q_MOC_RUN
  #include <boost/any.hpp>
  #include <boost/variant.hpp>
#endif

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include <ignition/math.hh>

#include "sdf/Console.hh"
#include "sdf/system_util.hh"
#include "sdf/Types.hh"

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

namespace sdf
{
  class SDFORMAT_VISIBLE Param;

  /// \def ParamPtr
  /// \brief Shared pointer to a Param
  typedef std::shared_ptr<Param> ParamPtr;

  /// \def Param_V
  /// \brief vector of shared pointers to a Param
  typedef std::vector<ParamPtr> Param_V;

  /// \internal
  class ParamPrivate;

  /// \class Param Param.hh sdf/sdf.hh
  /// \brief A parameter class
  class SDFORMAT_VISIBLE Param
  {
    /// \brief Constructor.
    /// \param[in] _key Key for the parameter.
    /// \param[in] _typeName String name for the value type (double,
    /// int,...).
    /// \param[in] _default Default value.
    /// \param[in] _required True if the parameter is required to be set.
    /// \param[in] _description Description of the parameter.
    /// \throws sdf::AssertionInternalError if an invalid type is given.
    public: Param(const std::string &_key, const std::string &_typeName,
                  const std::string &_default, bool _required,
                  const std::string &_description = "");

    /// \brief Destructor
    public: virtual ~Param();

    /// \brief Get the value as a string.
    /// \return String containing the value of the parameter.
    public: std::string GetAsString() const;

    /// \brief Get the default value as a string.
    /// \return String containing the default value of the parameter.
    public: std::string GetDefaultAsString() const;

    /// \brief Set the parameter value from a string.
    /// \param[in] _value New value for the parameter in string form.
    public: bool SetFromString(const std::string &_value);

    /// \brief Reset the parameter to the default value.
    public: void Reset();

    /// \brief Get the key value.
    /// \return The key.
    public: const std::string &GetKey() const;

    /// \brief Return true if the param is a particular type
    /// \return True if the type held by this Param matches the Type
    /// template parameter.
    public: template<typename Type>
            bool IsType() const;

    /// \brief Get the type name value.
    /// \return The type name.
    public: const std::string &GetTypeName() const;

    /// \brief Return whether the parameter is required.
    /// \return True if the parameter is required.
    public: bool GetRequired() const;

    /// \brief Return true if the parameter has been set.
    /// \return True if the parameter has been set.
    public: bool GetSet() const;

    /// \brief Clone the parameter.
    /// \return A new parameter that is the clone of this.
    public: ParamPtr Clone() const;

    /// \brief Set the update function. The updateFunc will be used to
    /// set the parameter's value when Param::Update is called.
    /// \param[in] _updateFunc Function pointer to an update function.
    public: template<typename T>
            void SetUpdateFunc(T _updateFunc);

    /// \brief Set the parameter's value using the updateFunc.
    /// \sa Param::SetUpdateFunc
    public: void Update();

    /// \brief Set the parameter's value.
    ///
    /// The passed in value value must have an input and output stream operator.
    /// \param[in] _value The value to set the parameter to.
    /// \return True if the value was successfully set.
    public: template<typename T>
            bool Set(const T &_value);

    /// \brief Get the value of the parameter as a boost::any.
    /// \param[out] _anyVal The boost::any object to set.
    /// \return True if successfully fetched _anyVal, false otherwise.
    public: bool GetAny(boost::any &_anyVal) const;

    /// \brief Get the value of the parameter.
    /// \param[out] _value The value of the parameter.
    /// \return True if parameter was successfully cast to the value type
    /// passed in.
    public: template<typename T>
            bool Get(T &_value) const;

    /// \brief Get the default value of the parameter.
    /// \param[out] _value The default value of the parameter.
    /// \return True if parameter was successfully cast to the value type
    /// passed in.
    public: template<typename T>
            bool GetDefault(T &_value) const;

    /// \brief Equal operator. Set's the value and default value from the
    /// provided Param.
    /// \param[in] _param The parameter to set values from.
    /// \return *This
    public: Param &operator=(const Param &_param);

    /// \brief Set the description of the parameter.
    /// \param[in] _desc New description for the parameter.
    public: void SetDescription(const std::string &_desc);

    /// \brief Get the description of the parameter.
    /// \return The description of the parameter.
    public: std::string GetDescription() const;

    /// \brief Ostream operator. Outputs the parameter's value.
    /// \param[in] _out Output stream.
    /// \param[in] _p The parameter to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Param &_p)
    {
      _out << _p.GetAsString();
      return _out;
    }

    /// \brief Private method to set the Element from a passed-in string.
    /// \param[in] _value Value to set the parameter to.
    private: bool ValueFromString(const std::string &_value);

    /// \brief Private data
    private: std::unique_ptr<ParamPrivate> dataPtr;
  };

  /// \internal
  /// \brief Private data for the param class
  class ParamPrivate
  {
    /// \brief Key value
    public: std::string key;

    /// \brief True if the parameter is required.
    public: bool required;

    /// \brief True if the parameter is set.
    public: bool set;

    //// \brief Name of the type.
    public: std::string typeName;

    /// \brief Description of the parameter.
    public: std::string description;

    /// \brief Update function pointer.
    public: std::function<boost::any ()> updateFunc;

    /// \def ParamVariant
    /// \brief Variant type def.
    public: typedef boost::variant<bool, char, std::string, int, std::uint64_t,
                                   unsigned int, double, float, sdf::Time,
                                   ignition::math::Color,
                                   ignition::math::Vector2i,
                                   ignition::math::Vector2d,
                                   ignition::math::Vector3d,
                                   ignition::math::Quaterniond,
                                   ignition::math::Pose3d> ParamVariant;

    /// \brief This parameter's value
    public: ParamVariant value;

    /// \brief This parameter's default value
    public: ParamVariant defaultValue;
  };

  ///////////////////////////////////////////////
  template<typename T>
  void Param::SetUpdateFunc(T _updateFunc)
  {
    this->dataPtr->updateFunc = _updateFunc;
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Param::Set(const T &_value)
  {
    try
    {
      std::stringstream ss;
      ss << _value;
      return this->SetFromString(ss.str());
    }
    catch(...)
    {
      sdferr << "Unable to set parameter["
             << this->dataPtr->key << "]."
             << "Type used must have a stream input and output operator,"
             << "which allows proper functioning of Param.\n";
      return false;
    }
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Param::Get(T &_value) const
  {
    try
    {
      if (typeid(T) == typeid(bool) && this->dataPtr->typeName == "string")
      {
        std::stringstream ss;
        ss << this->dataPtr->value;

        std::string strValue;

        ss >> strValue;
        std::transform(strValue.begin(), strValue.end(), strValue.begin(),
            [](unsigned char c)
            {
              return static_cast<unsigned char>(std::tolower(c));
            });

        std::stringstream tmp;
        if (strValue == "true" || strValue  == "1")
        {
          tmp << "1";
        }
        else
        {
          tmp << "0";
        }
        tmp >> _value;
      }
      else if (typeid(T) == this->dataPtr->value.type())
      {
#if BOOST_VERSION < 105800
        _value = boost::get<T>(this->dataPtr->value);
#else
        _value = boost::relaxed_get<T>(this->dataPtr->value);
#endif
      }
      else
      {
        std::stringstream ss;
        ss << this->dataPtr->value;
        ss >> _value;
      }
    }
    catch(...)
    {
      sdferr << "Unable to convert parameter["
             << this->dataPtr->key << "] "
             << "whose type is["
             << this->dataPtr->typeName << "], to "
             << "type[" << typeid(T).name() << "]\n";
      return false;
    }
    return true;
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Param::GetDefault(T &_value) const
  {
    std::stringstream ss;

    try
    {
      ss << this->dataPtr->defaultValue;
      ss >> _value;
    }
    catch(...)
    {
      sdferr << "Unable to convert parameter["
             << this->dataPtr->key << "] "
             << "whose type is["
             << this->dataPtr->typeName << "], to "
             << "type[" << typeid(T).name() << "]\n";
      return false;
    }

    return true;
  }

  ///////////////////////////////////////////////
  template<typename Type>
  bool Param::IsType() const
  {
    return this->dataPtr->value.type() == typeid(Type);
  }
}

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
