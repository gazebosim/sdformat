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

#ifndef _SDF_PARAM_HH_
#define _SDF_PARAM_HH_

// See: https://bugreports.qt-project.org/browse/QTBUG-22829
#ifndef Q_MOC_RUN
  #include <boost/lexical_cast.hpp>
  #include <boost/any.hpp>
  #include <boost/variant.hpp>
  #include <boost/version.hpp>
#endif

#include <memory>
#include <functional>
#include <algorithm>
#include <typeinfo>
#include <string>
#include <vector>
#include <ignition/math.hh>

#include "sdf/Console.hh"
#include "sdf/system_util.hh"

/// \todo Remove this diagnositic push/pop in version 5
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include "sdf/Types.hh"
#ifndef _WIN32
#pragma GCC diagnostic pop
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

    /// \brief Get the type of the value stored.
    /// \return The std::type_info.
    /// \deprecated GetType is unstable. Use IsType().
    /// \sa IsType
    public: const std::type_info &GetType() const SDF_DEPRECATED(4.0);

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
    /// The passed in value must conform to the boost::lexical_cast spec.
    /// This means the value must have an input and output stream operator.
    /// \param[in] _value The value to set the parameter to.
    /// \return True if the value was successfully set.
    public: template<typename T>
            bool Set(const T &_value);

    /// \brief Get the value of the parameter as a boost::any.
    /// \param[out] _anyVal The boost::any object to set.
    /// \return True if successfully set _anyVal, false otherwise.
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

    /// \brief Initialize the value. This is called from the constructor.
    /// \param[in] _value Value to set the parameter to.
    private: template<typename T>
             void Init(const std::string &_value);

    /// \brief Private data
    private: ParamPrivate *dataPtr;
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

/// \todo Remove this diagnositic push/pop in version 5
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    /// \def ParamVariant
    /// \briead Variant type def.
    public: typedef boost::variant<bool, char, std::string, int, uint64_t,
               unsigned int, double, float, sdf::Time, sdf::Color,
               sdf::Vector3, sdf::Vector2i, sdf::Vector2d,
               sdf::Quaternion, sdf::Pose,
               ignition::math::Vector3d, ignition::math::Vector2i,
               ignition::math::Vector2d, ignition::math::Quaterniond,
               ignition::math::Pose3d> ParamVariant;
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

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
      this->SetFromString(boost::lexical_cast<std::string>(_value));
    }
    catch(...)
    {
      sdferr << "Unable to set parameter["
             << this->dataPtr->key << "]."
             << "Type type used must have a stream input and output"
             << "operator, which allow boost::lexical_cast to"
             << "function properly.\n";
      return false;
    }
    return true;
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Param::Get(T &_value) const
  {
    try
    {
      if (typeid(T) == typeid(bool) &&
          this->dataPtr->typeName == "string")
      {
        std::string strValue =
          boost::lexical_cast<std::string>(this->dataPtr->value);
        if (strValue == "true" || strValue  == "1")
          _value = boost::lexical_cast<T>("1");
        else
          _value = boost::lexical_cast<T>("0");
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
        _value = boost::lexical_cast<T>(this->dataPtr->value);
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
    try
    {
      _value = boost::lexical_cast<T>(this->dataPtr->defaultValue);
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
  void Param::Init(const std::string &_value)
  {
    try
    {
      this->dataPtr->value = boost::lexical_cast<T>(_value);
    }
    catch(...)
    {
      if (this->dataPtr->typeName == "bool")
      {
        std::string strValue = _value;
        std::transform(strValue.begin(), strValue.end(),
                       strValue.begin(), ::tolower);
        if (strValue == "true" || strValue == "1")
          this->dataPtr->value = true;
        else
          this->dataPtr->value = false;
      }
      else
      {
        sdferr << "Unable to init parameter value from string["
               << _value << "]\n";
      }
    }

    this->dataPtr->defaultValue = this->dataPtr->value;
    this->dataPtr->set = false;
  }

  ///////////////////////////////////////////////
  template<typename Type>
  bool Param::IsType() const
  {
    return this->dataPtr->value.type() == typeid(Type);
  }
}
#endif
