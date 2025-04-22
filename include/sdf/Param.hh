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

#include <any>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <typeinfo>
#include <variant>
#include <vector>

#include <gz/math/Angle.hh>
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

#include "sdf/Console.hh"
#include "sdf/PrintConfig.hh"
#include "sdf/config.hh"
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
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  class SDFORMAT_VISIBLE Element;
  using ElementPtr = std::shared_ptr<Element>;
  using ElementWeakPtr = std::weak_ptr<Element>;

  class SDFORMAT_VISIBLE Param;

  /// \def ParamPtr
  /// \brief Shared pointer to a Param
  typedef std::shared_ptr<Param> ParamPtr;

  /// \def Param_V
  /// \brief vector of shared pointers to a Param
  typedef std::vector<ParamPtr> Param_V;

  /// \internal
  class ParamPrivate;

  template<class T>
  struct ParamStreamer
  {
    const T &val;
    const int precision;  // Used to set std::ostream's std::setprecision
    explicit ParamStreamer(const T &_val, int _precision = 0)
       : val(_val), precision(_precision) {}
  };

  // Template deduction guide for ParamVariant
  template<typename ParamVariant>
  ParamStreamer(const ParamVariant &_val, int _precision)
    -> ParamStreamer<ParamVariant>;

  template<class T>
  std::ostream& operator<<(std::ostream &os, ParamStreamer<T> s)
  {
    if (s.precision == std::numeric_limits<int>::max())
    {
      if constexpr (std::is_same_v<T, double>
                    || std::is_same_v<T, gz::math::Angle>
                    || std::is_same_v<T, gz::math::Vector2d>
                    || std::is_same_v<T, gz::math::Vector3d>
                    || std::is_same_v<T, gz::math::Quaterniond>
                    || std::is_same_v<T, gz::math::Pose3d>)
      {
        os << std::setprecision(std::numeric_limits<double>::max_digits10);
      }
      else if constexpr (std::is_same_v<T, float>
                         || std::is_same_v<T, gz::math::Color>)
      {
        os << std::setprecision(std::numeric_limits<float>::max_digits10);
      }
    }
    else
    {
      os << std::setprecision(s.precision);
    }

    os << s.val;
    return os;
  }

  template<class... Ts>
  std::ostream& operator<<(std::ostream& os,
                           ParamStreamer<std::variant<Ts...>> sv)
  {
    std::visit([&os, &sv](auto const &v)
      {
        os << ParamStreamer{v, sv.precision};
      }, sv.val);
    return os;
  }

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

    /// \brief Constructor.
    /// \param[in] _key Key for the parameter.
    /// \param[in] _typeName String name for the value type (double,
    /// int,...).
    /// \param[in] _default Default value.
    /// \param[in] _required True if the parameter is required to be set.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _description Description of the parameter.
    public: Param(const std::string &_key, const std::string &_typeName,
                  const std::string &_default, bool _required,
                  sdf::Errors &_errors,
                  const std::string &_description = "");

    /// \brief Constructor with min and max values.
    /// \param[in] _key Key for the parameter.
    /// \param[in] _typeName String name for the value type (double,
    /// int,...).
    /// \param[in] _default Default value.
    /// \param[in] _required True if the parameter is required to be set.
    /// \param[in] _minValue Minimum allowed value for the parameter.
    /// \param[in] _maxValue Maximum allowed value for the parameter.
    /// \param[in] _description Description of the parameter.
    /// \throws sdf::AssertionInternalError if an invalid type is given.
    public: Param(const std::string &_key, const std::string &_typeName,
                  const std::string &_default, bool _required,
                  const std::string &_minValue, const std::string &_maxValue,
                  const std::string &_description = "");

    /// \brief Constructor with min and max values.
    /// \param[in] _key Key for the parameter.
    /// \param[in] _typeName String name for the value type (double,
    /// int,...).
    /// \param[in] _default Default value.
    /// \param[in] _required True if the parameter is required to be set.
    /// \param[in] _minValue Minimum allowed value for the parameter.
    /// \param[in] _maxValue Maximum allowed value for the parameter.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _description Description of the parameter.
    public: Param(const std::string &_key, const std::string &_typeName,
                  const std::string &_default, bool _required,
                  const std::string &_minValue, const std::string &_maxValue,
                  sdf::Errors &_errors,
                  const std::string &_description = "");

    /// \brief Copy constructor
    /// Note that the updateFunc member does not get copied
    /// \param[in] _param Param to copy
    public: Param(const Param &_param);

    /// \brief Move constructor
    /// \param[in] _param Param to move from
    public: Param(Param &&_param) noexcept = default;

    /// \brief Copy assignment operator
    /// Note that the updateFunc member will not get copied
    /// \param[in] _param The parameter to set values from.
    /// \return *This
    public: Param &operator=(const Param &_param);

    /// \brief Move assignment operator
    /// \param[in] _param Param to move from
    /// \returns Reference to this
    public: Param &operator=(Param &&_param) noexcept = default;

    /// \brief Destructor
    public: virtual ~Param();

    /// \brief Get the value as a string.
    /// \param[in] _config Configuration for conversion to string.
    /// \return String containing the value of the parameter.
    public: std::string GetAsString(
        const PrintConfig &_config = PrintConfig()) const;

    /// \brief Get the value as a string.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _config Configuration for conversion to string.
    /// \return String containing the value of the parameter.
    public: std::string GetAsString(
        sdf::Errors &_errors,
        const PrintConfig &_config = PrintConfig()) const;

    /// \brief Get the default value as a string.
    /// \param[in] _config Configuration for conversion to string.
    /// \return String containing the default value of the parameter.
    public: std::string GetDefaultAsString(
        const PrintConfig &_config = PrintConfig()) const;

    /// \brief Get the default value as a string.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _config Configuration for conversion to string.
    /// \return String containing the default value of the parameter.
    public: std::string GetDefaultAsString(
        sdf::Errors &_errors,
        const PrintConfig &_config = PrintConfig()) const;

    /// \brief Get the minimum allowed value as a string.
    /// \param[in] _config Configuration for conversion to string.
    /// \return Returns a string containing the minimum allowed value of the
    /// parameter if the minimum value is specified in the SDFormat description
    /// of the parameter. nullopt otherwise.
    public: std::optional<std::string> GetMinValueAsString(
        const PrintConfig &_config = PrintConfig()) const;

    /// \brief Get the minimum allowed value as a string.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _config Configuration for conversion to string.
    /// \return Returns a string containing the minimum allowed value of the
    /// parameter if the minimum value is specified in the SDFormat description
    /// of the parameter. nullopt otherwise.
    public: std::optional<std::string> GetMinValueAsString(
        sdf::Errors &_errors,
        const PrintConfig &_config = PrintConfig()) const;

    /// \brief Get the maximum allowed value as a string.
    /// \param[in] _config Configuration for conversion to string.
    /// \return Returns a string containing the maximum allowed value of the
    /// parameter if the maximum value is specified in the SDFormat description
    /// of the parameter. nullopt otherwise.
    public: std::optional<std::string> GetMaxValueAsString(
        const PrintConfig &_config = PrintConfig()) const;

    /// \brief Get the maximum allowed value as a string.
    /// \param[in] _config Configuration for conversion to string.
    /// \param[out] _errors Vector of errors.
    /// \return Returns a string containing the maximum allowed value of the
    /// parameter if the maximum value is specified in the SDFormat description
    /// of the parameter. nullopt otherwise.
    public: std::optional<std::string> GetMaxValueAsString(
        sdf::Errors &_errors,
        const PrintConfig &_config = PrintConfig()) const;

    /// \brief Set the parameter value from a string.
    /// \param[in] _value New value for the parameter in string form.
    /// \param[in] _ignoreParentAttributes Whether to ignore parent element
    /// attributes when parsing value from string as well as subsequent
    /// reparses.
    public: bool SetFromString(const std::string &_value,
                               bool _ignoreParentAttributes);

    /// \brief Set the parameter value from a string.
    /// \param[in] _value New value for the parameter in string form.
    /// \param[in] _ignoreParentAttributes Whether to ignore parent element
    /// \param[out] _errors Vector of errors.
    /// attributes when parsing value from string as well as subsequent
    /// reparses.
    public: bool SetFromString(const std::string &_value,
                               bool _ignoreParentAttributes,
                               sdf::Errors &_errors);

    /// \brief Set the parameter value from a string.
    /// \param[in] _value New value for the parameter in string form.
    public: bool SetFromString(const std::string &_value);

    /// \brief Set the parameter value from a string.
    /// \param[in] _value New value for the parameter in string form.
    /// \param[out] _errors Vector of errors.
    public: bool SetFromString(const std::string &_value,
                               sdf::Errors &_errors);

    /// \brief Get the parent Element of this Param.
    /// \return Pointer to this Param's parent Element, nullptr if there is no
    /// parent Element.
    public: ElementPtr GetParentElement() const;

    /// \brief Set the parent Element of this Param.
    /// \param[in] _parentElement Pointer to new parent Element. A nullptr can
    ///  be provided to remove the current parent Element.
    /// \return True if the parent Element was set and the value was reparsed
    /// successfully.
    public: bool SetParentElement(ElementPtr _parentElement);

    /// \brief Set the parent Element of this Param.
    /// \param[in] _parentElement Pointer to new parent Element. A nullptr can
    /// be provided to remove the current parent Element.
    /// \param[out] _errors Vector of errors.
    /// \return True if the parent Element was set and the value was reparsed
    /// successfully.
    public: bool SetParentElement(ElementPtr _parentElement,
                                  sdf::Errors &_errors);

    /// \brief Set the parent Element of this Param without reparsing.
    /// This is meant for internal consumption when cloning elements.
    /// \param[in] _parentElement Pointer to new parent Element. A nullptr can
    /// be provided to remove the current parent Element.
    /// \return True if the parent Element was set.
    public: bool SetParentElementNoReparse(
        ElementPtr _parentElement);

    /// \brief Reset the parameter to the default value.
    public: void Reset();

    /// \brief Reparse the parameter value. This should be called after the
    /// parent element's attributes have been modified, in the event that the
    /// value was set using SetFromString or possesses a default value, and that
    /// the final parsed value is dependent on the attributes of the parent
    /// element. For example, the rotation component of a pose element can
    /// be parsed as degrees or radians, depending on the attribute @degrees
    /// of the parent element. If however the value was explicitly set using the
    /// Set<T> function, reparsing would not change the value.
    /// \return True if the parameter value has been reparsed successfully.
    /// \sa bool SetFromString(const std::string &_value)
    /// \sa bool Set(const T &_value)
    public: bool Reparse();

    /// \brief Reparse the parameter value. This should be called after the
    /// parent element's attributes have been modified, in the event that the
    /// value was set using SetFromString or possesses a default value, and that
    /// the final parsed value is dependent on the attributes of the parent
    /// element. For example, the rotation component of a pose element can
    /// be parsed as degrees or radians, depending on the attribute @degrees
    /// of the parent element. If however the value was explicitly set using the
    /// Set<T> function, reparsing would not change the value.
    /// \param[out] _errors Vector of errors.
    /// \return True if the parameter value has been reparsed successfully.
    /// \sa bool SetFromString(const std::string &_value)
    /// \sa bool Set(const T &_value)
    public: bool Reparse(sdf::Errors &_errors);

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

    /// \brief Return true if the parameter ignores the parent element's
    /// attributes, or if the parameter has no parent element.
    /// \return True if the parameter ignores the parent element's attributes,
    /// or if the parameter has no parent element.
    public: bool IgnoresParentElementAttribute() const;

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

    /// \brief Set the parameter's value using the updateFunc.
    /// \param[out] _errors Vector of errors.
    /// \sa Param::SetUpdateFunc
    public: void Update(sdf::Errors &_errors);

    /// \brief Set the parameter's value.
    ///
    /// The passed in value value must have an input and output stream operator.
    /// \param[in] _value The value to set the parameter to.
    /// \return True if the value was successfully set.
    public: template<typename T>
            bool Set(const T &_value);

    /// \brief Set the parameter's value.
    ///
    /// The passed in value value must have an input and output stream operator.
    /// \param[in] _value The value to set the parameter to.
    /// \param[out] _errors Vector of errors.
    /// \return True if the value was successfully set.
    public: template<typename T>
            bool Set(const T &_value,
                     sdf::Errors &_errors);

    /// \brief Get the value of the parameter as a std::any.
    /// \param[out] _anyVal The std::any object to set.
    /// \return True if successfully fetched _anyVal, false otherwise.
    public: bool GetAny(std::any &_anyVal) const;

    /// \brief Get the value of the parameter as a std::any.
    /// \param[out] _anyVal The std::any object to set.
    /// \param[out] _errors Vector of errors.
    /// \return True if successfully fetched _anyVal, false otherwise.
    public: bool GetAny(std::any &_anyVal, sdf::Errors &_errors) const;

    /// \brief Get the value of the parameter.
    /// \param[out] _value The value of the parameter.
    /// \return True if parameter was successfully cast to the value type
    /// passed in.
    public: template<typename T>
            bool Get(T &_value) const;

    /// \brief Get the value of the parameter.
    /// \param[out] _value The value of the parameter.
    /// \param[out] _errors Vector of errors.
    /// \return True if parameter was successfully cast to the value type
    /// passed in.
    public: template<typename T>
            bool Get(T &_value,
                     sdf::Errors &_errors) const;

    /// \brief Get the default value of the parameter.
    /// \param[out] _value The default value of the parameter.
    /// \return True if parameter was successfully cast to the value type
    /// passed in.
    public: template<typename T>
            bool GetDefault(T &_value) const;

    /// \brief Get the default value of the parameter.
    /// \param[out] _value The default value of the parameter.
    /// \param[out] _errors Vector of errors.
    /// \return True if parameter was successfully cast to the value type
    /// passed in.
    public: template<typename T>
            bool GetDefault(T &_value,
                            sdf::Errors &_errors) const;

    /// \brief Set the description of the parameter.
    /// \param[in] _desc New description for the parameter.
    public: void SetDescription(const std::string &_desc);

    /// \brief Get the description of the parameter.
    /// \return The description of the parameter.
    public: std::string GetDescription() const;

    /// \brief Validate the value against minimum and maximum allowed values
    /// \return True if the value is valid
    public: bool ValidateValue() const;

    /// \brief Validate the value against minimum and maximum allowed values
    /// \param[out] _errors Vector of errors.
    /// \return True if the value is valid
    public: bool ValidateValue(sdf::Errors &_errors) const;

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

    /// \brief Parent element.
    public: ElementWeakPtr parentElement;

    /// \brief Update function pointer.
    public: std::function<std::any ()> updateFunc;

    /// \def ParamVariant
    /// \brief Variant type def.
    /// Note: When a new variant is added, add variant to functions
    /// ParamPrivate::TypeToString, ParamPrivate::ValueFromStringImpl,
    /// and template std::ostream operator if new variant is floating point
    public: typedef std::variant<bool, char, std::string, int, std::uint64_t,
                                   unsigned int, double, float, sdf::Time,
                                   gz::math::Angle,
                                   gz::math::Color,
                                   gz::math::Vector2i,
                                   gz::math::Vector2d,
                                   gz::math::Vector3d,
                                   gz::math::Quaterniond,
                                   gz::math::Pose3d> ParamVariant;

    /// \brief This parameter's value
    public: ParamVariant value;

    /// \brief True if the value has been parsed while ignoring its parent
    /// element's attributes, and will continue to ignore them for subsequent
    /// reparses.
    public: bool ignoreParentAttributes;

    /// \brief This parameter's value that was provided as a string
    public: std::optional<std::string> strValue;

    /// \brief This parameter's default value that was provided as a string
    public: std::string defaultStrValue;

    /// \brief This parameter's default value
    public: ParamVariant defaultValue;

    /// \brief This parameter's minimum allowed value
    public: std::optional<ParamVariant> minValue;

    /// \brief This parameter's maximum allowed value
    public: std::optional<ParamVariant> maxValue;

    /// \brief Initializer function to help Param constructors.
    /// \param[in] _key Key for the parameter.
    /// \param[in] _typeName String name for the value type (double,
    /// int,...).
    /// \param[in] _default Default value.
    /// \param[in] _required True if the parameter is required to be set.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _description Description of the parameter.
    public: void Init(const std::string &_key, const std::string &_typeName,
                const std::string &_default, bool _required,
                sdf::Errors &_errors,
                const std::string &_description);

    /// \brief Initializer function to help Param constructors.
    /// \param[in] _key Key for the parameter.
    /// \param[in] _typeName String name for the value type (double,
    /// int,...).
    /// \param[in] _default Default value.
    /// \param[in] _required True if the parameter is required to be set.
    /// \param[in] _minValue Minimum allowed value for the parameter.
    /// \param[in] _maxValue Maximum allowed value for the parameter.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _description Description of the parameter.
    public: void Init(const std::string &_key, const std::string &_typeName,
                const std::string &_default, bool _required,
                const std::string &_minValue, const std::string &_maxValue,
                sdf::Errors &_errors,
                const std::string &_description);

    /// \brief Method used to set the Param from a passed-in string
    /// \param[in] _typeName The data type of the value to set
    /// \param[in] _valueStr The value as a string
    /// \param[out] _valueToSet The value to set
    /// \param[out] _errors Vector of errors.
    /// \return True if the value was successfully set, false otherwise
    public: bool SDFORMAT_VISIBLE ValueFromStringImpl(
                                    const std::string &_typeName,
                                    const std::string &_valueStr,
                                    ParamVariant &_valueToSet,
                                    sdf::Errors &_errors) const;

    /// \brief Method used to get the string representation from a ParamVariant,
    /// or the string that was used to set it.
    /// \param[in] _config Print configuration for the string output
    /// \param[in] _typeName The data type of the value
    /// \param[in] _value The value
    /// \param[out] _valueStr The output string.
    /// \param[out] _errors Vector of errors.
    /// \return True if the string was successfully retrieved, false otherwise.
    public: bool StringFromValueImpl(
                const PrintConfig &_config,
                const std::string &_typeName,
                const ParamVariant &_value,
                std::string &_valueStr,
                sdf::Errors &_errors) const;

    /// \brief Data type to string mapping
    /// \return The type as a string, empty string if unknown type
    public: template<typename T>
            static std::string TypeToString();
  };

  ///////////////////////////////////////////////
  template<typename T>
  std::string ParamPrivate::TypeToString()
  {
    // cppcheck-suppress syntaxError
    if constexpr (std::is_same_v<T, bool>)
      return "bool";
    else if constexpr (std::is_same_v<T, char>)
      return "char";
    else if constexpr (std::is_same_v<T, std::string>)
      return "string";
    else if constexpr (std::is_same_v<T, int>)
      return "int";
    else if constexpr (std::is_same_v<T, std::uint64_t>)
      return "uint64_t";
    else if constexpr (std::is_same_v<T, unsigned int>)
      return "unsigned int";
    else if constexpr (std::is_same_v<T, double>)
      return "double";
    else if constexpr (std::is_same_v<T, float>)
      return "float";
    else if constexpr (std::is_same_v<T, sdf::Time>)
      return "time";
    else if constexpr (std::is_same_v<T, gz::math::Angle>)
      return "angle";
    else if constexpr (std::is_same_v<T, gz::math::Color>)
      return "color";
    else if constexpr (std::is_same_v<T, gz::math::Vector2i>)
      return "vector2i";
    else if constexpr (std::is_same_v<T, gz::math::Vector2d>)
      return "vector2d";
    else if constexpr (std::is_same_v<T, gz::math::Vector3d>)
      return "vector3";
    else if constexpr (std::is_same_v<T, gz::math::Quaterniond>)
      return "quaternion";
    else if constexpr (std::is_same_v<T, gz::math::Pose3d>)
      return "pose";
    else
      return "";
  }

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
    sdf::Errors errors;
    bool result = this->Set<T>(_value, errors);
    if (!errors.empty())
      sdferr << errors;
    return result;
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Param::Set(const T &_value, sdf::Errors &_errors)
  {
    try
    {
      std::stringstream ss;
      ss << ParamStreamer<T>{_value, std::numeric_limits<int>::max()};
      return this->SetFromString(ss.str(), true, _errors);
    }
    catch(...)
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Unable to set parameter["
          + this->dataPtr->key + "]."
          + "Type used must have a stream input and output operator,"
          + "which allows proper functioning of Param."});
      return false;
    }
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Param::Get(T &_value) const
  {
    sdf::Errors errors;
    bool result = this->Get<T>(_value, errors);
    if (!errors.empty())
      sdferr << errors;
    return result;
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Param::Get(T &_value, sdf::Errors &_errors) const
  {
    T *value = std::get_if<T>(&this->dataPtr->value);
    if (value)
    {
      _value = *value;
    }
    else
    {
      std::string typeStr = this->dataPtr->TypeToString<T>();
      if (typeStr.empty())
      {
        _errors.push_back({ErrorCode::UNKNOWN_PARAMETER_TYPE,
            "Unknown parameter type[" + std::string(typeid(T).name()) + "]"});
        return false;
      }

      std::string valueStr = this->GetAsString(_errors);
      ParamPrivate::ParamVariant pv;
      bool success = this->dataPtr->ValueFromStringImpl(
          typeStr, valueStr, pv, _errors);

      if (success)
      {
        _value = std::get<T>(pv);
      }

      return success;
    }

    return true;
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Param::GetDefault(T &_value) const
  {
    sdf::Errors errors;
    bool result = this>GetDefault<T>(_value, errors);
    if (!errors.empty())
      sdferr << errors;
    return result;
  }

  ///////////////////////////////////////////////
  template<typename T>
  bool Param::GetDefault(T &_value, sdf::Errors &_errors) const
  {
    std::stringstream ss;

    try
    {
      ss << ParamStreamer{this->dataPtr->defaultValue,
                          std::numeric_limits<int>::max()};
      ss >> _value;
    }
    catch(...)
    {
      _errors.push_back({ErrorCode::PARAMETER_ERROR,
          "Unable to convert parameter["
          + this->dataPtr->key + "] "
          + "whose type is["
          + this->dataPtr->typeName + "], to "
          + "type[" + typeid(T).name() + "]"});
      return false;
    }

    return true;
  }

  ///////////////////////////////////////////////
  template<typename Type>
  bool Param::IsType() const
  {
    return std::holds_alternative<Type>(this->dataPtr->value);
  }
  }
}

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
