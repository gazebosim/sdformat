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

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <boost/lexical_cast.hpp>
#endif
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/variant.hpp>
#include <boost/any.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <typeinfo>
#include <string>
#include <vector>

#include "sdf/Console.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  class SDFORMAT_VISIBLE Param;

  /// \def ParamPtr
  /// \brief boost shared_ptr to a Param
  typedef boost::shared_ptr<Param> ParamPtr;

  /// \def Param_V
  /// \brief vector or boost shared_ptrs to a Param
  typedef std::vector<ParamPtr> Param_V;

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
    public: const std::string &GetKey() const {return this->key;}

    /// \brief Get the type of the value stored.
    /// \return The std::type_info.
    public: const std::type_info &GetType() const;

    /// \brief Get the type name value.
    /// \return The type name.
    public: const std::string &GetTypeName() const;

    /// \brief Return whether the parameter is required.
    /// \return True if the parameter is required.
    public: bool GetRequired() const {return this->required;}

    /// \brief Return true if the parameter has been set.
    /// \return True if the parameter has been set.
    public: bool GetSet() const {return this->set;}

    /// \brief Clone the parameter.
    /// \return A new parameter that is the clone of this.
    public: boost::shared_ptr<Param> Clone() const;

    /// \brief Set the update function. The updateFunc will be used to
    /// set the parameter's value when Param::Update is called.
    /// \param[in] _updateFunc Function pointer to an update function.
    public: template<typename T> void SetUpdateFunc(T _updateFunc)
            {this->updateFunc = _updateFunc;}

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
            bool Set(const T &_value)
            {
              try
              {
                this->SetFromString(boost::lexical_cast<std::string>(_value));
              }
              catch(...)
              {
                sdferr << "Unable to set parameter[" << this->key << "]."
                       << "Type type used must have a stream input and output"
                       << "operator, which allow boost::lexical_cast to"
                       << "function properly.\n";
                return false;
              }
              return true;
            }

    /// \brief Get the value of the parameter.
    /// \param[out] _value The value of the parameter.
    /// \return True if parameter was successfully cast to the value type
    /// passed in.
    public: template<typename T>
            bool Get(T &_value)
            {
              try
              {
                _value = boost::lexical_cast<T>(this->value);
              }
              catch(...)
              {
                sdferr << "Unable to convert parameter[" << this->key << "] "
                       << "whose type is[" << this->typeName << "], to "
                       << "type[" << typeid(T).name() << "]\n";
                return false;
              }
              return true;
            }

    /// \brief Get the default value of the parameter.
    /// \param[out] _value The default value of the parameter.
    /// \return True if parameter was successfully cast to the value type
    /// passed in.
    public: template<typename T>
            bool GetDefault(T &_value)
            {
              try
              {
                _value = boost::lexical_cast<T>(this->defaultValue);
              }
              catch(...)
              {
                sdferr << "Unable to convert parameter[" << this->key << "] "
                       << "whose type is[" << this->typeName << "], to "
                       << "type[" << typeid(T).name() << "]\n";
                return false;
              }
              return true;
            }

    /// \brief Equal operator. Set's the value and default value from the
    /// provided Param.
    /// \param[in] _param The parameter to set values from.
    /// \return *This
    public: Param &operator =(const Param &_param)
            {
              this->value = _param.value;
              this->defaultValue  = _param.defaultValue;
              return *this;
            }

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
            void Init(const std::string &_value)
            {
              try
              {
                this->value = boost::lexical_cast<T>(_value);
              }
              catch(...)
              {
                if (this->typeName == "bool")
                {
                  std::string strValue = _value;
                  boost::algorithm::to_lower(strValue);
                  if (strValue == "true" || strValue == "1")
                    this->value = true;
                  else
                    this->value = false;
                }
                else
                  sdferr << "Unable to init parameter value from string["
                    << _value << "]\n";
              }

              this->defaultValue = this->value;
              this->set = false;
            }

    /// \brief Key value
    private: std::string key;

    /// \brief True if the parameter is required.
    private: bool required;

    /// \brief True if the parameter is set.
    private: bool set;

    //// \brief Name of the type.
    private: std::string typeName;

    /// \brief Description of the parameter.
    private: std::string description;

    /// \brief Update function pointer.
    private: boost::function<boost::any ()> updateFunc;

    /// \def ParamVariant
    /// \briead Variant type def.
    private: typedef boost::variant<bool, char, std::string, int,
               unsigned int, double, float, sdf::Vector3, sdf::Vector2i,
               sdf::Vector2d, sdf::Quaternion, sdf::Pose, sdf::Color,
               sdf::Time> ParamVariant;

    /// \brief This parameter's value
    protected: ParamVariant value;

    /// \brief This parameter's default value
    protected: ParamVariant defaultValue;
  };
}
#endif
