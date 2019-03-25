/*
 * Copyright 2018 Open Source Robotics Foundation
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
#include <string>
#include <vector>
#include "sdf/Error.hh"
#include "sdf/Battery.hh"
#include "sdf/Types.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::BatteryPrivate
{
  /// \brief Name of the battery.
  public: std::string name = "";

  /// \brief Voltage of the battery.
  public: double voltage = 0.0;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  public: bool LoadVoltage(sdf::ElementPtr _sdf, double &_voltage);
};

/////////////////////////////////////////////////
Battery::Battery()
  : dataPtr(new BatteryPrivate)
{
}

/////////////////////////////////////////////////
Battery::Battery(const Battery &_battery)
  : dataPtr(new BatteryPrivate)
{
  this->dataPtr->name = _battery.dataPtr->name;
  this->dataPtr->voltage = _battery.dataPtr->voltage;
  this->dataPtr->sdf = _battery.dataPtr->sdf;
}

/////////////////////////////////////////////////
Battery &Battery::operator=(const Battery &_battery)
{
  this->dataPtr->name = _battery.dataPtr->name;
  this->dataPtr->voltage = _battery.dataPtr->voltage;
  this->dataPtr->sdf = _battery.dataPtr->sdf;
  return *this;
}

/////////////////////////////////////////////////
Battery::Battery(Battery &&_battery)
{
  this->dataPtr = _battery.dataPtr;
  _battery.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Battery::~Battery()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors Battery::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a Battery, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <battery>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "battery")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Battery, but the provided SDF element is not a "
        "<battery>."});
    return errors;
  }

  // Read the battery's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
        "A battery name is required, but the name is not set."});
    return errors;
  }

  // Read the battery's voltage
  if (!this->dataPtr->LoadVoltage(_sdf, this->dataPtr->voltage))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
        "A battery voltage is required, but the voltage is not set."});
    return errors;
  }

  return errors;
}

/////////////////////////////////////////////////
bool BatteryPrivate::LoadVoltage(sdf::ElementPtr _sdf, double &_voltage)
{
  std::pair<double, bool> voltagePair = _sdf->Get<double>("voltage", 0.0);

  _voltage = voltagePair.first;
  return voltagePair.second;
}

/////////////////////////////////////////////////
std::string Battery::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Battery::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
double Battery::Voltage() const
{
  return this->dataPtr->voltage;
}

/////////////////////////////////////////////////
void Battery::SetVoltage(double _voltage)
{
  this->dataPtr->voltage = _voltage;
}

/////////////////////////////////////////////////
sdf::ElementPtr Battery::Element() const
{
  return this->dataPtr->sdf;
}
