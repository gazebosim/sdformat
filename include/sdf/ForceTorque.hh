/*
 * Copyright 2020 Open Source Robotics Foundation
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
#ifndef SDF_FORCE_TORQUE_HH_
#define SDF_FORCE_TORQUE_HH_

#include <string>
#include <gz/utils/ImplPtr.hh>
#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/Noise.hh>
#include <sdf/config.hh>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  /// \enum ForceTorqueFrame
  /// \brief The set of supported frames of the wrench values.
  enum class ForceTorqueFrame : uint8_t
  {
    /// \brief Invalid frame
    INVALID = 0,

    /// \brief Wrench expressed in the orientation of the parent link frame
    PARENT = 1,

    /// \brief Wrench expressed in the orientation of the child link frame
    CHILD = 2,

    /// \brief Wrench expressed in the orientation of the joint sensor frame
    SENSOR = 3
  };

  /// \enum ForceTorqueMeasureDirection
  /// \brief The set of measure directions of the wrench values.
  enum class ForceTorqueMeasureDirection : uint8_t
  {
    /// \brief Invalid frame
    INVALID = 0,

    /// \brief Wrench measured as applied by the parent link on the child link
    PARENT_TO_CHILD = 1,

    /// \brief Wrench measured as applied by the child link on the parent link
    CHILD_TO_PARENT = 2
  };

  /// \brief ForceTorque contains information about a force torque sensor.
  /// This sensor can be attached to a joint.
  class SDFORMAT_VISIBLE ForceTorque
  {
    /// \brief Default constructor
    public: ForceTorque();

    /// \brief Load the force torque sensor based on an element pointer. This is
    /// *not* the usual entry point. Typical usage of the SDF DOM is through the
    /// Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get a pointer to the SDF element that was used during load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the force noise values in the measurement frame X-axis.
    /// \return Noise values for the X-axis force.
    public: const Noise &ForceXNoise() const;

    /// \brief Set the force noise values in the measurement frame X-axis.
    /// \param[in] _noise Noise values for the X-axis force.
    public: void SetForceXNoise(const Noise &_noise);

    /// \brief Get the force noise values in the measurement frame Y-axis.
    /// \return Noise values for the Y-axis force.
    public: const Noise &ForceYNoise() const;

    /// \brief Set the force noise values in the measurement frame Y-axis.
    /// \param[in] _noise Noise values for the Y-axis force.
    public: void SetForceYNoise(const Noise &_noise);

    /// \brief Get the force noise values in the measurement frame Z-axis.
    /// \return Noise values for the Z-axis force.
    public: const Noise &ForceZNoise() const;

    /// \brief Set the force noise values in the measurement frame Z-axis.
    /// \param[in] _noise Noise values for the Z-axis force.
    public: void SetForceZNoise(const Noise &_noise);

    /// \brief Get the torque noise values in the measurement frame X-axis.
    /// \return Noise values for the X-axis torque.
    public: const Noise &TorqueXNoise() const;

    /// \brief Set the torque noise values in the measurement frame X-axis.
    /// \param[in] _noise Noise values for the X-axis torque.
    public: void SetTorqueXNoise(const Noise &_noise);

    /// \brief Get the torque noise values in the measurement frame Y-axis.
    /// \return Noise values for the Y-axis torque.
    public: const Noise &TorqueYNoise() const;

    /// \brief Set the torque noise values in the measurement frame Y-axis.
    /// \param[in] _noise Noise values for the Y-axis torque.
    public: void SetTorqueYNoise(const Noise &_noise);

    /// \brief Get the torque noise values in the measurement frame Z-axis.
    /// \return Noise values for the Z-axis torque.
    public: const Noise &TorqueZNoise() const;

    /// \brief Set the torque noise values in the measurement frame Z-axis.
    /// \param[in] _noise Noise values for the Z-axis torque.
    public: void SetTorqueZNoise(const Noise &_noise);

    /// \brief Get the frame in which the wrench values are reported.
    /// \return The frame of the wrench values.
    public: ForceTorqueFrame Frame() const;

    /// \brief Set the frame in which the wrench values are reported.
    /// \param[in] _frame The frame of the wrench values.
    public: void SetFrame(ForceTorqueFrame _frame);

    /// \brief Get the measure direction of the wrench values.
    /// \return The measure direction of the wrench values.
    public: ForceTorqueMeasureDirection MeasureDirection() const;

    /// \brief Set the measure direction of the wrench values.
    /// \param[in] _direction The measure direction of the wrench values.
    public: void SetMeasureDirection(ForceTorqueMeasureDirection _direction);

    /// \brief Return true if both force torque objects contain the same values.
    /// \param[_in] _ft Force torque value to compare.
    /// \return True if 'this' == _ft.
    public: bool operator==(const ForceTorque &_ft) const;

    /// \brief Return true this force torque object does not contain the same
    /// values as the passed-in parameter.
    /// \param[_in] _ft Force torque value to compare.
    /// \return True if 'this' != _ft.
    public: bool operator!=(const ForceTorque &_ft) const;

    /// \brief Create and return an SDF element filled with data from this
    /// force torque sensor.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated sensor values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Create and return an SDF element filled with data from this
    /// force torque sensor.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[out] _errors Vector of errors.
    /// \return SDF element pointer with updated sensor values.
    public: sdf::ElementPtr ToElement(sdf::Errors &_errors) const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}

#endif
