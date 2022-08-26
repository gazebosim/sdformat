/*
 * Copyright 2019 Open Source Robotics Foundation
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
#ifndef SDF_IMU_HH_
#define SDF_IMU_HH_

#include <string>
#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/Noise.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  class ImuPrivate;

  /// \brief Imu contains information about an imu sensor.
  /// This sensor can be attached to a link.
  class SDFORMAT_VISIBLE Imu
  {
    /// \brief Default constructor
    public: Imu();

    /// \brief Copy constructor
    /// \param[in] _imu Imu to copy.
    public: Imu(const Imu &_imu);

    /// \brief Move constructor
    /// \param[in] _imu Imu to move.
    public: Imu(Imu &&_imu) noexcept;

    /// \brief Destructor
    public: ~Imu();

    /// \brief Assignment operator.
    /// \param[in] _imu The IMU to set values from.
    /// \return *this
    public: Imu &operator=(const Imu &_imu);

    /// \brief Move assignment operator.
    /// \param[in] _imu The IMU to set values from.
    /// \return *this
    public: Imu &operator=(Imu &&_imu) noexcept;

    /// \brief Load the IMU based on an element pointer. This is *not*
    /// the usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the noise values related to the body-frame linear
    /// acceleration on the X-axis.
    /// \return Noise values for the X-axis linear acceleration.
    public: const Noise &LinearAccelerationXNoise() const;

    /// \brief Set the noise values related to the body-frame linear
    /// acceleration on the X-axis.
    /// \param[in] _noise Noise values for the X-axis linear acceleration.
    public: void SetLinearAccelerationXNoise(const Noise &_noise);

    /// \brief Get the noise values related to the body-frame linear
    /// acceleration on the Y-axis.
    /// \return Noise values for the Y-axis linear acceleration.
    public: const Noise &LinearAccelerationYNoise() const;

    /// \brief Set the noise values related to the body-frame linear
    /// acceleration on the Y-axis.
    /// \param[in] _noise Noise values for the Y-axis linear acceleration.
    public: void SetLinearAccelerationYNoise(const Noise &_noise);

    /// \brief Get the noise values related to the body-frame linear
    /// acceleration on the Z-axis.
    /// \return Noise values for the Z-axis linear acceleration.
    public: const Noise &LinearAccelerationZNoise() const;

    /// \brief Set the noise values related to the body-frame linear
    /// acceleration on the Z-axis.
    /// \param[in] _noise Noise values for the Z-axis linear acceleration.
    public: void SetLinearAccelerationZNoise(const Noise &_noise);

    /// \brief Get the noise values related to the body-frame angular
    /// velocity on the X-axis.
    /// \return Noise values for the X-axis linear acceleration.
    public: const Noise &AngularVelocityXNoise() const;

    /// \brief Set the noise values related to the body-frame angular
    /// velocity around the X-axis.
    /// \param[in] _noise Noise values for the X-axis angular velocity.
    public: void SetAngularVelocityXNoise(const Noise &_noise);

    /// \brief Get the noise values related to the body-frame angular
    /// velocity around the Y-axis.
    /// \return Noise values for the Y-axis angular velocity.
    public: const Noise &AngularVelocityYNoise() const;

    /// \brief Set the noise values related to the body-frame angular
    /// velocity around the Y-axis.
    /// \param[in] _noise Noise values for the Y-axis angular velocity.
    public: void SetAngularVelocityYNoise(const Noise &_noise);

    /// \brief Get the noise values related to the body-frame angular
    /// velocity around the Z-axis.
    /// \return Noise values for the Z-axis angular velocity.
    public: const Noise &AngularVelocityZNoise() const;

    /// \brief Set the noise values related to the body-frame angular
    /// velocity around the Z-axis.
    /// \param[in] _noise Noise values for the Z-axis angular velocity.
    public: void SetAngularVelocityZNoise(const Noise &_noise);

    /// \brief Used when localization is set to GRAV_UP or GRAV_DOWN, a
    /// projection of this vector into a plane that is orthogonal to the
    /// gravity vector defines the direction of the IMU reference frame's
    /// X-axis.  grav_dir_x is  defined in the coordinate frame as defined by
    /// the parent_frame element.
    /// \return The gravity direction.
    public: gz::math::Vector3d &GravityDirX() const;

    /// \brief Used when localization is set to GRAV_UP or GRAV_DOWN, a
    /// projection of this vector into a plane that is orthogonal to the
    /// gravity vector defines the direction of the IMU reference frame's
    /// X-axis.  grav_dir_x is  defined in the coordinate frame as defined by
    /// the parent_frame element.
    /// \param[in] _grav The gravity direction.
    public: void SetGravityDirX(const gz::math::Vector3d  &_grav) const;

    /// \brief Get the name of parent frame which the GravityDirX vector is
    /// defined relative to. It can be any valid fully scoped link name or the
    /// special reserved "world" frame. If left empty, use the sensor's own
    /// local frame.
    /// \return The name of the parent frame.
    public: const std::string &GravityDirXParentFrame() const;

    /// \brief Set the name of parent frame which the GravityDirX vector is
    /// defined relative to. It can be any valid fully scoped link name or the
    /// special reserved "world" frame. If left empty, use the sensor's own
    /// local frame.
    /// \return The name of the parent frame.
    public: void SetGravityDirXParentFrame(const std::string &_frame) const;

    /// \brief This string represents special hardcoded use cases that are
    /// commonly seen with typical robot IMU's:
    ///   - CUSTOM: use Euler angle custom_rpy orientation specification.
    ///             The orientation of the IMU's reference frame is defined
    ///             by adding the custom_rpy rotation
    ///             to the parent_frame.
    ///   - NED: The IMU XYZ aligns with NED, where NED orientation relative
    ///          to the world
    ///             is defined by the SphericalCoordinates class.
    ///   - ENU: The IMU XYZ aligns with ENU, where ENU orientation relative
    ///          to the world is defined by the SphericalCoordinates class.
    ///   - NWU: The IMU XYZ aligns with NWU, where NWU orientation relative
    ///          to the world is defined by the SphericalCoordinates class.
    ///   - GRAV_UP: where direction of gravity maps to IMU reference frame
    ///              Z-axis with Z-axis pointing in the opposite direction of
    ///              gravity. IMU reference frame X-axis direction is defined
    ///              by GravityDirX(). Note if GravityDirX() is parallel to
    ///              gravity direction, this configuration fails. Otherwise,
    ///              IMU reference frame X-axis is defined by projection of
    ///              GravtyDirX onto a plane normal to the gravity vector.
    ///              IMU reference frame Y-axis is a vector orthogonal to
    ///              both X and Z axis following the right hand rule.
    ///  - GRAV_DOWN: where direction of gravity maps to IMU reference frame
    ///               Z-axis with Z-axis pointing in the direction of gravity.
    ///               IMU reference frame X-axis direction is defined by
    ///               GravityDirX(). Note if GravityDirX() is parallel to
    ///               gravity direction, this configuration fails. Otherwise,
    ///               IMU reference frame X-axis is defined by projection of
    ///               GravityDirX() onto a plane normal to the gravity vector.
    ///               IMU reference frame Y-axis is a vector orthogonal to both
    ///               X and Z axis following the right hand rule.
    /// \return Localization frame name
    public: const std::string &Localization() const;

    /// \brief See Localization(const std::string &).
    /// \param[in] _localization Localization frame name
    public: void SetLocalization(const std::string &_localization);

    /// \brief This field and CustomRpyParentFrame are used when
    /// Localization is set to CUSTOM. Orientation
    /// (fixed axis roll, pitch yaw) transform from ParentFrame to this IMU's
    /// reference frame.
    ///
    /// Some common examples are:
    ///  - IMU reports in its local frame on boot. IMU sensor frame is the
    ///    reference frame. Example: parent_frame="", custom_rpy="0 0 0"
    ///  - IMU reports in Gazebo world frame.
    ///    Example sdf: parent_frame="world", custom_rpy="0 0 0"
    ///  - IMU reports in NWU frame. Uses SphericalCoordinates class to
    ///    determine world frame in relation to magnetic north and gravity;
    ///    i.e. rotation between North-West-Up and world (+X,+Y,+Z) frame is
    ///    defined by SphericalCoordinates class.
    ///    Example sdf given world is NWU: parent_frame="world",
    ///    custom_rpy="0 0 0"
    ///  - IMU reports in NED frame. Uses SphericalCoordinates class to
    ///    determine world frame in relation to magnetic north and gravity;
    ///    i.e. rotation between North-East-Down and world (+X,+Y,+Z) frame is
    ///    defined by SphericalCoordinates class.
    ///    Example sdf given world is NWU: parent_frame="world",
    ///    custom_rpy="M_PI 0 0"
    ///  - IMU reports in ENU frame. Uses SphericalCoordinates class to
    ///    determine world frame in relation to magnetic north and gravity;
    ///    i.e. rotation between East-North-Up and world (+X,+Y,+Z) frame is
    ///    defined by SphericalCoordinates class.
    ///    Example sdf given world is NWU: parent_frame="world",
    ///    custom_rpy="0 0 -0.5*M_PI"
    ///  - IMU reports in ROS optical frame as described in
    ///    http://www.ros.org/reps/rep-0103.html#suffix-frames, which is
    ///    (z-forward, x-left to right when facing +z, y-top to bottom when
    ///    facing +z). (default gazebo camera is +x:view direction, +y:left,
    ///    +z:up).
    ///    Example sdf: parent_frame="local", custom_rpy="-0.5*M_PI 0 -0.5*M_PI"
    /// \return Custom RPY vectory
    public: const gz::math::Vector3d &CustomRpy() const;

    /// \brief See CustomRpy() const.
    /// \param[in] Custom RPY vectory
    public: void SetCustomRpy(const gz::math::Vector3d &_rpy) const;

    /// \brief Get the name of parent frame which the custom_rpy transform is
    /// defined relative to. It can be any valid fully scoped link name or the
    /// special reserved "world" frame. If left empty, use the sensor's own
    /// local frame.
    /// \return The name of the parent frame.
    public: const std::string &CustomRpyParentFrame() const;

    /// \brief Set the name of parent frame which the custom_rpy transform is
    /// defined relative to. It can be any valid fully scoped link name or the
    /// special reserved "world" frame. If left empty, use the sensor's own
    /// local frame.
    /// \param[in] _frame The name of the parent frame.
    public: void SetCustomRpyParentFrame(const std::string &_frame) const;

    /// \brief Return true if both Imu objects contain the same values.
    /// \param[_in] _imu Imu value to compare.
    /// \returen True if 'this' == _imu.
    public: bool operator==(const Imu &_imu) const;

    /// \brief Return true this Imu object does not contain the same
    /// values as the passed in parameter.
    /// \param[_in] _imu Imu value to compare.
    /// \returen True if 'this' != _imu.
    public: bool operator!=(const Imu &_imu) const;

    /// \brief Private data pointer.
    private: ImuPrivate *dataPtr;
  };
  }
}
#endif
