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
#ifndef SDF_SENSOR_HH_
#define SDF_SENSOR_HH_

#include <memory>
#include <string>
#include <gz/math/Pose3.hh>
#include <gz/utils/ImplPtr.hh>
#include "sdf/Element.hh"
#include "sdf/Plugin.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Types.hh"
#include "sdf/config.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declarations.
  class AirPressure;
  class AirSpeed;
  class Altimeter;
  class Camera;
  class ForceTorque;
  class Imu;
  class Lidar;
  class Magnetometer;
  class NavSat;
  struct PoseRelativeToGraph;
  template <typename T> class ScopedGraph;

  /// \enum SensorType
  /// \brief The set of sensor types.
  // Developer note: Make sure to update kSensorTypeStrs in the source file
  // when changing this enum.
  enum class SensorType
  {
    /// \brief An unspecified sensor type.
    NONE = 0,

    /// \brief An altimeter sensor.
    ALTIMETER = 1,

    /// \brief A monocular camera sensor.
    CAMERA = 2,

    /// \brief A contact sensor.
    CONTACT = 3,

    /// \brief A depth camera sensor.
    DEPTH_CAMERA = 4,

    /// \brief A force-torque sensor.
    FORCE_TORQUE = 5,

    /// \brief A GPS sensor.
    GPS = 6,

    /// \brief A GPU based lidar sensor.
    GPU_LIDAR = 7,

    /// \brief An IMU sensor.
    IMU = 8,

    /// \brief A logical camera sensor.
    LOGICAL_CAMERA = 9,

    /// \brief A magnetometer sensor.
    MAGNETOMETER = 10,

    /// \brief A multicamera sensor.
    MULTICAMERA = 11,

    /// \brief A CPU based lidar sensor.
    LIDAR = 12,

    /// \brief An RFID sensor.
    RFID = 13,

    /// \brief An RFID tag.
    RFIDTAG = 14,

    /// \brief A sonar tag sensor.
    SONAR = 15,

    /// \brief A wireless receiver.
    WIRELESS_RECEIVER = 16,

    /// \brief A wireless transmitter.
    WIRELESS_TRANSMITTER = 17,

    /// \brief An air pressure sensor.
    AIR_PRESSURE = 18,

    /// \brief An RGBD sensor, which produces both a color image and
    /// a depth image.
    RGBD_CAMERA = 19,

    /// \brief A thermal camera sensor
    THERMAL_CAMERA = 20,

    /// \brief A NavSat sensor, such as GPS.
    NAVSAT = 21,

    /// \brief A segmentation camera sensor
    SEGMENTATION_CAMERA = 22,

    /// \brief A boundingbox camera sensor
    BOUNDINGBOX_CAMERA = 23,

    /// \brief A custom sensor
    CUSTOM = 24,

    /// \brief A wide angle camera sensor
    WIDE_ANGLE_CAMERA = 25,

    /// \brief An air speed sensor.
    AIR_SPEED = 26,
  };

  /// \brief Information about an SDF sensor.
  class SDFORMAT_VISIBLE Sensor
  {
    /// \brief Default constructor
    public: Sensor();

    /// \brief Load the sensor based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the sensor.
    /// The name of the sensor should be unique within the scope of a World.
    /// \return Name of the sensor.
    public: std::string Name() const;

    /// \brief Set the name of the sensor.
    /// The name of the sensor should be unique within the scope of a World.
    /// \param[in] _name Name of the sensor.
    public: void SetName(const std::string &_name);

    /// \brief Get the frame ID of the sensor.
    /// \return Sensor frame ID.
    public: std::string FrameId() const;

    /// \brief Set the frame ID of the sensor.
    /// \param[in] _frameId Frame ID to set.
    public: void SetFrameId(const std::string &_frameId);

    /// \brief Get the topic on which sensor data should be published.
    /// \return Topic for this sensor's data.
    public: std::string Topic() const;

    /// \brief Set the topic on which sensor data should be published.
    /// \param[in] _topic Topic for this sensor's data.
    public: void SetTopic(const std::string &_topic);

    /// \brief Get flag state for enabling performance metrics publication.
    /// \return True if performance metrics are enabled, false otherwise.
    public: bool EnableMetrics() const;

    /// \brief Set flag to enable publishing performance metrics
    /// \param[in] _enableMetrics True to enable.
    public: void SetEnableMetrics(bool _enableMetrics);

    /// \brief Get the pose of the sensor. This is the pose of the sensor
    /// as specified in SDF (<sensor> <pose> ... </pose></sensor>), and is
    /// typically used to express the position and rotation of a sensor in a
    /// global coordinate frame.
    /// \return The pose of the sensor.
    public: const gz::math::Pose3d &RawPose() const;

    /// \brief Set the pose of the sensor.
    /// \sa const gz::math::Pose3d &RawPose() const
    /// \param[in] _pose The new sensor pose.
    public: void SetRawPose(const gz::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent link/joint coordinate frame.
    /// \return The name of the pose relative-to frame.
    public: const std::string &PoseRelativeTo() const;

    /// \brief Set the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent link/joint coordinate frame.
    /// \param[in] _frame The name of the pose relative-to frame.
    public: void SetPoseRelativeTo(const std::string &_frame);

    /// \brief Get SemanticPose object of this object to aid in resolving
    /// poses.
    /// \return SemanticPose object for this link.
    public: sdf::SemanticPose SemanticPose() const;

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Create and return an SDF element filled with data from this
    /// sensor.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated sensor values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Create and return an SDF element filled with data from this
    /// sensor.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[in] _errors Vector of errors.
    /// \return SDF element pointer with updated sensor values.
    public: sdf::ElementPtr ToElement(sdf::Errors &_errors) const;

    /// \brief Get the sensor type.
    /// \return The sensor type.
    public: SensorType Type() const;

    /// \brief Set the sensor type.
    /// \param[in] _type The sensor type.
    public: void SetType(const SensorType _type);

    /// \brief Set the sensor type from a string.
    /// \param[in] _typeStr The sensor type. A valid parameter should equal
    /// one of the enum value name in the SensorType enum. For example,
    /// "altimeter" or "camera".
    /// \return True if the _typeStr parameter matched a known sensor type.
    /// False if the sensor type could not be set.
    public: bool SetType(const std::string &_typeStr);

    /// \brief Get the sensor type as a string.
    /// \return The sensor type as a string.
    public: std::string TypeStr() const;

    /// \brief Get the update rate in Hz.
    /// This is The frequency at which the sensor data is generated.
    /// If left unspecified (0.0), the sensor will generate data every cycle.
    /// \return The update rate in Hz.
    public: double UpdateRate() const;

    /// \brief Set the update rate.
    /// This is The frequency at which the sensor data is generated.
    /// If left unspecified (0.0), the sensor will generate data every cycle.
    /// \param[in] _rate The update rate in Hz.
    public: void SetUpdateRate(double _hz);

    /// \brief Return true if both Sensor objects contain the same values.
    /// \param[_in] _sensor Sensor object to compare.
    /// \return True if 'this' == _sensor.
    public: bool operator==(const Sensor &_sensor) const;

    /// \brief Return true this Sensor object does not contain the same
    /// values as the passed in parameter.
    /// \param[_in] _sensor Sensor object to compare.
    /// \return True if 'this' != _sensor.
    public: bool operator!=(const Sensor &_sensor) const;

    /// \brief Get the magnetometer sensor, or nullptr if this sensor type
    /// is not a Magnetometer.
    /// \return Pointer to the Magnetometer sensor, or nullptr if this
    /// Sensor is not a Magnetometer.
    /// \sa SensorType Type() const
    public: const Magnetometer *MagnetometerSensor() const;

    /// \brief Get a mutable magnetometer sensor, or nullptr if this sensor type
    /// is not a Magnetometer.
    /// \return Pointer to the Magnetometer sensor, or nullptr if this
    /// Sensor is not a Magnetometer.
    /// \sa SensorType Type() const
    public: Magnetometer *MagnetometerSensor();

    /// \brief Set the magnetometer sensor.
    /// \param[in] _mag The magnetometer sensor.
    public: void SetMagnetometerSensor(const Magnetometer &_mag);

    /// \brief Get the altimeter sensor, or nullptr if this sensor type
    /// is not an Altimeter.
    /// \return Pointer to the Altimeter sensor, or nullptr if this
    /// Sensor is not a Altimeter.
    /// \sa SensorType Type() const
    public: const Altimeter *AltimeterSensor() const;

    /// \brief Get a mutable altimeter sensor, or nullptr if this sensor type
    /// is not an Altimeter.
    /// \return Pointer to the Altimeter sensor, or nullptr if this
    /// Sensor is not a Altimeter.
    /// \sa SensorType Type() const
    public: Altimeter *AltimeterSensor();

    /// \brief Set the altimeter sensor.
    /// \param[in] _alt The altimeter sensor.
    public: void SetAltimeterSensor(const Altimeter &_alt);

    /// \brief Get the air pressure sensor, or nullptr if this sensor type
    /// is not an AirPressure sensor.
    /// \return Pointer to the AirPressure sensor, or nullptr if this
    /// Sensor is not a AirPressure sensor.
    /// \sa SensorType Type() const
    public: const AirPressure *AirPressureSensor() const;

    /// \brief Get a mutable air pressure sensor, or nullptr if this sensor type
    /// is not an AirPressure sensor.
    /// \return Pointer to the AirPressure sensor, or nullptr if this
    /// Sensor is not a AirPressure sensor.
    /// \sa SensorType Type() const
    public: AirPressure *AirPressureSensor();

    /// \brief Set the air pressure sensor.
    /// \param[in] _air The air pressure sensor.
    public: void SetAirPressureSensor(const AirPressure &_air);

    /// \brief Get the air speed sensor, or nullptr if this sensor type
    /// is not an AirSpeed sensor.
    /// \return Pointer to the AirSpeed sensor, or nullptr if this
    /// Sensor is not a AirSpeed sensor.
    /// \sa SensorType Type() const
    public: const AirSpeed *AirSpeedSensor() const;

    /// \brief Get a mutable air speed sensor, or nullptr if this sensor type
    /// is not an AirSpeed sensor.
    /// \return Pointer to the AirSpeed sensor, or nullptr if this
    /// Sensor is not a AirSpeed sensor.
    /// \sa SensorType Type() const
    public: AirSpeed *AirSpeedSensor();

    /// \brief Set the air pressure sensor.
    /// \param[in] _air The air pressure sensor.
    public: void SetAirSpeedSensor(const AirSpeed &_air);

    /// \brief Set the camera sensor.
    /// \param[in] _cam The camera sensor.
    public: void SetCameraSensor(const Camera &_cam);

    /// \brief Get a pointer to a camera sensor, or nullptr if the sensor
    /// does not contain a camera sensor.
    /// \return Pointer to the sensor's camera, or nullptr if the sensor
    /// is not a camera.
    /// \sa SensorType Type() const
    public: const Camera *CameraSensor() const;

    /// \brief Get a mutable camera sensor, or nullptr if the
    /// sensor does not contain a camera sensor.
    /// \return Pointer to the sensor's camera, or nullptr if the sensor
    /// is not a camera.
    /// \sa SensorType Type() const
    public: Camera *CameraSensor();

    /// \brief Set the NAVSAT sensor.
    /// \param[in] _navsat The NAVSAT sensor.
    public: void SetNavSatSensor(const NavSat &_navsat);

    /// \brief Get a pointer to a NAVSAT sensor, or nullptr if the sensor
    /// does not contain an NAVSAT sensor.
    /// \return Pointer to the sensor's NAVSAT, or nullptr if the sensor
    /// is not an NAVSAT.
    /// \sa SensorType Type() const
    public: const NavSat *NavSatSensor() const;

    /// \brief Get a mutable NAVSAT sensor, or nullptr if the sensor
    /// does not contain an NAVSAT sensor.
    /// \return Pointer to the sensor's NAVSAT, or nullptr if the sensor
    /// is not an NAVSAT.
    /// \sa SensorType Type() const
    public: NavSat *NavSatSensor();

    /// \brief Set the force torque sensor.
    /// \param[in] _ft The force torque sensor.
    public: void SetForceTorqueSensor(const ForceTorque &_ft);

    /// \brief Get a pointer to a force torque sensor, or nullptr if the sensor
    /// does not contain a force torque sensor.
    /// \return Pointer to the force torque sensor, or nullptr if the sensor
    /// is not a force torque sensor.
    /// \sa SensorType Type() const
    public: const ForceTorque *ForceTorqueSensor() const;

    /// \brief Get a mutable force torque sensor, or nullptr if the sensor
    /// does not contain a force torque sensor.
    /// \return Pointer to the force torque sensor, or nullptr if the sensor
    /// is not a force torque sensor.
    /// \sa SensorType Type() const
    public: ForceTorque *ForceTorqueSensor();

    /// \brief Set the IMU sensor.
    /// \param[in] _imu The IMU sensor.
    public: void SetImuSensor(const Imu &_imu);

    /// \brief Get a pointer to an IMU sensor, or nullptr if the sensor
    /// does not contain an IMU sensor.
    /// \return Pointer to the sensor's IMU, or nullptr if the sensor
    /// is not an IMU.
    /// \sa SensorType Type() const
    public: const Imu *ImuSensor() const;

    /// \brief Get a mutable IMU sensor, or nullptr if the sensor
    /// does not contain an IMU sensor.
    /// \return Pointer to the sensor's IMU, or nullptr if the sensor
    /// is not an IMU.
    /// \sa SensorType Type() const
    public: Imu *ImuSensor();

    /// \brief Get the lidar sensor, or nullptr if this sensor type is not a
    /// Lidar.
    /// \return Pointer to the Lidar sensor, or nullptr if this Sensor is not a
    /// Lidar.
    /// \sa SensorType Type() const
    public: const Lidar *LidarSensor() const;

    /// \brief Get a mutable lidar sensor, or nullptr if this sensor type is
    /// not a Lidar.
    /// \return Pointer to the Lidar sensor, or nullptr if this Sensor is not a
    /// Lidar.
    /// \sa SensorType Type() const
    public: Lidar *LidarSensor();

    /// \brief Set the lidar sensor.
    /// \param[in] _lidar The lidar sensor.
    public: void SetLidarSensor(const Lidar &_lidar);

    /// \brief Get the plugins attached to this object.
    /// \return A vector of Plugin, which will be empty if there are no
    /// plugins.
    public: const sdf::Plugins &Plugins() const;

    /// \brief Get a mutable vector of plugins attached to this object.
    /// \return A vector of Plugin, which will be empty if there are no
    /// plugins.
    public: sdf::Plugins &Plugins();

    /// \brief Remove all plugins
    public: void ClearPlugins();

    /// \brief Add a plugin to this object.
    /// \param[in] _plugin Plugin to add.
    public: void AddPlugin(const Plugin &_plugin);

    /// \brief Give the name of the xml parent of this object, to be used
    /// for resolving poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph.
    /// \param[in] _xmlParentName Name of xml parent object.
    private: void SetXmlParentName(const std::string &_xmlParentName);

    /// \brief Give the scoped PoseRelativeToGraph to be used for resolving
    /// poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph.
    /// \param[in] _graph scoped PoseRelativeToGraph object.
    private: void SetPoseRelativeToGraph(
        sdf::ScopedGraph<PoseRelativeToGraph> _graph);

    /// \brief Allow Link::SetPoseRelativeToGraph to call SetXmlParentName
    /// and SetPoseRelativeToGraph, but Link::SetPoseRelativeToGraph is
    /// a private function, so we need to befriend the entire class.
    friend class Link;
    /// \brief Allow Joint::SetPoseRelativeToGraph to call SetXmlParentName
    /// and SetPoseRelativeToGraph, but Joint::SetPoseRelativeToGraph is
    /// a private function, so we need to befriend the entire class.
    friend class Joint;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
