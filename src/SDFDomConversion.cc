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

#include <ignition/math/Angle.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "sdf/AirPressure.hh"
#include "sdf/Altimeter.hh"
#include "sdf/Camera.hh"
#include "sdf/Lidar.hh"
#include "sdf/ForceTorque.hh"
#include "sdf/Imu.hh"
#include "sdf/Magnetometer.hh"
#include "sdf/Noise.hh"
#include "sdf/SDFDomConversion.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {

/////////////////////////////////////////////////
bool convertNoiseDomToElement(sdf::ElementPtr _elem,
                              const sdf::Noise &_noise,
                              bool simpleNoise = false)
{
  std::string noiseType;
  switch (_noise.Type())
  {
    case sdf::NoiseType::NONE:
      noiseType = "none";
      break;
    case sdf::NoiseType::GAUSSIAN:
      noiseType = "gaussian";
      break;
    case sdf::NoiseType::GAUSSIAN_QUANTIZED:
      noiseType = "gaussian_quantized";
      break;
    default:
      noiseType = "none";
  }
  // camera and lidar <noise> does not have type attribute
  if (!simpleNoise)
    _elem->GetAttribute("type")->Set<std::string>(noiseType);
  else
    _elem->GetElement("type")->Set<std::string>(noiseType);
  _elem->GetElement("mean")->Set<double>(_noise.Mean());
  _elem->GetElement("stddev")->Set<double>(_noise.StdDev());

  // camera and lidar <noise> does not have the sdf params below
  if (!simpleNoise)
  {
    _elem->GetElement("bias_mean")->Set<double>(_noise.BiasMean());
    _elem->GetElement("bias_stddev")->Set<double>(_noise.BiasStdDev());
    _elem->GetElement("dynamic_bias_stddev")->Set<double>(
      _noise.DynamicBiasStdDev());
    _elem->GetElement("dynamic_bias_correlation_time")->Set<double>(
        _noise.DynamicBiasCorrelationTime());
    _elem->GetElement("precision")->Set<double>(_noise.Precision());
  }
  return true;
}

/////////////////////////////////////////////////
bool convertCameraDomToElement(sdf::ElementPtr _elem,
                               const sdf::Camera *_camera,
                               sdf::SensorType _type)
{
  _elem->GetAttribute("name")->Set<std::string>(_camera->Name());
  _elem->GetElement("pose")->Set<ignition::math::Pose3d>(_camera->RawPose());
  _elem->GetElement("horizontal_fov")->Set<double>(
      _camera->HorizontalFov().Radian());
  sdf::ElementPtr imageElem = _elem->GetElement("image");
  imageElem->GetElement("width")->Set<double>(_camera->ImageWidth());
  imageElem->GetElement("height")->Set<double>(_camera->ImageHeight());
  imageElem->GetElement("format")->Set<std::string>(_camera->PixelFormatStr());
  sdf::ElementPtr clipElem = _elem->GetElement("clip");
  clipElem->GetElement("near")->Set<double>(_camera->NearClip());
  clipElem->GetElement("far")->Set<double>(_camera->FarClip());

  if (_type == sdf::SensorType::DEPTH_CAMERA ||
      _type == sdf::SensorType::RGBD_CAMERA)
  {
    sdf::ElementPtr depthElem = _elem->GetElement("depth_camera");
    sdf::ElementPtr depthClipElem = depthElem->GetElement("clip");
    depthClipElem->GetElement("near")->Set<double>(_camera->DepthNearClip());
    depthClipElem->GetElement("far")->Set<double>(_camera->DepthFarClip());
  }

  sdf::ElementPtr saveElem = _elem->GetElement("save");
  saveElem->GetAttribute("enabled")->Set<bool>(_camera->SaveFrames());
  if (_camera->SaveFrames())
    saveElem->GetElement("path")->Set<std::string>(_camera->SaveFramesPath());

  sdf::ElementPtr noiseElem = _elem->GetElement("noise");
  convertNoiseDomToElement(noiseElem, _camera->ImageNoise(), true);

  sdf::ElementPtr distortionElem = _elem->GetElement("distortion");
  distortionElem->GetElement("k1")->Set<double>(_camera->DistortionK1());
  distortionElem->GetElement("k2")->Set<double>(_camera->DistortionK2());
  distortionElem->GetElement("k3")->Set<double>(_camera->DistortionK3());
  distortionElem->GetElement("p1")->Set<double>(_camera->DistortionP1());
  distortionElem->GetElement("p2")->Set<double>(_camera->DistortionP2());
  distortionElem->GetElement("center")->Set<ignition::math::Vector2d>(
      _camera->DistortionCenter());

  sdf::ElementPtr lensElem = _elem->GetElement("lens");
  lensElem->GetElement("type")->Set<std::string>(_camera->LensType());
  lensElem->GetElement("scale_to_hfov")->Set<bool>(_camera->LensScaleToHfov());
  lensElem->GetElement("cutoff_angle")->Set<double>(
     _camera->LensCutoffAngle().Radian());
  lensElem->GetElement("env_texture_size")->Set<double>(
      _camera->LensEnvironmentTextureSize());
  if (_camera->LensType() == "custom")
  {
    sdf::ElementPtr customLensElem = lensElem->GetElement("custom_function");
    customLensElem->GetElement("c1")->Set<double>(_camera->LensC1());
    customLensElem->GetElement("c2")->Set<double>(_camera->LensC2());
    customLensElem->GetElement("c3")->Set<double>(_camera->LensC3());
    customLensElem->GetElement("f")->Set<double>(_camera->LensFocalLength());
    customLensElem->GetElement("fun")->Set<std::string>(
        _camera->LensFunction());
  }
  if (_camera->HasLensIntrinsics())
  {
    sdf::ElementPtr intrinsicsElem = lensElem->GetElement("intrinsics");
    intrinsicsElem->GetElement("fx")->Set<double>(_camera->LensIntrinsicsFx());
    intrinsicsElem->GetElement("fy")->Set<double>(_camera->LensIntrinsicsFy());
    intrinsicsElem->GetElement("cx")->Set<double>(_camera->LensIntrinsicsCx());
    intrinsicsElem->GetElement("cy")->Set<double>(_camera->LensIntrinsicsCy());
    intrinsicsElem->GetElement("s")->Set<double>(_camera->LensIntrinsicsSkew());
  }

  if (_camera->HasSegmentationType())
  {
    _elem->GetElement("segmentation_type")->Set<std::string>(
        _camera->SegmentationType());
  }

  return true;
}

/////////////////////////////////////////////////
bool convertSensorCommonDomToElement(sdf::ElementPtr _elem,
                               const sdf::Sensor &_sensor)
{
  _elem->GetAttribute("type")->Set<std::string>(_sensor.TypeStr());
  _elem->GetAttribute("name")->Set<std::string>(_sensor.Name());
  _elem->GetElement("pose")->Set<ignition::math::Pose3d>(_sensor.RawPose());
  _elem->GetElement("topic")->Set<std::string>(_sensor.Topic());
  _elem->GetElement("update_rate")->Set<double>(_sensor.UpdateRate());
  _elem->GetElement("enable_metrics")->Set<double>(_sensor.EnableMetrics());
  return true;
}

/////////////////////////////////////////////////
bool convertSensorDomToElement(const sdf::ElementPtr &_elem,
                               const sdf::Sensor &_sensor)
{
  // convert common elements
  convertSensorCommonDomToElement(_elem, _sensor);

  // magnetometer
  if (_sensor.Type() == sdf::SensorType::MAGNETOMETER)
  {
    const sdf::Magnetometer *magnetometer = _sensor.MagnetometerSensor();
    sdf::ElementPtr magnetometerElem = _elem->GetElement("magnetometer");
    sdf::ElementPtr magnetometerXElem = magnetometerElem->GetElement("x");
    sdf::ElementPtr magnetometerXNoiseElem =
        magnetometerXElem->GetElement("noise");
    convertNoiseDomToElement(magnetometerXNoiseElem,
        magnetometer->XNoise());
    sdf::ElementPtr magnetometerYElem = magnetometerElem->GetElement("y");
    sdf::ElementPtr magnetometerYNoiseElem =
        magnetometerYElem->GetElement("noise");
    convertNoiseDomToElement(magnetometerYNoiseElem,
        magnetometer->YNoise());
    sdf::ElementPtr magnetometerZElem = magnetometerElem->GetElement("z");
    sdf::ElementPtr magnetometerZNoiseElem =
        magnetometerZElem->GetElement("noise");
    convertNoiseDomToElement(magnetometerZNoiseElem,
        magnetometer->ZNoise());
  }
  // imu
  else if (_sensor.Type() == sdf::SensorType::IMU)
  {
    const sdf::Imu *imu = _sensor.ImuSensor();
    sdf::ElementPtr imuElem = _elem->GetElement("imu");
    sdf::ElementPtr orientationRefFrameElem =
        imuElem->GetElement("orientation_reference_frame");
    orientationRefFrameElem->GetElement("localization")->Set<std::string>(
        imu->Localization());
    std::string localizationStr = imu->Localization();

    sdf::ElementPtr customRPY =
        orientationRefFrameElem->GetElement("custom_rpy");
    customRPY->Set<ignition::math::Vector3d>(imu->CustomRpy());
    customRPY->GetAttribute("parent_frame")->Set<std::string>(
        imu->CustomRpyParentFrame());

    sdf::ElementPtr gravDirX =
        orientationRefFrameElem->GetElement("grav_dir_x");
    gravDirX->Set<ignition::math::Vector3d>(imu->GravityDirX());
    gravDirX->GetAttribute("parent_frame")->Set<std::string>(
        imu->GravityDirXParentFrame());

    sdf::ElementPtr angularVelElem = imuElem->GetElement("angular_velocity");
    sdf::ElementPtr angularVelXElem = angularVelElem->GetElement("x");
    sdf::ElementPtr angularVelXNoiseElem =
        angularVelXElem->GetElement("noise");
    convertNoiseDomToElement(angularVelXNoiseElem,
        imu->AngularVelocityXNoise());
    sdf::ElementPtr angularVelYElem = angularVelElem->GetElement("y");
    sdf::ElementPtr angularVelYNoiseElem =
        angularVelYElem->GetElement("noise");
    convertNoiseDomToElement(angularVelYNoiseElem,
        imu->AngularVelocityYNoise());
    sdf::ElementPtr angularVelZElem = angularVelElem->GetElement("z");
    sdf::ElementPtr angularVelZNoiseElem =
        angularVelZElem->GetElement("noise");
    convertNoiseDomToElement(angularVelZNoiseElem,
        imu->AngularVelocityZNoise());

    sdf::ElementPtr linearAccElem =
        imuElem->GetElement("linear_acceleration");
    sdf::ElementPtr linearAccXElem = linearAccElem->GetElement("x");
    sdf::ElementPtr linearAccXNoiseElem = linearAccXElem->GetElement("noise");
    convertNoiseDomToElement(linearAccXNoiseElem,
        imu->LinearAccelerationXNoise());
    sdf::ElementPtr linearAccYElem = linearAccElem->GetElement("y");
    sdf::ElementPtr linearAccYNoiseElem = linearAccYElem->GetElement("noise");
    convertNoiseDomToElement(linearAccYNoiseElem,
        imu->LinearAccelerationYNoise());
    sdf::ElementPtr linearAccZElem = linearAccElem->GetElement("z");
    sdf::ElementPtr linearAccZNoiseElem = linearAccZElem->GetElement("noise");
    convertNoiseDomToElement(linearAccZNoiseElem,
        imu->LinearAccelerationZNoise());

    imuElem->GetElement("enable_orientation")->Set<bool>(
        imu->OrientationEnabled());
  }
  // gpu_lidar, lidar
  else if (_sensor.Type() == sdf::SensorType::GPU_LIDAR ||
           _sensor.Type() == sdf::SensorType::LIDAR)
  {
    const sdf::Lidar *lidar = _sensor.LidarSensor();
    sdf::ElementPtr rayElem = (_elem->HasElement("ray")) ?
        _elem->GetElement("ray") : _elem->GetElement("lidar");

    sdf::ElementPtr scanElem = rayElem->GetElement("scan");
    sdf::ElementPtr horElem = scanElem->GetElement("horizontal");
    horElem->GetElement("samples")->Set<double>(
        lidar->HorizontalScanSamples());
    horElem->GetElement("resolution")->Set<double>(
        lidar->HorizontalScanResolution());
    horElem->GetElement("min_angle")->Set<double>(
        lidar->HorizontalScanMinAngle().Radian());
    horElem->GetElement("max_angle")->Set<double>(
        lidar->HorizontalScanMaxAngle().Radian());

    sdf::ElementPtr vertElem = scanElem->GetElement("vertical");
    vertElem->GetElement("samples")->Set<double>(
        lidar->VerticalScanSamples());
    vertElem->GetElement("resolution")->Set<double>(
        lidar->VerticalScanResolution());
    vertElem->GetElement("min_angle")->Set<double>(
        lidar->VerticalScanMinAngle().Radian());
    vertElem->GetElement("max_angle")->Set<double>(
        lidar->VerticalScanMaxAngle().Radian());

    sdf::ElementPtr rangeElem = rayElem->GetElement("range");
    rangeElem->GetElement("min")->Set<double>(lidar->RangeMin());
    rangeElem->GetElement("max")->Set<double>(lidar->RangeMax());
    rangeElem->GetElement("resolution")->Set<double>(
        lidar->RangeResolution());

    sdf::ElementPtr noiseElem = rayElem->GetElement("noise");
    convertNoiseDomToElement(noiseElem, lidar->LidarNoise(), true);
  }
  // altimeter
  else if (_sensor.Type() == sdf::SensorType::ALTIMETER)
  {
    const sdf::Altimeter *altimeter = _sensor.AltimeterSensor();
    sdf::ElementPtr altimeterElem = _elem->GetElement("altimeter");
    sdf::ElementPtr verticalPosElem =
        altimeterElem->GetElement("vertical_position");
    sdf::ElementPtr verticalPosNoiseElem =
        verticalPosElem->GetElement("noise");
    const sdf::Noise &vertPosNoise = altimeter->VerticalPositionNoise();
    convertNoiseDomToElement(verticalPosNoiseElem, vertPosNoise);

    sdf::ElementPtr verticalVelElem =
        altimeterElem->GetElement("vertical_velocity");
    sdf::ElementPtr verticalVelNoiseElem = verticalVelElem->GetElement("noise");
    const sdf::Noise &vertVelNoise = altimeter->VerticalVelocityNoise();
    convertNoiseDomToElement(verticalVelNoiseElem, vertVelNoise);
  }
  // air pressure
  else if (_sensor.Type() == sdf::SensorType::AIR_PRESSURE)
  {
    const sdf::AirPressure *airPressure = _sensor.AirPressureSensor();
    sdf::ElementPtr airPressureElem = _elem->GetElement("air_pressure");
    airPressureElem->GetElement("reference_altitude")->Set<double>(
        airPressure->ReferenceAltitude());
    sdf::ElementPtr pressureElem = airPressureElem->GetElement("pressure");
    sdf::ElementPtr noiseElem = pressureElem->GetElement("noise");
    convertNoiseDomToElement(noiseElem, airPressure->PressureNoise());
  }
  // force torque
  else if (_sensor.Type() == sdf::SensorType::FORCE_TORQUE)
  {
    const sdf::ForceTorque *forceTorque = _sensor.ForceTorqueSensor();
    sdf::ElementPtr forceTorqueElem = _elem->GetElement("force_torque");

    std::string frame;
    switch (forceTorque->Frame())
    {
      case sdf::ForceTorqueFrame::PARENT:
        frame = "parent";
        break;
      case sdf::ForceTorqueFrame::CHILD:
        frame = "child";
        break;
      case sdf::ForceTorqueFrame::SENSOR:
        frame = "sensor";
        break;
      case sdf::ForceTorqueFrame::INVALID:
      default:
        break;
    }
    if (!frame.empty())
      forceTorqueElem->GetElement("frame")->Set<std::string>(frame);

    std::string measureDirection;
    switch (forceTorque->MeasureDirection())
    {
      case sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD:
        measureDirection = "parent_to_child";
        break;
      case sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT:
        measureDirection = "child_to_parent";
        break;
      case sdf::ForceTorqueMeasureDirection::INVALID:
      default:
        break;
    }
    if (!measureDirection.empty())
    {
      forceTorqueElem->GetElement("measure_direction")->Set<std::string>(
          measureDirection);
    }

    sdf::ElementPtr forceElem = forceTorqueElem->GetElement("force");
    sdf::ElementPtr forceXElem = forceElem->GetElement("x");
    sdf::ElementPtr forceXNoiseElem = forceXElem->GetElement("noise");
    convertNoiseDomToElement(forceXNoiseElem, forceTorque->ForceXNoise());
    sdf::ElementPtr forceYElem = forceElem->GetElement("y");
    sdf::ElementPtr forceYNoiseElem = forceYElem->GetElement("noise");
    convertNoiseDomToElement(forceYNoiseElem, forceTorque->ForceYNoise());
    sdf::ElementPtr forceZElem = forceElem->GetElement("z");
    sdf::ElementPtr forceZNoiseElem = forceZElem->GetElement("noise");
    convertNoiseDomToElement(forceZNoiseElem, forceTorque->ForceZNoise());

    sdf::ElementPtr torqueElem = forceTorqueElem->GetElement("torque");
    sdf::ElementPtr torqueXElem = torqueElem->GetElement("x");
    sdf::ElementPtr torqueXNoiseElem = torqueXElem->GetElement("noise");
    convertNoiseDomToElement(torqueXNoiseElem, forceTorque->TorqueXNoise());
    sdf::ElementPtr torqueYElem = torqueElem->GetElement("y");
    sdf::ElementPtr torqueYNoiseElem = torqueYElem->GetElement("noise");
    convertNoiseDomToElement(torqueYNoiseElem, forceTorque->TorqueYNoise());
    sdf::ElementPtr torqueZElem = torqueElem->GetElement("z");
    sdf::ElementPtr torqueZNoiseElem = torqueZElem->GetElement("noise");
    convertNoiseDomToElement(torqueZNoiseElem, forceTorque->TorqueZNoise());
  }
  // camera, depth, thermal, segmentation
  else if (_sensor.CameraSensor())
  {
    sdf::ElementPtr cameraElem = _elem->GetElement("camera");
    const sdf::Camera *camera = _sensor.CameraSensor();
    convertCameraDomToElement(cameraElem, camera, _sensor.Type());
  }
  else
  {
    std::cout << "Conversion of sensor type: ["
              << _sensor.TypeStr() << "] from "
              << "SDF DOM to Element is not supported yet." << std::endl;
    return false;
  }
  return true;
}
}
}
