# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http:#www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
from gz_test_deps.math import Pose3d
from gz_test_deps.sdformat import (AirPressure, Altimeter, Camera, IMU,
                                   ForceTorque, Lidar, Magnetometer, NavSat,
                                   Noise, Plugin, SemanticPose, Sensor,
                                   SDFErrorsException)
import gz_test_deps.sdformat as sdf
import unittest


class SensorTEST(unittest.TestCase):

    def test_default_construction(self):
        sensor = Sensor()
        sensor2 = Sensor()
        self.assertTrue(sensor == sensor2)
        self.assertFalse(sensor != sensor2)

        self.assertEqual(sdf.Sensortype.NONE, sensor.type())

        sensor.set_type(sdf.Sensortype.ALTIMETER)
        self.assertEqual(sdf.Sensortype.ALTIMETER, sensor.type())

        self.assertEqual(Pose3d.ZERO, sensor.raw_pose())
        self.assertFalse(sensor.pose_relative_to())

        semanticPose = sensor.semantic_pose()
        self.assertEqual(sensor.raw_pose(), semanticPose.raw_pose())
        self.assertFalse(semanticPose.relative_to())
        # expect errors when trying to resolve pose
        with self.assertRaises(SDFErrorsException):
            semanticPose.resolve()

        sensor.set_raw_pose(Pose3d(1, 2, 3, 0, 0, 0))
        self.assertEqual(Pose3d(1, 2, 3, 0, 0, 0), sensor.raw_pose())

        sensor.set_pose_relative_to("a_frame")
        self.assertEqual("a_frame", sensor.pose_relative_to())

        semanticPose = sensor.semantic_pose()
        self.assertEqual(sensor.raw_pose(), semanticPose.raw_pose())
        self.assertEqual("a_frame", semanticPose.relative_to())
        # expect errors when trying to resolve pose
        with self.assertRaises(SDFErrorsException):
            semanticPose.resolve()

        self.assertAlmostEqual(0.0, sensor.update_rate())

        self.assertFalse(sensor.frame_id())
        self.assertFalse(sensor.topic())
        self.assertFalse(sensor == sensor2)
        self.assertTrue(sensor != sensor2)

    def test_copy_construction(self):
        sensor = Sensor()
        sensor.set_frame_id("test_frame_id")
        sensor.set_raw_pose(Pose3d(1, 2, 3, 0, 0, 0))
        sensor.set_type(sdf.Sensortype.MAGNETOMETER)
        sensor.set_pose_relative_to("a_frame")
        sensor.set_update_rate(0.123)

        noise = Noise()
        noise.set_mean(0.1)
        mag = Magnetometer()
        mag.set_x_noise(noise)
        sensor.set_magnetometer_sensor(mag)

        sensor2 = Sensor(sensor)

        self.assertEqual("test_frame_id", sensor.frame_id())
        self.assertEqual(sdf.Sensortype.MAGNETOMETER, sensor.type())
        self.assertEqual(Pose3d(1, 2, 3, 0, 0, 0), sensor.raw_pose())
        self.assertEqual("a_frame", sensor.pose_relative_to())
        self.assertTrue(None != sensor.magnetometer_sensor())
        self.assertAlmostEqual(mag.x_noise().mean(),
                               sensor.magnetometer_sensor().x_noise().mean())

        self.assertEqual("test_frame_id", sensor2.frame_id())
        self.assertEqual(sdf.Sensortype.MAGNETOMETER, sensor2.type())
        self.assertEqual(Pose3d(1, 2, 3, 0, 0, 0), sensor2.raw_pose())
        self.assertEqual("a_frame", sensor2.pose_relative_to())
        self.assertTrue(None != sensor2.magnetometer_sensor())
        self.assertAlmostEqual(mag.x_noise().mean(),
                               sensor2.magnetometer_sensor().x_noise().mean())
        self.assertAlmostEqual(0.123, sensor2.update_rate())

    def test_deepcopy(self):
        sensor = Sensor()
        sensor.set_frame_id("test_frame_id")
        sensor.set_raw_pose(Pose3d(1, 2, 3, 0, 0, 0))
        sensor.set_type(sdf.Sensortype.MAGNETOMETER)
        sensor.set_pose_relative_to("a_frame")

        noise = Noise()
        noise.set_mean(0.1)
        mag = Magnetometer()
        mag.set_x_noise(noise)
        sensor.set_magnetometer_sensor(mag)

        sensor2 = copy.deepcopy(sensor)

        self.assertEqual("test_frame_id", sensor.frame_id())
        self.assertEqual(sdf.Sensortype.MAGNETOMETER, sensor.type())
        self.assertEqual(Pose3d(1, 2, 3, 0, 0, 0), sensor.raw_pose())
        self.assertEqual("a_frame", sensor.pose_relative_to())
        self.assertTrue(None != sensor.magnetometer_sensor())
        self.assertAlmostEqual(mag.x_noise().mean(),
                               sensor.magnetometer_sensor().x_noise().mean())

        self.assertEqual("test_frame_id", sensor2.frame_id())
        self.assertEqual(sdf.Sensortype.MAGNETOMETER, sensor2.type())
        self.assertEqual(Pose3d(1, 2, 3, 0, 0, 0), sensor2.raw_pose())
        self.assertEqual("a_frame", sensor2.pose_relative_to())
        self.assertTrue(None != sensor2.magnetometer_sensor())
        self.assertAlmostEqual(mag.x_noise().mean(),
                               sensor2.magnetometer_sensor().x_noise().mean())

    def test_type(self):
        sensor = Sensor()

        self.assertEqual(sdf.Sensortype.NONE, sensor.type())
        self.assertEqual("none", sensor.type_str())

        types = [
            sdf.Sensortype.NONE,
            sdf.Sensortype.ALTIMETER,
            sdf.Sensortype.BOUNDINGBOX_CAMERA,
            sdf.Sensortype.CAMERA,
            sdf.Sensortype.CONTACT,
            sdf.Sensortype.DEPTH_CAMERA,
            sdf.Sensortype.FORCE_TORQUE,
            sdf.Sensortype.NAVSAT,
            sdf.Sensortype.GPU_LIDAR,
            sdf.Sensortype.IMU,
            sdf.Sensortype.LOGICAL_CAMERA,
            sdf.Sensortype.MAGNETOMETER,
            sdf.Sensortype.MULTICAMERA,
            sdf.Sensortype.LIDAR,
            sdf.Sensortype.RFID,
            sdf.Sensortype.RFIDTAG,
            sdf.Sensortype.SEGMENTATION_CAMERA,
            sdf.Sensortype.SONAR,
            sdf.Sensortype.WIRELESS_RECEIVER,
            sdf.Sensortype.WIRELESS_TRANSMITTER,
            sdf.Sensortype.THERMAL_CAMERA,
            sdf.Sensortype.CUSTOM,
            sdf.Sensortype.WIDE_ANGLE_CAMERA
        ]
        type_strs = [
            "none",
            "altimeter",
            "boundingbox_camera",
            "camera",
            "contact",
            "depth_camera",
            "force_torque",
            "navsat",
            "gpu_lidar",
            "imu",
            "logical_camera",
            "magnetometer",
            "multicamera",
            "lidar",
            "rfid",
            "rfidtag",
            "segmentation_camera",
            "sonar",
            "wireless_receiver",
            "wireless_transmitter",
            "thermal_camera",
            "custom",
            "wide_angle_camera"
        ]

        for i in range(len(types)):
            sensor.set_type(types[i])
            self.assertEqual(types[i], sensor.type())
            self.assertEqual(type_strs[i], sensor.type_str())

        for i in range(len(type_strs)):
            self.assertTrue(sensor.set_type(type_strs[i]))
            self.assertEqual(types[i], sensor.type())
            self.assertEqual(type_strs[i], sensor.type_str())

            self.assertFalse(sensor.set_type("bad_sensor_type"))
            self.assertEqual(types[i], sensor.type())
            self.assertEqual(type_strs[i], sensor.type_str())

    def test_mutable_sensors(self):
        # Altimeter
        sensor = Sensor()
        sensor.set_type(sdf.Sensortype.ALTIMETER)

        alt = Altimeter()
        sensor.set_altimeter_sensor(alt)

        altMutable = sensor.altimeter_sensor()
        self.assertNotEqual(None, altMutable)
        self.assertAlmostEqual(altMutable.vertical_position_noise().mean(),
        sensor.altimeter_sensor().vertical_position_noise().mean())

        noise = Noise()
        noise.set_mean(2.0)
        altMutable.set_vertical_position_noise(noise)
        self.assertAlmostEqual(altMutable.vertical_position_noise().mean(), 2.0)
        self.assertAlmostEqual(
        sensor.altimeter_sensor().vertical_position_noise().mean(), 2.0)

        # Air pressure
        sensor = Sensor()
        sensor.set_type(sdf.Sensortype.AIR_PRESSURE)

        air = AirPressure()
        sensor.set_air_pressure_sensor(air)

        airMutable = sensor.air_pressure_sensor()
        self.assertNotEqual(None, airMutable)
        self.assertAlmostEqual(airMutable.reference_altitude(),
        sensor.air_pressure_sensor().reference_altitude())

        airMutable.set_reference_altitude(2.0)
        self.assertAlmostEqual(airMutable.reference_altitude(), 2.0)
        self.assertAlmostEqual(
            sensor.air_pressure_sensor().reference_altitude(),
            2.0)

        # Camera
        sensor = Sensor()
        sensor.set_type(sdf.Sensortype.CAMERA)

        cam = Camera()
        sensor.set_camera_sensor(cam)

        camMutable = sensor.camera_sensor()
        self.assertNotEqual(None, camMutable)
        self.assertAlmostEqual(camMutable.near_clip(), sensor.camera_sensor().near_clip())

        camMutable.set_near_clip(2.0)
        self.assertAlmostEqual(camMutable.near_clip(), 2.0)
        self.assertAlmostEqual(sensor.camera_sensor().near_clip(), 2.0)

        # Force torque
        sensor = Sensor()
        sensor.set_type(sdf.Sensortype.FORCE_TORQUE)

        ftq = ForceTorque()
        sensor.set_force_torque_sensor(ftq)

        ftqMutable = sensor.force_torque_sensor()
        self.assertNotEqual(None, ftqMutable)
        self.assertAlmostEqual(ftqMutable.force_x_noise().mean(),
        sensor.force_torque_sensor().force_x_noise().mean())

        noise = Noise()
        noise.set_mean(2.0)
        ftqMutable.set_force_x_noise(noise)
        self.assertAlmostEqual(ftqMutable.force_x_noise().mean(), 2.0)
        self.assertAlmostEqual(
        sensor.force_torque_sensor().force_x_noise().mean(), 2.0)

        # IMU
        sensor = Sensor()
        sensor.set_type(sdf.Sensortype.FORCE_TORQUE)

        imu = IMU()
        sensor.set_imu_sensor(imu)

        imuMutable = sensor.imu_sensor()
        self.assertNotEqual(None, imuMutable)
        self.assertAlmostEqual(imuMutable.linear_acceleration_x_noise().mean(),
        sensor.imu_sensor().linear_acceleration_x_noise().mean())

        noise = Noise()
        noise.set_mean(2.0)
        imuMutable.set_linear_acceleration_x_noise(noise)
        self.assertAlmostEqual(imuMutable.linear_acceleration_x_noise().mean(), 2.0)
        self.assertAlmostEqual(
        sensor.imu_sensor().linear_acceleration_x_noise().mean(), 2.0)

        # Lidar
        sensor = Sensor()
        sensor.set_type(sdf.Sensortype.LIDAR)

        ldr = Lidar()
        sensor.set_lidar_sensor(ldr)

        ldrMutable = sensor.lidar_sensor()
        self.assertNotEqual(None, ldrMutable)
        self.assertAlmostEqual(ldrMutable.lidar_noise().mean(),
        sensor.lidar_sensor().lidar_noise().mean())

        noise = Noise()
        noise.set_mean(2.0)
        ldrMutable.set_lidar_noise(noise)
        self.assertAlmostEqual(ldrMutable.lidar_noise().mean(), 2.0)
        self.assertAlmostEqual(
        sensor.lidar_sensor().lidar_noise().mean(), 2.0)

        # Magnetometer
        sensor = Sensor()
        sensor.set_type(sdf.Sensortype.MAGNETOMETER)

        mag = Magnetometer()
        sensor.set_magnetometer_sensor(mag)

        magMutable = sensor.magnetometer_sensor()
        self.assertNotEqual(None, magMutable)
        self.assertAlmostEqual(magMutable.x_noise().mean(),
        sensor.magnetometer_sensor().x_noise().mean())

        noise = Noise()
        noise.set_mean(2.0)
        magMutable.set_x_noise(noise)
        self.assertAlmostEqual(magMutable.x_noise().mean(), 2.0)
        self.assertAlmostEqual(sensor.magnetometer_sensor().x_noise().mean(), 2.0)

        # NavSat
        sensor = Sensor()
        sensor.set_type(sdf.Sensortype.NAVSAT)

        nav = NavSat()
        sensor.set_nav_sat_sensor(nav)

        navMutable = sensor.nav_sat_sensor()
        self.assertNotEqual(None, navMutable)
        self.assertAlmostEqual(navMutable.horizontal_position_noise().mean(),
        sensor.nav_sat_sensor().horizontal_position_noise().mean())

        noise = Noise()
        noise.set_mean(2.0)
        navMutable.set_horizontal_position_noise(noise)
        self.assertAlmostEqual(
            navMutable.horizontal_position_noise().mean(),
            2.0)
        self.assertAlmostEqual(
            sensor.nav_sat_sensor().horizontal_position_noise().mean(), 2.0)

    def test_plugins(self):
        sensor = Sensor()
        self.assertEqual(0, len(sensor.plugins()))

        plugin = Plugin()
        plugin.set_name("name1")
        plugin.set_filename("filename1")

        sensor.add_plugin(plugin)
        self.assertEqual(1, len(sensor.plugins()))

        plugin.set_name("name2")
        sensor.add_plugin(plugin)
        self.assertEqual(2, len(sensor.plugins()))

        self.assertEqual("name1", sensor.plugins()[0].name())
        self.assertEqual("name2", sensor.plugins()[1].name())

        sensor.clear_plugins()
        self.assertEqual(0, len(sensor.plugins()))


if __name__ == '__main__':
    unittest.main()
