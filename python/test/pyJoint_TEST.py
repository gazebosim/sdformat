# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
from gz.math import Pose3d, Vector3d
from sdformat import (Joint, JointAxis, Error, SemanticPose,
                                   Sensor, SDFErrorsException)
import sdformat as sdf
import math
import unittest


class JointTEST(unittest.TestCase):

    def test_default_construction(self):
        joint = Joint()
        self.assertFalse(joint.name())
        self.assertEqual(sdf.JointType.INVALID, joint.type())
        self.assertFalse(joint.parent_name())
        self.assertFalse(joint.child_name())
        self.assertEqual(Pose3d.ZERO, joint.raw_pose())
        self.assertFalse(joint.pose_relative_to())

        semanticPose = joint.semantic_pose()
        self.assertEqual(Pose3d.ZERO, semanticPose.raw_pose())
        self.assertFalse(semanticPose.relative_to())
        # expect errors when trying to resolve pose without graph
        with self.assertRaises(SDFErrorsException):
            semanticPose.resolve()

        joint.set_raw_pose(Pose3d(-1, -2, -3, math.pi, math.pi, 0))
        self.assertEqual(Pose3d(-1, -2, -3, math.pi, math.pi, 0),
                joint.raw_pose())

        joint.set_pose_relative_to("link")
        self.assertEqual("link", joint.pose_relative_to())

        semanticPose = joint.semantic_pose()
        self.assertEqual(joint.raw_pose(), semanticPose.raw_pose())
        self.assertEqual("link", semanticPose.relative_to())
        # expect errors when trying to resolve pose without graph
        with self.assertRaises(SDFErrorsException):
            semanticPose.resolve()

        joint.set_name("test_joint")
        self.assertEqual("test_joint", joint.name())

        joint.set_parent_name("parent")
        self.assertEqual("parent", joint.parent_name())

        joint.set_child_name("child")
        self.assertEqual("child", joint.child_name())

        with self.assertRaises(SDFErrorsException):
            joint.resolve_child_link()

        with self.assertRaises(SDFErrorsException):
            joint.resolve_parent_link()

        joint.set_type(sdf.JointType.BALL)
        self.assertEqual(sdf.JointType.BALL, joint.type())
        joint.set_type(sdf.JointType.CONTINUOUS)
        self.assertEqual(sdf.JointType.CONTINUOUS, joint.type())
        joint.set_type(sdf.JointType.GEARBOX)
        self.assertEqual(sdf.JointType.GEARBOX, joint.type())
        joint.set_type(sdf.JointType.PRISMATIC)
        self.assertEqual(sdf.JointType.PRISMATIC, joint.type())
        joint.set_type(sdf.JointType.REVOLUTE)
        self.assertEqual(sdf.JointType.REVOLUTE, joint.type())
        joint.set_type(sdf.JointType.REVOLUTE2)
        self.assertEqual(sdf.JointType.REVOLUTE2, joint.type())
        joint.set_type(sdf.JointType.SCREW)
        self.assertEqual(sdf.JointType.SCREW, joint.type())
        joint.set_type(sdf.JointType.UNIVERSAL)
        self.assertEqual(sdf.JointType.UNIVERSAL, joint.type())

        self.assertEqual(None, joint.axis(0))
        self.assertEqual(None, joint.axis(1))
        axis = JointAxis()
        axis.set_xyz(Vector3d(1, 0, 0))
        joint.set_axis(0, axis)
        axis1 = JointAxis()
        axis1.set_xyz(Vector3d(0, 1, 0))
        joint.set_axis(1, axis1)
        self.assertNotEqual(None, joint.axis(0))
        self.assertNotEqual(None, joint.axis(1))
        self.assertEqual(axis.xyz(), joint.axis(0).xyz())
        self.assertEqual(axis1.xyz(), joint.axis(1).xyz())

        # Default thread pitch
        self.assertAlmostEqual(1.0, joint.screw_thread_pitch())
        self.assertAlmostEqual(-2*math.pi, joint.thread_pitch())
        threadPitch = 0.1
        joint.set_screw_thread_pitch(threadPitch)
        self.assertAlmostEqual(threadPitch, joint.screw_thread_pitch())
        self.assertAlmostEqual(-2*math.pi / threadPitch, joint.thread_pitch())
        joint.set_thread_pitch(threadPitch)
        self.assertAlmostEqual(threadPitch, joint.thread_pitch())
        self.assertAlmostEqual(-2*math.pi / threadPitch, joint.screw_thread_pitch())

        self.assertEqual(0, joint.sensor_count())
        self.assertEqual(None, joint.sensor_by_index(0))
        self.assertEqual(None, joint.sensor_by_index(1))
        self.assertEqual(None, joint.sensor_by_name("empty"))
        self.assertFalse(joint.sensor_name_exists(""))
        self.assertFalse(joint.sensor_name_exists("default"))

    def test_copy_construction(self):
        joint = Joint()
        joint.set_name("test_joint")
        axis = JointAxis()
        axis.set_xyz(Vector3d(1, 0, 0))
        joint.set_axis(0, axis)
        axis1 = JointAxis()
        axis1.set_xyz(Vector3d(0, 1, 0))
        joint.set_axis(1, axis1)

        joint2 = Joint(joint)

        self.assertEqual("test_joint", joint.name())
        self.assertIsNotNone(joint.axis(0))
        self.assertIsNotNone(joint.axis(1))
        self.assertEqual(axis.xyz(), joint.axis(0).xyz())
        self.assertEqual(axis1.xyz(), joint.axis(1).xyz())

        self.assertEqual("test_joint", joint2.name())
        self.assertIsNotNone(joint2.axis(0))
        self.assertIsNotNone(joint2.axis(1))
        self.assertEqual(axis.xyz(), joint2.axis(0).xyz())
        self.assertEqual(axis1.xyz(), joint2.axis(1).xyz())


    def test_deepcopy(self):
        joint = Joint()
        joint.set_name("test_joint")
        axis = JointAxis()
        axis.set_xyz(Vector3d(1, 0, 0))
        joint.set_axis(0, axis)
        axis1 = JointAxis()
        axis1.set_xyz(Vector3d(0, 1, 0))
        joint.set_axis(1, axis1)

        joint2 = copy.deepcopy(joint)

        self.assertEqual("test_joint", joint.name())
        self.assertIsNotNone(joint.axis(0))
        self.assertIsNotNone(joint.axis(1))
        self.assertEqual(axis.xyz(), joint.axis(0).xyz())
        self.assertEqual(axis1.xyz(), joint.axis(1).xyz())

        self.assertEqual("test_joint", joint2.name())
        self.assertIsNotNone(joint2.axis(0))
        self.assertIsNotNone(joint2.axis(1))
        self.assertEqual(axis.xyz(), joint2.axis(0).xyz())
        self.assertEqual(axis1.xyz(), joint2.axis(1).xyz())

    def test_mutable_by_index(self):
        joint = Joint()

        sensor = Sensor()
        sensor.set_name("sensor1")
        self.assertTrue(joint.add_sensor(sensor))

        # Modify the sensor
        s = joint.sensor_by_index(0)
        self.assertNotEqual(None, s)
        self.assertEqual("sensor1", s.name())
        s.set_name("sensor2")
        self.assertEqual("sensor2", joint.sensor_by_index(0).name())

    def test_mutable_by_name(self):
        joint = Joint()

        sensor = Sensor()
        sensor.set_name("sensor1")
        self.assertTrue(joint.add_sensor(sensor))

        # Modify the sensor
        s = joint.sensor_by_name("sensor1")
        self.assertNotEqual(None, s)
        self.assertEqual("sensor1", s.name())
        s.set_name("sensor2")
        self.assertEqual("sensor2", joint.sensor_by_name("sensor2").name())


if __name__ == '__main__':
    unittest.main()
