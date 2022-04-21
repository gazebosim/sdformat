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
from ignition.math import Pose3d, Vector3d
from sdformat import Joint, JointAxis, Error, SemanticPose
import math
import unittest

class JointTEST(unittest.TestCase):

    def test_default_construction(self):
        joint = Joint()
        self.assertFalse(joint.name())
        self.assertEqual(Joint.JointType.INVALID, joint.type())
        self.assertFalse(joint.parent_link_name())
        self.assertFalse(joint.child_link_name())
        self.assertEqual(Pose3d.ZERO, joint.raw_pose())
        self.assertFalse(joint.pose_relative_to())

        semanticPose = joint.semantic_pose()
        self.assertEqual(Pose3d.ZERO, semanticPose.raw_pose())
        self.assertFalse(semanticPose.relative_to())
        pose = Pose3d()
        # expect errors when trying to resolve pose without graph
        self.assertTrue(semanticPose.resolve(pose))

        joint.set_raw_pose(Pose3d(-1, -2, -3, math.pi, math.pi, 0))
        self.assertEqual(Pose3d(-1, -2, -3, math.pi, math.pi, 0),
                joint.raw_pose())

        joint.set_pose_relative_to("link")
        self.assertEqual("link", joint.pose_relative_to())

        semanticPose = joint.semantic_pose()
        self.assertEqual(joint.raw_pose(), semanticPose.raw_pose())
        self.assertEqual("link", semanticPose.relative_to())
        pose = Pose3d()
        # expect errors when trying to resolve pose without graph
        self.assertTrue(semanticPose.resolve(pose))

        joint.set_name("test_joint")
        self.assertEqual("test_joint", joint.name())

        joint.set_parent_link_name("parent")
        self.assertEqual("parent", joint.parent_link_name())

        joint.set_child_link_name("child")
        self.assertEqual("child", joint.child_link_name())

        body = ""
        resolveChildLink = joint.resolve_child_link(body)
        self.assertEqual(1, len(resolveChildLink[0]))
        self.assertFalse(resolveChildLink[1])
        resolveParentLink = joint.resolve_parent_link(body)
        self.assertEqual(1, len(resolveParentLink[0]))
        self.assertFalse(resolveParentLink[1])

        joint.set_type(Joint.JointType.BALL)
        self.assertEqual(Joint.JointType.BALL, joint.type())
        joint.set_type(Joint.JointType.CONTINUOUS)
        self.assertEqual(Joint.JointType.CONTINUOUS, joint.type())
        joint.set_type(Joint.JointType.GEARBOX)
        self.assertEqual(Joint.JointType.GEARBOX, joint.type())
        joint.set_type(Joint.JointType.PRISMATIC)
        self.assertEqual(Joint.JointType.PRISMATIC, joint.type())
        joint.set_type(Joint.JointType.REVOLUTE)
        self.assertEqual(Joint.JointType.REVOLUTE, joint.type())
        joint.set_type(Joint.JointType.REVOLUTE2)
        self.assertEqual(Joint.JointType.REVOLUTE2, joint.type())
        joint.set_type(Joint.JointType.SCREW)
        self.assertEqual(Joint.JointType.SCREW, joint.type())
        joint.set_type(Joint.JointType.UNIVERSAL)
        self.assertEqual(Joint.JointType.UNIVERSAL, joint.type())

        self.assertEqual(None, joint.axis(0))
        self.assertEqual(None, joint.axis(1))
        axis = JointAxis()
        self.assertEqual(0, len(axis.set_xyz(Vector3d(1, 0, 0))))
        joint.set_axis(0, axis)
        axis1 = JointAxis()
        self.assertEqual(0, len(axis1.set_xyz(Vector3d(0, 1, 0))))
        joint.set_axis(1, axis1)
        self.assertNotEqual(None, joint.axis(0))
        self.assertNotEqual(None, joint.axis(1))
        self.assertEqual(axis.xyz(), joint.axis(0).xyz())
        self.assertEqual(axis1.xyz(), joint.axis(1).xyz())

        self.assertAlmostEqual(1.0, joint.thread_pitch())
        threadPitch = 0.1
        joint.set_thread_pitch(threadPitch)
        self.assertAlmostEqual(threadPitch, joint.thread_pitch())

        # TODO(ahcorde): Add sensor when sdf::sensors class is converted
        # self.assertEqual(0u, joint.SensorCount())
        # self.assertEqual(None, joint.SensorByIndex(0))
        # self.assertEqual(None, joint.SensorByIndex(1))
        # self.assertEqual(None, joint.SensorByName("empty"))
        # self.assertFalse(joint.SensorNameExists(""))
        # self.assertFalse(joint.SensorNameExists("default"))


    def test_copy_construction(self):
        joint = Joint()
        joint.set_name("test_joint")
        axis = JointAxis()
        self.assertEqual(0, len(axis.set_xyz(Vector3d(1, 0, 0))))
        joint.set_axis(0, axis)
        axis1 = JointAxis()
        self.assertEqual(0, len(axis1.set_xyz(Vector3d(0, 1, 0))))
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
        self.assertEqual(0, len(axis.set_xyz(Vector3d(1, 0, 0))))
        joint.set_axis(0, axis)
        axis1 = JointAxis()
        self.assertEqual(0, len(axis1.set_xyz(Vector3d(0, 1, 0))))
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

if __name__ == '__main__':
    unittest.main()
