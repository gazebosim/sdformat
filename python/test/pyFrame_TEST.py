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
from ignition.math import Pose3d
from sdformat import Frame, Error
import unittest
import math

class FrameColor(unittest.TestCase):

  def test_default_construction(self):
    frame = Frame()
    self.assertFalse(frame.name())

    frame.set_name("test_frame")
    self.assertEqual(frame.name(), "test_frame")

    self.assertFalse(frame.attached_to())
    self.assertEqual(Pose3d.ZERO, frame.raw_pose())
    self.assertFalse(frame.pose_relative_to())

    semanticPose = frame.semantic_pose()
    self.assertEqual(Pose3d.ZERO, semanticPose.raw_pose())
    self.assertFalse(semanticPose.relative_to())
    pose = Pose3d()
    # expect errors when trying to resolve pose
    self.assertTrue(semanticPose.resolve(pose))

    frame.set_attached_to("attachment")
    self.assertEqual("attachment", frame.attached_to())

    frame.set_raw_pose(Pose3d(-10, -20, -30, math.pi, math.pi, math.pi))
    self.assertEqual(Pose3d(-10, -20, -30, math.pi, math.pi, math.pi),
        frame.raw_pose())

    semanticPose = frame.semantic_pose()
    self.assertEqual(frame.raw_pose(), semanticPose.raw_pose())
    self.assertEqual("attachment", semanticPose.relative_to())
    pose = Pose3d()
    # expect errors when trying to resolve pose
    self.assertTrue(semanticPose.resolve(pose))

    frame.set_pose_relative_to("link")
    self.assertEqual("link", frame.pose_relative_to())

    semanticPose = frame.semantic_pose()
    self.assertEqual(frame.raw_pose(), semanticPose.raw_pose())
    self.assertEqual("link", semanticPose.relative_to())
    pose = Pose3d()
    # expect errors when trying to resolve pose
    self.assertEqual(1, len(semanticPose.resolve(pose)))

    body = ""
    self.assertEqual(1, len(frame.resolve_attached_to_body(body)))
    self.assertFalse(body)


if __name__ == '__main__':
    unittest.main()
