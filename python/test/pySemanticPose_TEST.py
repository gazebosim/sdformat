# Copyright (C) 2022 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
from gz.math import Pose3d
from sdformat import Link, SemanticPose
import unittest

class semantic_poseTEST(unittest.TestCase):


    def test_default_construction(self):
        link = Link()
        rawPose = Pose3d(1, 0, 0, 0, 0, 0)
        link.set_raw_pose(rawPose)
        semPose = link.semantic_pose()
        self.assertEqual(rawPose, semPose.raw_pose())


    def test_copy_construction(self):
        link = Link()
        rawPose = Pose3d(1, 0, 0, 0, 0, 0)
        link.set_raw_pose(rawPose)
        semPose1 = link.semantic_pose()

        semPose2 = SemanticPose(semPose1)
        self.assertEqual(rawPose, semPose2.raw_pose())


    def test_deepcopy(self):
        link = Link()
        rawPose = Pose3d(1, 0, 0, 0, 0, 0)
        link.set_raw_pose(rawPose)
        semPose1 = link.semantic_pose()

        # Create another semantic_pose object from another Link
        link2 = Link()
        semPose2 = link2.semantic_pose()
        self.assertEqual(Pose3d.ZERO, semPose2.raw_pose())

        semPose2 = copy.deepcopy(semPose1)
        self.assertEqual(rawPose, semPose2.raw_pose())


if __name__ == '__main__':
    unittest.main()
