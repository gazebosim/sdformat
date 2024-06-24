# Copyright (C) 2023 Open Source Robotics Foundation

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
from gz_test_deps.sdformat import Plugin, Projector
from gz_test_deps.math import Angle, Pose3d
import unittest


class ProjectorTEST(unittest.TestCase):

    def test_default_construction(self):
        projector = Projector()

        self.assertEqual(0, len(projector.name()))

        projector.set_name("test_projector")
        self.assertEqual(projector.name(), "test_projector")

        self.assertAlmostEqual(0.1, projector.near_clip())
        projector.set_near_clip(2.0)
        self.assertAlmostEqual(2.0, projector.near_clip())

        self.assertAlmostEqual(10.0, projector.far_clip())
        projector.set_far_clip(20.0)
        self.assertAlmostEqual(20.0, projector.far_clip())

        self.assertEqual(Angle(0.785), projector.horizontal_fov())
        projector.set_horizontal_fov(Angle(3.1416 * 0.5))
        self.assertEqual(Angle(3.1416  * 0.5), projector.horizontal_fov())

        self.assertEqual(4294967295, projector.visibility_flags())
        projector.set_visibility_flags(0x03)
        self.assertEqual(0x03, projector.visibility_flags())

        self.assertEqual(0, len(projector.texture()))
        projector.set_texture("texture.png")
        self.assertEqual("texture.png", projector.texture())

        self.assertEqual(Pose3d.ZERO, projector.raw_pose())
        projector.set_raw_pose(Pose3d(1, 2, 3, 0, 0, 1.5707))
        self.assertEqual(Pose3d(1, 2, 3, 0, 0, 1.5707), projector.raw_pose())

        self.assertEqual(0, len(projector.pose_relative_to()))
        projector.set_pose_relative_to("/test/relative")
        self.assertEqual("/test/relative", projector.pose_relative_to())

        self.assertEqual(0, len(projector.file_path()))
        projector.set_file_path("/test/path")
        self.assertEqual("/test/path", projector.file_path())

        self.assertEqual(0, len(projector.plugins()))
        plugin = Plugin()
        plugin.set_name("name1")
        plugin.set_filename("filename1")

        projector.add_plugin(plugin)
        self.assertEqual(1, len(projector.plugins()))

        plugin.set_name("name2")
        projector.add_plugin(plugin)
        self.assertEqual(2, len(projector.plugins()))

        self.assertEqual("name1", projector.plugins()[0].name())
        self.assertEqual("name2", projector.plugins()[1].name())

        projector.clear_plugins()
        self.assertEqual(0, len(projector.plugins()))
