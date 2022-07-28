# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
from gz.math import Color, Pose3d, Vector3d, Helpers
from sdformat import ParticleEmitter
import unittest


class ParticleEmitterTEST(unittest.TestCase):


    def test_default_construction(self):
        emitter = ParticleEmitter()

        self.assertFalse(emitter.name())

        emitter.set_name("test_emitter")
        self.assertEqual(emitter.name(), "test_emitter")

        self.assertEqual("point", emitter.type_str())
        self.assertEqual(ParticleEmitter.ParticleEmitterType.POINT, emitter.type())
        self.assertTrue(emitter.set_type("box"))
        self.assertEqual("box", emitter.type_str())
        self.assertEqual(ParticleEmitter.ParticleEmitterType.BOX, emitter.type())
        emitter.set_type(ParticleEmitter.ParticleEmitterType.CYLINDER)
        self.assertEqual("cylinder", emitter.type_str())

        self.assertTrue(emitter.emitting())
        emitter.set_emitting(False)
        self.assertFalse(emitter.emitting())

        self.assertAlmostEqual(0.0, emitter.duration())
        emitter.set_duration(10.0)
        self.assertAlmostEqual(10.0, emitter.duration())

        self.assertAlmostEqual(5.0, emitter.lifetime())
        emitter.set_lifetime(22.0)
        self.assertAlmostEqual(22.0, emitter.lifetime())
        emitter.set_lifetime(-1.0)
        self.assertAlmostEqual(Helpers.MIN_D, emitter.lifetime())

        self.assertAlmostEqual(10.0, emitter.rate())
        emitter.set_rate(123.0)
        self.assertAlmostEqual(123.0, emitter.rate())
        emitter.set_rate(-123.0)
        self.assertAlmostEqual(0.0, emitter.rate())

        self.assertAlmostEqual(0.0, emitter.scale_rate())
        emitter.set_scale_rate(1.2)
        self.assertAlmostEqual(1.2, emitter.scale_rate())
        emitter.set_scale_rate(-1.2)
        self.assertAlmostEqual(0.0, emitter.scale_rate())

        self.assertAlmostEqual(1.0, emitter.min_velocity())
        emitter.set_min_velocity(12.4)
        self.assertAlmostEqual(12.4, emitter.min_velocity())
        emitter.set_min_velocity(-12.4)
        self.assertAlmostEqual(0.0, emitter.min_velocity())

        self.assertAlmostEqual(1.0, emitter.max_velocity())
        emitter.set_max_velocity(20.6)
        self.assertAlmostEqual(20.6, emitter.max_velocity())
        emitter.set_max_velocity(-12.4)
        self.assertAlmostEqual(0.0, emitter.max_velocity())

        self.assertEqual(Vector3d.ONE, emitter.size())
        emitter.set_size(Vector3d(3, 2, 1))
        self.assertEqual(Vector3d(3, 2, 1), emitter.size())
        emitter.set_size(Vector3d(-3, -2, -1))
        self.assertEqual(Vector3d(0, 0, 0), emitter.size())

        self.assertEqual(Vector3d.ONE, emitter.particle_size())
        emitter.set_particle_size(Vector3d(4, 5, 6))
        self.assertEqual(Vector3d(4, 5, 6), emitter.particle_size())
        emitter.set_particle_size(Vector3d(-4, -5, -6))
        self.assertEqual(Vector3d(0, 0, 0), emitter.particle_size())

        self.assertEqual(Color.WHITE, emitter.color_start())
        emitter.set_color_start(Color(0.1, 0.2, 0.3, 1.0))
        self.assertEqual(Color(0.1, 0.2, 0.3, 1.0),
          emitter.color_start())

        self.assertEqual(Color.WHITE, emitter.color_end())
        emitter.set_color_end(Color(0.4, 0.5, 0.6, 1.0))
        self.assertEqual(Color(0.4, 0.5, 0.6, 1.0), emitter.color_end())

        self.assertFalse(emitter.color_range_image())
        emitter.set_color_range_image("/test/string")
        self.assertEqual("/test/string", emitter.color_range_image())

        self.assertFalse(emitter.topic())
        emitter.set_topic("/test/topic")
        self.assertEqual("/test/topic", emitter.topic())

        self.assertAlmostEqual(0.65, emitter.scatter_ratio())
        emitter.set_scatter_ratio(0.5)
        self.assertAlmostEqual(0.5, emitter.scatter_ratio())

        self.assertEqual(Pose3d.ZERO, emitter.raw_pose())
        emitter.set_raw_pose(Pose3d(1, 2, 3, 0, 0, 1.5707))
        self.assertEqual(Pose3d(1, 2, 3, 0, 0, 1.5707), emitter.raw_pose())

        self.assertFalse(emitter.pose_relative_to())
        emitter.set_pose_relative_to("/test/relative")
        self.assertEqual("/test/relative", emitter.pose_relative_to())


if __name__ == '__main__':
    unittest.main()
