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
from gz.math import Vector3d
from sdformat import Heightmap, HeightmapBlend, HeightmapTexture
import unittest


class HeightmapTEST(unittest.TestCase):

    def test_default_construction(self):
        heightmap = Heightmap()

        self.assertEqual('', heightmap.file_path())
        self.assertEqual('', heightmap.uri())
        self.assertEqual(Vector3d(1, 1, 1), heightmap.size())
        self.assertEqual(Vector3d.ZERO, heightmap.position())
        self.assertFalse(heightmap.use_terrain_paging())
        self.assertEqual(1, heightmap.sampling())
        self.assertEqual(0, heightmap.texture_count())
        self.assertEqual(0, heightmap.blend_count())
        self.assertEqual(None, heightmap.texture_by_index(0))
        self.assertEqual(None, heightmap.blend_by_index(0))

        heightmapTexture = HeightmapTexture()

        self.assertAlmostEqual(10.0, heightmapTexture.size())
        self.assertFalse(heightmapTexture.diffuse())
        self.assertFalse(heightmapTexture.normal())

        heightmapBlend = HeightmapBlend()

        self.assertAlmostEqual(0.0, heightmapBlend.min_height())
        self.assertAlmostEqual(0.0, heightmapBlend.fade_distance())

    def test_copy_construction(self):
        heightmap = Heightmap()
        heightmap.set_uri("banana")
        heightmap.set_file_path("/pear")
        heightmap.set_size(Vector3d(0.1, 0.2, 0.3))
        heightmap.set_position(Vector3d(0.5, 0.6, 0.7))
        heightmap.set_use_terrain_paging(True)
        heightmap.set_sampling(123)

        heightmap2 = Heightmap(heightmap)
        self.assertEqual("banana", heightmap2.uri())
        self.assertEqual("/pear", heightmap2.file_path())
        self.assertEqual(Vector3d(0.1, 0.2, 0.3), heightmap2.size())
        self.assertEqual(Vector3d(0.5, 0.6, 0.7), heightmap2.position())
        self.assertTrue(heightmap2.use_terrain_paging())
        self.assertEqual(123, heightmap2.sampling())

        heightmapTexture = HeightmapTexture()
        heightmapTexture.set_size(123.456)
        heightmapTexture.set_diffuse("diffuse")
        heightmapTexture.set_normal("normal")

        heightmapTexture2 = HeightmapTexture(heightmapTexture)
        self.assertAlmostEqual(123.456, heightmapTexture2.size())
        self.assertEqual("diffuse", heightmapTexture2.diffuse())
        self.assertEqual("normal", heightmapTexture2.normal())

        heightmapBlend = HeightmapBlend()
        heightmapBlend.set_min_height(123.456)
        heightmapBlend.set_fade_distance(456.123)

        heightmapBlend2 = HeightmapBlend(heightmapBlend)
        self.assertAlmostEqual(123.456, heightmapBlend2.min_height())
        self.assertAlmostEqual(456.123, heightmapBlend2.fade_distance())


    def test_deepcopy(self):
        heightmap = Heightmap()
        heightmap.set_uri("banana")
        heightmap.set_file_path("/pear")
        heightmap.set_size(Vector3d(0.1, 0.2, 0.3))
        heightmap.set_position(Vector3d(0.5, 0.6, 0.7))
        heightmap.set_use_terrain_paging(True)
        heightmap.set_sampling(123)

        heightmap2 = copy.deepcopy(heightmap)
        self.assertEqual("banana", heightmap2.uri())
        self.assertEqual("/pear", heightmap2.file_path())
        self.assertEqual(Vector3d(0.1, 0.2, 0.3), heightmap2.size())
        self.assertEqual(Vector3d(0.5, 0.6, 0.7), heightmap2.position())
        self.assertTrue(heightmap2.use_terrain_paging())
        self.assertEqual(123, heightmap2.sampling())

        heightmapTexture = HeightmapTexture()
        heightmapTexture.set_size(123.456)
        heightmapTexture.set_diffuse("diffuse")
        heightmapTexture.set_normal("normal")

        heightmapTexture2 = copy.deepcopy(heightmapTexture)
        self.assertAlmostEqual(123.456, heightmapTexture2.size())
        self.assertEqual("diffuse", heightmapTexture2.diffuse())
        self.assertEqual("normal", heightmapTexture2.normal())

        heightmapBlend = HeightmapBlend()
        heightmapBlend.set_min_height(123.456)
        heightmapBlend.set_fade_distance(456.123)

        heightmapBlend2 = copy.deepcopy(heightmapBlend)
        self.assertAlmostEqual(123.456, heightmapBlend2.min_height())
        self.assertAlmostEqual(456.123, heightmapBlend2.fade_distance())

    def test_set(self):
        heightmapTexture = HeightmapTexture()

        self.assertAlmostEqual(10.0, heightmapTexture.size())
        heightmapTexture.set_size(21.05)
        self.assertAlmostEqual(21.05, heightmapTexture.size())

        self.assertFalse(heightmapTexture.diffuse())
        heightmapTexture.set_diffuse("diffuse")
        self.assertEqual("diffuse", heightmapTexture.diffuse())

        self.assertFalse(heightmapTexture.normal())
        heightmapTexture.set_normal("normal")
        self.assertEqual("normal", heightmapTexture.normal())

        heightmapBlend = HeightmapBlend()

        self.assertAlmostEqual(0.0, heightmapBlend.min_height())
        heightmapBlend.set_min_height(21.05)
        self.assertAlmostEqual(21.05, heightmapBlend.min_height())

        self.assertAlmostEqual(0.0, heightmapBlend.fade_distance())
        heightmapBlend.set_fade_distance(21.05)
        self.assertAlmostEqual(21.05, heightmapBlend.fade_distance())

        heightmap = Heightmap()

        self.assertEqual('', heightmap.uri())
        heightmap.set_uri("http://myuri.com")
        self.assertEqual("http://myuri.com", heightmap.uri())

        self.assertEqual('', heightmap.file_path())
        heightmap.set_file_path("/mypath")
        self.assertEqual("/mypath", heightmap.file_path())

        self.assertEqual(Vector3d.ONE, heightmap.size())
        heightmap.set_size(Vector3d(0.2, 1.4, 7.8))
        self.assertEqual(Vector3d(0.2, 1.4, 7.8), heightmap.size())

        self.assertEqual(Vector3d.ZERO, heightmap.position())
        heightmap.set_position(Vector3d(0.2, 1.4, 7.8))
        self.assertEqual(Vector3d(0.2, 1.4, 7.8), heightmap.position())

        self.assertFalse(heightmap.use_terrain_paging())
        heightmap.set_use_terrain_paging(True)
        self.assertTrue(heightmap.use_terrain_paging())

        self.assertEqual(1, heightmap.sampling())
        heightmap.set_sampling(12)
        self.assertEqual(12, heightmap.sampling())

        self.assertEqual(0, heightmap.texture_count())
        heightmap.add_texture(heightmapTexture)
        self.assertEqual(1, heightmap.texture_count())
        heightmapTexture2 = heightmap.texture_by_index(0)
        self.assertAlmostEqual(heightmapTexture2.size(), heightmapTexture.size())
        self.assertEqual(heightmapTexture2.diffuse(), heightmapTexture.diffuse())
        self.assertEqual(heightmapTexture2.normal(), heightmapTexture.normal())

        self.assertEqual(0, heightmap.blend_count())
        heightmap.add_blend(heightmapBlend)
        self.assertEqual(1, heightmap.blend_count())
        heightmapBlend2 = heightmap.blend_by_index(0)
        self.assertAlmostEqual(heightmapBlend2.min_height(), heightmapBlend.min_height())
        self.assertAlmostEqual(heightmapBlend2.fade_distance(),
          heightmapBlend.fade_distance())

if __name__ == '__main__':
    unittest.main()
