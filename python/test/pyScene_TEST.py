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
from gz.math import Color
from sdformat import Scene, Sky
import unittest


class SceneTEST(unittest.TestCase):

    def test_default_construction(self):
        scene = Scene()
        self.assertEqual(Color(0.4, 0.4, 0.4), scene.ambient())
        self.assertEqual(Color(0.7, 0.7, 0.7), scene.background())
        self.assertTrue(scene.grid())
        self.assertTrue(scene.shadows())
        self.assertTrue(scene.origin_visual())
        self.assertEqual(None, scene.sky())


    def test_copy_construction(self):
        scene = Scene()
        scene.set_ambient(Color.BLUE)
        scene.set_background(Color.RED)
        scene.set_grid(False)
        scene.set_shadows(False)
        scene.set_origin_visual(False)
        sky = Sky()
        scene.set_sky(sky)

        scene2 = Scene(scene)
        self.assertEqual(Color.BLUE, scene2.ambient())
        self.assertEqual(Color.RED, scene2.background())
        self.assertFalse(scene2.grid())
        self.assertFalse(scene2.shadows())
        self.assertFalse(scene2.origin_visual())
        self.assertNotEqual(None, scene2.sky())

    def test_assignment(self):
        scene = Scene()
        scene.set_ambient(Color.RED)
        scene.set_background(Color(0.2, 0.3, 0.4))
        scene.set_grid(False)
        scene.set_shadows(False)
        scene.set_origin_visual(False)
        sky = Sky()
        scene.set_sky(sky)

        scene2 = scene
        self.assertEqual(Color.RED, scene2.ambient())
        self.assertEqual(Color(0.2, 0.3, 0.4), scene2.background())
        self.assertFalse(scene2.grid())
        self.assertFalse(scene2.shadows())
        self.assertFalse(scene2.origin_visual())
        self.assertNotEqual(None, scene2.sky())

    def test_deepcopy(self):
        scene = Scene()
        scene.set_ambient(Color.RED)
        scene.set_background(Color(0.2, 0.3, 0.4))
        scene.set_grid(False)
        scene.set_shadows(False)
        scene.set_origin_visual(False)
        sky = Sky()
        scene.set_sky(sky)

        scene2 = copy.deepcopy(scene)
        self.assertEqual(Color.RED, scene2.ambient())
        self.assertEqual(Color(0.2, 0.3, 0.4), scene2.background())
        self.assertFalse(scene2.grid())
        self.assertFalse(scene2.shadows())
        self.assertFalse(scene2.origin_visual())
        self.assertNotEqual(None, scene2.sky())

    def test_set(self):
        scene = Scene()
        scene.set_ambient(Color(0.1, 0.2, 0.3))
        self.assertEqual(Color(0.1, 0.2, 0.3), scene.ambient())

        scene.set_background(Color(0.2, 0.3, 0.4))
        self.assertEqual(Color(0.2, 0.3, 0.4), scene.background())

        scene.set_grid(True)
        self.assertTrue(scene.grid())
        scene.set_grid(False)
        self.assertFalse(scene.grid())

        scene.set_shadows(True)
        self.assertTrue(scene.shadows())
        scene.set_shadows(False)
        self.assertFalse(scene.shadows())

        scene.set_origin_visual(True)
        self.assertTrue(scene.origin_visual())
        scene.set_origin_visual(False)
        self.assertFalse(scene.origin_visual())

        sky = Sky()
        scene.set_sky(sky)
        self.assertNotEqual(None, scene.sky())

if __name__ == '__main__':
    unittest.main()
