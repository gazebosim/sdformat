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
from sdformat import Polyline
from gz.math import Vector2d
import unittest


class PolylineTEST(unittest.TestCase):

    def test_default_construction(self):
        polyline = Polyline()

        self.assertEqual(0, polyline.point_count())
        self.assertEqual(0, len(polyline.points()))
        self.assertAlmostEqual(1.0, polyline.height())

    def test_copy_construction(self):
        polyline = Polyline()
        polyline.set_height(0.2)

        polyline2 = Polyline(polyline)
        self.assertAlmostEqual(0.2, polyline2.height())

    def test_copy_assignment(self):
        polyline = Polyline()
        polyline.set_height(0.2)

        polyline2 = polyline
        self.assertAlmostEqual(0.2, polyline2.height())

    def test_point(self):
        polyline = Polyline()
        self.assertEqual(0, polyline.point_count())

        p1 = Vector2d(1, 2)
        p2 = Vector2d(3, 4)
        p3 = Vector2d(5, 6)
        p4 = Vector2d(7, 8)

        self.assertTrue(polyline.add_point(p1))
        self.assertTrue(polyline.add_point(p2))
        self.assertTrue(polyline.add_point(p3))

        self.assertEqual(3, polyline.point_count())
        self.assertEqual(3, len(polyline.points()))
        self.assertEqual(p1, polyline.point_by_index(0))
        self.assertEqual(p2, polyline.point_by_index(1))
        self.assertEqual(p3, polyline.point_by_index(2))
        self.assertEqual(p1, polyline.points()[0])
        self.assertEqual(p2, polyline.points()[1])
        self.assertEqual(p3, polyline.points()[2])
        self.assertEqual(None, polyline.point_by_index(3))

        polyline.clear_points()
        self.assertEqual(0, polyline.point_count())


if __name__ == '__main__':
    unittest.main()
