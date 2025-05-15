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
from sdformat import Plane
from gz.math import Vector3d, Vector2d
import unittest


class PlaneTEST(unittest.TestCase):

  def test_default_construction(self):
    plane = Plane()

    self.assertEqual(Vector3d.UNIT_Z, plane.normal())
    self.assertEqual(Vector2d.ONE, plane.size())

    plane.set_normal(Vector3d(1, 0, 0))
    self.assertEqual(Vector3d.UNIT_X, plane.normal())

    plane.set_normal(Vector3d(1, 0, 1))
    self.assertEqual(Vector3d(0.707107, 0, 0.707107), plane.normal())

    plane.set_size(Vector2d(1.2, 3.4))
    self.assertEqual(Vector2d(1.2, 3.4), plane.size())


  def test_assigment(self):
    plane = Plane()
    plane.set_normal(Vector3d(1, 0, 0))
    plane.set_size(Vector2d(1.2, 3.4))

    plane2 = plane
    self.assertEqual(Vector3d.UNIT_X, plane2.normal())
    self.assertEqual(Vector2d(1.2, 3.4), plane2.size())

    self.assertEqual(Vector3d.UNIT_X, plane2.shape().normal())
    self.assertEqual(Vector2d(1.2, 3.4), plane2.shape().size())

    plane.set_normal(Vector3d(0, 1, 0))
    plane.set_size(Vector2d(3.3, 2.4))

    self.assertEqual(Vector3d(0, 1, 0), plane2.normal())
    self.assertEqual(Vector2d(3.3, 2.4), plane2.size())

    self.assertEqual(Vector3d(0, 1, 0), plane.normal())
    self.assertEqual(Vector2d(3.3, 2.4), plane.size())

  def test_deepcopy_construction(self):
    plane = Plane()
    plane.set_normal(Vector3d(1, 0, 0))
    plane.set_size(Vector2d(1.2, 3.4))

    plane2 = copy.deepcopy(plane)
    self.assertEqual(Vector3d.UNIT_X, plane2.normal())
    self.assertEqual(Vector2d(1.2, 3.4), plane2.size())

    plane.set_normal(Vector3d(0, 1, 0))
    plane.set_size(Vector2d(2.1, 4.3))

    self.assertEqual(Vector3d.UNIT_X, plane2.normal())
    self.assertEqual(Vector2d(1.2, 3.4), plane2.size())
    self.assertEqual(Vector2d(2.1, 4.3), plane.size())
    self.assertEqual(Vector3d(0, 1, 0), plane.normal())


  def test_shape(self):

    plane = Plane()
    self.assertEqual(Vector2d.ONE, plane.size())

    plane.shape().set(plane.shape().normal(), Vector2d(1, 2),
      plane.shape().offset())
    self.assertEqual(Vector2d(1, 2), plane.size())


if __name__ == '__main__':
    unittest.main()
