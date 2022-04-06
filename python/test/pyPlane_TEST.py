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
from ignition.math import Vector3d, Vector2d, Planed
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


  def test_move_construction(self):
    plane = Plane()
    plane.set_normal(Vector3d(1, 0, 0))
    plane.set_size(Vector2d(1.2, 3.4))

    plane2 = plane
    self.assertEqual(Vector3d.UNIT_X, plane2.normal())
    self.assertEqual(Vector2d(1.2, 3.4), plane2.size())

    self.assertEqual(Vector3d.UNIT_X, plane2.shape().normal())
    self.assertEqual(Vector2d(1.2, 3.4), plane2.shape().size())


  def test_copy_construction(self):
    plane = Plane()
    plane.set_normal(Vector3d(1, 0, 0))
    plane.set_size(Vector2d(1.2, 3.4))

    plane2 = copy.deepcopy(plane)
    self.assertEqual(Vector3d.UNIT_X, plane2.normal())
    self.assertEqual(Vector2d(1.2, 3.4), plane2.size())


  def test_assignment_after_move(self):
    plane1 = Plane()
    plane1.set_normal(Vector3d.UNIT_X)

    plane2 = Plane()
    plane2.set_normal(Vector3d.UNIT_Y)

    # This is similar to what std::swap does except it uses std::move for each
    # assignment
    tmp = copy.deepcopy(plane1)
    plane1 = plane2
    plane2 = tmp

    self.assertEqual(Vector3d.UNIT_Y, plane1.normal())
    self.assertEqual(Vector3d.UNIT_X, plane2.normal())


  def test_shape(self):

    plane = Plane()
    self.assertEqual(Vector2d.ONE, plane.size())

    plane.shape().set(plane.shape().normal(), Vector2d(1, 2),
      plane.shape().offset())
    self.assertEqual(Vector2d(1, 2), plane.size())


if __name__ == '__main__':
    unittest.main()
