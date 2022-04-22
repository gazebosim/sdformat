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
from ignition.math import Vector3d, Ellipsoidd
import math
from sdformat import Ellipsoid
import unittest


class BoxTEST(unittest.TestCase):

  def test_default_construction(self):
    ellipsoid = Ellipsoid()
    # A default ellipsoid has all three radii set to 1
    self.assertEqual(math.pi * 4. / 3., ellipsoid.shape().volume())
    self.assertEqual(Vector3d.ONE, ellipsoid.shape().radii())

    expectedradii = Vector3d(1.0, 2.0, 3.0)
    ellipsoid.set_radii(expectedradii)
    self.assertEqual(expectedradii, ellipsoid.shape().radii())


  def test_assignment(self):
    ellipsoid = Ellipsoid()
    expectedradii = Vector3d(1.0, 2.0, 3.0)
    ellipsoid.set_radii(expectedradii)

    ellipsoid2 = ellipsoid
    self.assertEqual(expectedradii, ellipsoid2.shape().radii())


  def test_deepcopy(self):
    ellipsoid = Ellipsoid()
    expectedradii = Vector3d(1.0, 2.0, 3.0)
    ellipsoid.set_radii(expectedradii)

    ellipsoid2 = copy.deepcopy(ellipsoid)
    self.assertEqual(expectedradii, ellipsoid2.shape().radii())


  def test_deepcopy_after_assignment(self):
      ellipsoid1 = Ellipsoid();
      expectedradii1 = Vector3d(1.0, 2.0, 3.0)
      ellipsoid1.set_radii(expectedradii1)

      ellipsoid2 = Ellipsoid()
      expectedradii2 = Vector3d(10.0, 20.0, 30.0)
      ellipsoid2.set_radii(expectedradii2)

      # This is similar to what std::swap does except it uses std::move for each
      # assignment
      tmp = copy.deepcopy(ellipsoid1)
      ellipsoid1 = ellipsoid2;
      ellipsoid2 = tmp;

      self.assertEqual(expectedradii1, ellipsoid2.shape().radii())
      self.assertEqual(expectedradii2, ellipsoid1.shape().radii())


  def test_shape(self):
    ellipsoid = Ellipsoid()
    self.assertEqual(Vector3d.ONE, ellipsoid.radii())

    expectedradii = Vector3d(1.0, 2.0, 3.0)
    ellipsoid.shape().set_radii(expectedradii)
    self.assertEqual(expectedradii, ellipsoid.radii())


if __name__ == '__main__':
    unittest.main()
