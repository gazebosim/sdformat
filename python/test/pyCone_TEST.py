# Copyright 2024 CogniPilot Foundation
# Copyright 2024 Open Source Robotics Foundation
# Copyright 2024 Rudis Laboratories

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

import math

from gz.math import AxisAlignedBox, Vector3d
from sdformat import Cone

import unittest


class ConeTEST(unittest.TestCase):

  def test_default_construction(self):
    cone = Cone()

    self.assertEqual(math.pi * math.pow(0.5, 2) * 1.0 / 3.0,
                     cone.shape().volume())

    self.assertEqual(0.5, cone.radius())
    self.assertEqual(1.0, cone.length())

    cone.set_radius(0.5)
    cone.set_length(2.3)

    self.assertEqual(0.5, cone.radius())
    self.assertEqual(2.3, cone.length())

  def test_assignment(self):
    cone = Cone()
    cone.set_radius(0.2)
    cone.set_length(3.0)
    self.assertEqual(math.pi * math.pow(0.2, 2) * 3.0 / 3.0,
                   cone.shape().volume())

    cone2 = cone
    self.assertEqual(0.2, cone2.radius())
    self.assertEqual(3.0, cone2.length())

    self.assertEqual(math.pi * math.pow(0.2, 2) * 3.0 / 3.0,
                   cone2.shape().volume())
    self.assertEqual(0.2, cone2.shape().radius())
    self.assertEqual(3.0, cone2.shape().length())

    cone.set_radius(2.0)
    cone.set_length(0.3)

    self.assertEqual(2.0, cone.radius())
    self.assertEqual(0.3, cone.length())
    self.assertEqual(2.0, cone2.radius())
    self.assertEqual(0.3, cone2.length())


  def test_copy_construction(self):
    cone = Cone();
    cone.set_radius(0.2)
    cone.set_length(3.0)

    cone2 = Cone(cone)
    self.assertEqual(0.2, cone2.radius())
    self.assertEqual(3.0, cone2.length())

    cone.set_radius(2.)
    cone.set_length(0.3)

    self.assertEqual(2, cone.radius())
    self.assertEqual(0.3, cone.length())
    self.assertEqual(0.2, cone2.radius())
    self.assertEqual(3.0, cone2.length())

  def test_deepcopy(self):
    cone = Cone();
    cone.set_radius(0.2)
    cone.set_length(3.0)

    cone2 = copy.deepcopy(cone);
    self.assertEqual(0.2, cone2.radius())
    self.assertEqual(3.0, cone2.length())

    cone.set_radius(2.)
    cone.set_length(0.3)

    self.assertEqual(2, cone.radius())
    self.assertEqual(0.3, cone.length())
    self.assertEqual(0.2, cone2.radius())
    self.assertEqual(3.0, cone2.length())

  def test_shape(self):
    cone = Cone();
    self.assertEqual(0.5, cone.radius())
    self.assertEqual(1.0, cone.length())

    cone.shape().set_radius(0.123)
    cone.shape().set_length(0.456)
    self.assertEqual(0.123, cone.radius())
    self.assertEqual(0.456, cone.length())

  def test_axis_aligned_box(self):
    cone = Cone()
    cone.set_radius(1.0)
    cone.set_length(2.0)

    self.assertEqual(
      AxisAlignedBox(Vector3d(-1, -1, -1), Vector3d(1, 1, 1)),
      cone.axis_aligned_box())


if __name__ == '__main__':
    unittest.main()
