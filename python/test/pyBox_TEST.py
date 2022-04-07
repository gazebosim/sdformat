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

from sdformat import Box
from ignition.math import Vector3d
import unittest

class BoxTEST(unittest.TestCase):

  def test_default_construction(self):
    box = Box()

    self.assertEqual(Vector3d.ONE, box.size())

    box.set_size(Vector3d.ZERO)
    self.assertEqual(Vector3d.ZERO, box.size())


  def test_assignment(self):
    size = Vector3d(1, 2, 3)

    box = Box()
    box.set_size(size)

    box2 = box;
    self.assertEqual(size, box2.size())

    self.assertEqual(1 * 2 * 3, box2.shape().volume())
    self.assertEqual(size, box2.shape().size())

  def test_copy_construction(self):
    size = Vector3d(0.1, 0.2, 0.3)

    box = Box()
    box.set_size(size)

    box2 = Box(box)
    self.assertEqual(size, box2.size())

  def test_shape(self):
    box = Box()
    self.assertEqual(Vector3d.ONE, box.size())

    box.shape().set_size(Vector3d(1, 2, 3))
    self.assertEqual(Vector3d(1, 2, 3), box.size())

if __name__ == '__main__':
    unittest.main()
