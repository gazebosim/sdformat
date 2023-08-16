# Copyright (C) 2023 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from gz_test_deps.sdformat import PrintConfig, SDFErrorsException
import unittest


class PrintConfigTEST(unittest.TestCase):

    def test_construction(self):
        config = PrintConfig()
        self.assertFalse(config.rotation_in_degrees())
        self.assertFalse(config.rotation_snap_to_degrees())
        self.assertFalse(config.preserve_includes())

    def test_rotation_in_degrees(self):
        config = PrintConfig()
        self.assertFalse(config.rotation_in_degrees())

        config.set_rotation_in_degrees(True)
        self.assertTrue(config.rotation_in_degrees())

        config.set_rotation_in_degrees(False)
        self.assertFalse(config.rotation_in_degrees())

    def test_rotation_snap_to_degrees(self):
        config = PrintConfig()
        self.assertIsNone(config.rotation_snap_to_degrees())
        self.assertIsNone(config.rotation_snap_tolerance())

        self.assertTrue(config.set_rotation_snap_to_degrees(5, 0.01))
        self.assertEqual(5, config.rotation_snap_to_degrees())
        self.assertEqual(0.01, config.rotation_snap_tolerance())

        self.assertRaises(SDFErrorsException,
                          config.set_rotation_snap_to_degrees, 0, 0.01)
        self.assertRaises(SDFErrorsException,
                          config.set_rotation_snap_to_degrees, 360 + 1, 0.01)
        self.assertRaises(SDFErrorsException,
                          config.set_rotation_snap_to_degrees, 5, -1e6)
        self.assertRaises(SDFErrorsException,
                          config.set_rotation_snap_to_degrees, 5, 360 + 1e-6)

        self.assertRaises(SDFErrorsException,
                          config.set_rotation_snap_to_degrees, 5, 5 + 1e-6)
        self.assertTrue(config.set_rotation_snap_to_degrees(5, 5 - 1e-6))
        self.assertEqual(5, config.rotation_snap_to_degrees())
        self.assertEqual(5 - 1e-6, config.rotation_snap_tolerance())

    def test_compare(self):
        first = PrintConfig()
        second = PrintConfig()
        self.assertTrue(first == second)
        self.assertTrue(second == first)

        first.set_rotation_in_degrees(True)
        self.assertTrue(first.rotation_in_degrees())
        self.assertFalse(second.rotation_in_degrees())
        self.assertFalse(first == second)
        self.assertFalse(second == first)

        second.set_rotation_in_degrees(True)
        self.assertTrue(first == second)
        self.assertTrue(second == first)

        self.assertTrue(first.set_rotation_snap_to_degrees(5, 0.01))
        self.assertFalse(first == second)
        self.assertFalse(second == first)

        self.assertTrue(second.set_rotation_snap_to_degrees(5, 0.01))
        self.assertTrue(first == second)
        self.assertTrue(second == first)

    def test_preserve_includes(self):
        config = PrintConfig()
        self.assertFalse(config.preserve_includes())
        config.set_preserve_includes(True)
        self.assertTrue(config.preserve_includes())
        config.set_preserve_includes(False)
        self.assertFalse(config.preserve_includes())

    def test_out_precision(self):
        config = PrintConfig()
        config.set_out_precision(2)
        self.assertEqual(2, config.out_precision())
        config.set_out_precision(8)
        self.assertEqual(8, config.out_precision())


if __name__ == '__main__':
    unittest.main()
