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

from gz_test_deps.sdformat import Element, Param, SDFErrorsException
from gz_test_deps.math import Vector2i, Vector3d, Pose3d
import unittest


class ParamTEST(unittest.TestCase):

    def test_bool(self):
        bool_param = Param("key", "bool", "true", False, "description")
        value, rc = bool_param.get_bool()
        self.assertTrue(value)
        bool_param.set_bool(False)
        self.assertFalse(bool_param.get_bool()[0])

        str_param = Param("key", "string", "true", False, "description")

        self.assertTrue(str_param.get_bool()[0])

        str_param.set_string("False")
        self.assertFalse(str_param.get_bool()[0])

        str_param.set_string("1")
        self.assertTrue(str_param.get_bool()[0])

        str_param.set_string("0")
        value, rc = str_param.get_bool()
        self.assertFalse(str_param.get_bool()[0])

        str_param.set_string("True")
        self.assertTrue(str_param.get_bool()[0])

        str_param.set_string("TRUE")
        self.assertTrue(str_param.get_bool()[0])

        # Anything other than 1 or true is treated as a False value
        str_param.set_string("%")
        with self.assertRaises(SDFErrorsException):
            str_param.get_bool()

    # Test getting and setting a floating point value
    def test_double(self):
        double_param = Param("key", "double", "0.0", False, "description")

        self.assertEqual(double_param.get_double(), (0.0, True))

        double_param.set_double(1.0)
        self.assertEqual(double_param.get_double(), (1.0, True))

        sixteen_digits = 12345678.87654321
        double_param.set_double(sixteen_digits)
        self.assertEqual(double_param.get_double(), (sixteen_digits, True))

    # Test getting and setting a Vector3d
    def test_Vector3d(self):
        vector_param = Param("key", "vector3", "0 0 0", False, "description")
        value, rc = vector_param.get_vector3()
        self.assertTrue(rc)
        self.assertAlmostEqual(0.0, value.x())
        self.assertAlmostEqual(0.0, value.y())
        self.assertAlmostEqual(0.0, value.z())

        vector_param.set_vector3(Vector3d.ONE)
        value, rc = vector_param.get_vector3()
        self.assertTrue(rc)
        self.assertAlmostEqual(1.0, value.x())
        self.assertAlmostEqual(1.0, value.y())
        self.assertAlmostEqual(1.0, value.z())

    # Test getting and setting a Pose3d
    def test_pose3d(self):
        pose_param = Param("key", "pose", "0 0 0 0 0 0", False, "description")
        value, rc = pose_param.get_pose()
        self.assertAlmostEqual(0.0, value.pos().x())
        self.assertAlmostEqual(0.0, value.pos().y())
        self.assertAlmostEqual(0.0, value.pos().z())
        self.assertAlmostEqual(0.0, value.rot().euler().x())
        self.assertAlmostEqual(0.0, value.rot().euler().y())
        self.assertAlmostEqual(0.0, value.rot().euler().z())

        pose_param.set_pose(Pose3d(1, 2, 3, 0.1, 0.2, 0.3))
        value, rc = pose_param.get_pose()
        self.assertAlmostEqual(1.0, value.pos().x())
        self.assertAlmostEqual(2.0, value.pos().y())
        self.assertAlmostEqual(3.0, value.pos().z())
        self.assertAlmostEqual(0.1, value.rot().euler().x())
        self.assertAlmostEqual(0.2, value.rot().euler().y())
        self.assertAlmostEqual(0.3, value.rot().euler().z())

    # Test decimal number
    def test_set_from_string_decimals(self):
        param = Param("number", "double", "0.0", True)
        self.assertTrue(param.set_from_string("0.2345"))

    # Test setting and reading uint64_t values.
    def test_uint64t(self):
        uint64t_param = Param("key", "uint64_t", "1", False, "description")
        self.assertEqual(uint64t_param.get_uint64_t(), (1, True))

    # Unknown type, should fall back to stream operators
    def test_UnknownType(self):
        double_param = Param("key", "double", "1.0", False, "description")
        value, rc = double_param.get_angle()
        self.assertTrue(rc)
        self.assertEqual(value.radian(), 1.0)

    # Test setting and reading vector2i values.
    def test_Vector2i(self):
        vect2i_param = Param("key", "vector2i", "0 0", False, "description")
        self.assertEqual(vect2i_param.get_vector2i(), (Vector2i(0, 0), True))

    def test_invalid_constructor(self):
        with self.assertRaises(SDFErrorsException):
            Param("key", "badtype", "0", False, "description"),

    def test_set_description(self):
        uint64_param = Param("key", "uint64_t", "1", False, "description")
        uint64_param.set_description("new desc")
        self.assertEqual("new desc", uint64_param.get_description())

    def test_setting_parent_element(self):
        parent_element = Element()
        double_param = Param("key", "double", "1.0", False, "description")
        self.assertTrue(double_param.set_parent_element(parent_element))

        self.assertIsNotNone(double_param.get_parent_element())
        self.assertEqual(parent_element, double_param.get_parent_element())

        # Set a new parent Element
        new_parent_element = Element()

        self.assertTrue(double_param.set_parent_element(new_parent_element))
        self.assertIsNotNone(double_param.get_parent_element())
        self.assertEqual(new_parent_element, double_param.get_parent_element())

        # Remove the parent Element
        self.assertTrue(double_param.set_parent_element(None))
        self.assertIsNone(double_param.get_parent_element())

    def test_copy_constructor(self):
        parent_element = Element()

        double_param = Param("key", "double", "1.0", False, "description")
        self.assertTrue(double_param.set_parent_element(parent_element))

        self.assertNotEqual(None, double_param.get_parent_element())
        self.assertEqual(parent_element, double_param.get_parent_element())

        newParam = Param(double_param)
        self.assertNotEqual(None, newParam.get_parent_element())
        self.assertEqual(parent_element, newParam.get_parent_element())

    def test_get_as_string(self):
        param = Param("test_key", "int", "5", False, "bla")
        param.get_as_string()
        param.get_parent_element()
        elem = Element()
        param.set_parent_element(elem)
        self.assertEqual(elem, param.get_parent_element())


if __name__ == '__main__':
    unittest.main()
