# Copyright (C) 2023 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from gz_test_deps.sdformat import Element, SDFErrorsException
from gz_test_deps.math import Vector3d
import unittest


class ElementTEST(unittest.TestCase):

    def add_child_element(self, parent: Element, element_name: str,
                          add_name_attribute: bool, child_name: str):
        child = Element()
        child.set_parent(parent)
        child.set_name(element_name)
        parent.insert_element(child)
        if add_name_attribute:
            child.add_attribute("name", "string", child_name, False,
                                "description")
        return child

    def test_child(self):
        child = Element()
        parent = Element()
        self.assertIsNone(child.get_parent())
        child.set_parent(parent)
        self.assertIsNotNone(child.get_parent())

    def test_name(self):
        elem = Element()
        self.assertEqual(elem.get_name(), "")
        elem.set_name("test")
        self.assertEqual(elem.get_name(), "test")

    def test_required(self):
        elem = Element()
        self.assertEqual(elem.get_required(), "")
        elem.set_required("1")
        self.assertEqual(elem.get_required(), "1")

    def test_get_templates(self):
        elem = Element()

        elem.add_attribute("test", "string", "foo", False, "foo description")
        elem.add_attribute("test_int", "int", "5", False, "none")
        elem.add_attribute("test_vector3", "vector3", "1 2 3", False, "none")
        with self.assertRaises(SDFErrorsException):
            elem.add_attribute("test_error", "int", "bad", False, "none")

        out = elem.get_string("test")
        self.assertEqual(out, "foo")

        self.assertEqual(elem.get_int("test_int"), 5)
        self.assertEqual(elem.get_vector3("test_vector3"), Vector3d(1, 2, 3))

        pairout = elem.get_string("test", "def")
        self.assertEqual(pairout[0], "foo")
        self.assertEqual(pairout[1], True)

    def test_clone(self):
        parent = Element()
        child = Element()
        # desc = Element()

        parent.set_name("parent")
        child.set_name("child")

        parent.insert_element(child)
        self.assertIsNotNone(parent.get_first_element())

        # parent.add_element_description(desc)
        # self.assertEqual(parent.get_element_description_count(), 1)

        parent.add_attribute("test", "string", "foo", False, "foo description")
        self.assertEqual(parent.get_attribute_count(), 1)

        parent.add_value("string", "foo", False, "foo description")

        parent.set_file_path("/path/to/file.sdf")
        parent.set_line_number(12)
        parent.set_xml_path("/sdf/world[@name=\"default\"]")
        parent.set_original_version("1.5")

        includeElemToStore = Element()
        includeElemToStore.set_name("include")
        parent.set_include_element(includeElemToStore)

        newelem = parent.clone()

        self.assertEqual("/path/to/file.sdf", newelem.file_path())
        self.assertIsNotNone(newelem.line_number())
        self.assertEqual(12, newelem.line_number())
        self.assertEqual("/sdf/world[@name=\"default\"]", newelem.xml_path())
        self.assertEqual("1.5", newelem.original_version())
        self.assertIsNotNone(newelem.get_first_element())
        # self.assertEqual(newelem.get_element_description_count(), 1)
        self.assertEqual(newelem.get_attribute_count(), 1)
        self.assertIsNotNone(newelem.get_include_element())
        self.assertEqual("include", newelem.get_include_element().get_name())
        self.assertTrue(newelem.get_explicitly_set_in_file())

        # self.assertIsNotNone(parent.get_value().get_parent_element())
        # self.assertEqual(parent, parent.get_value().get_parent_element())
        #
        # self.assertIsNotNone(newelem.get_value().get_parent_element())
        # self.assertEqual(newelem, newelem.get_value().get_parent_element())

        # clonedAttribs = newelem.get_attributes()
        # self.assertEqual(newelem, clonedAttribs[0].get_parent_element())

    def test_find_element(self):
        root = Element()
        root.set_name("root")
        elem_a = self.add_child_element(root, "elem_A", False, "")
        self.add_child_element(elem_a, "child_elem_A", False, "")
        elem_b = self.add_child_element(root, "elem_B", False, "")
        self.add_child_element(elem_b, "child_elem_B", True, "first_child")
        self.add_child_element(elem_b, "child_elem_B", False, "")

        test_elem_a = root.find_element("elem_A")
        self.assertIsNotNone(test_elem_a)
        self.assertIsNotNone(test_elem_a.find_element("child_elem_A"))
        self.assertIsNone(test_elem_a.find_element("non_existent_elem"))

        elem_b = root.find_element("elem_B")
        self.assertIsNotNone(elem_b)
        # This should find the first "child_elem_B" element, which has the name
        # attribute
        child_elem_b = elem_b.find_element("child_elem_B")
        self.assertTrue(child_elem_b.has_attribute("name"))
        # self.assertEqual("first_child",
        # child_elem_b.get_attribute("name").get_as_string())


if __name__ == '__main__':
    unittest.main()
