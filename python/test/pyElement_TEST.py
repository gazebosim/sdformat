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

from gz_test_deps.sdformat import Element, SDFErrorsException
from gz_test_deps.math import Vector3d
import unittest


class ElementTEST(unittest.TestCase):

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

    def test_set_explicitly_set_in_file(self):
        # The heirarchy in xml:
        # <parent>
        #   <elem>
        #     <child>
        #       <child2>
        #         <grandChild/>
        #       <child2>
        #     </child>
        #     <sibling/>
        #     <sibling2/>
        #   </elem>
        #   <elem2/>
        # </parent>

        parent = Element()
        elem = Element()
        parent.insert_element(elem, True)
        elem2 = Element()
        parent.insert_element(elem2, True)

        self.assertTrue(elem.get_explicitly_set_in_file())

        elem.set_explicitly_set_in_file(False)

        self.assertFalse(elem.get_explicitly_set_in_file())

        elem.set_explicitly_set_in_file(True)

        self.assertTrue(elem.get_explicitly_set_in_file())

        # the childs and siblings of the element should all be
        # set to the same value when using this function
        child = Element()
        child.set_parent(elem)
        elem.insert_element(child, False)

        sibling = Element()
        sibling.set_parent(elem)
        elem.insert_element(sibling)

        sibling2 = Element()
        sibling2.set_parent(elem)
        elem.insert_element(sibling2)

        child2 = Element()
        child2.set_parent(child)
        child.insert_element(child2)

        grandChild = Element()
        grandChild.set_parent(child)
        child.insert_element(grandChild)

        self.assertTrue(elem.get_explicitly_set_in_file())
        self.assertTrue(child.get_explicitly_set_in_file())
        self.assertTrue(sibling.get_explicitly_set_in_file())
        self.assertTrue(sibling2.get_explicitly_set_in_file())
        self.assertTrue(child2.get_explicitly_set_in_file())
        self.assertTrue(grandChild.get_explicitly_set_in_file())
        self.assertTrue(elem2.get_explicitly_set_in_file())

        elem.set_explicitly_set_in_file(False)
        self.assertFalse(elem.get_explicitly_set_in_file())
        self.assertFalse(child.get_explicitly_set_in_file())
        self.assertFalse(sibling.get_explicitly_set_in_file())
        self.assertFalse(sibling2.get_explicitly_set_in_file())
        self.assertFalse(child2.get_explicitly_set_in_file())
        self.assertFalse(grandChild.get_explicitly_set_in_file())

        # set_explicitly_set_in_file(False) is be called only on `elem`. We
        # expect get_explicitly_set_in_file() to be False for all children and
        # grandchildren of `elem`, but True for `elem2`, which is a sibling of
        # `elem`.
        self.assertTrue(elem2.get_explicitly_set_in_file())

    def test_set_explicitly_set_in_file_with_insert(self):
        parent = Element()
        parent.set_explicitly_set_in_file(False)
        child = Element()
        child.set_parent(parent)
        parent.insert_element(child)

        self.assertFalse(parent.get_explicitly_set_in_file())
        self.assertTrue(child.get_explicitly_set_in_file())

    def test_add_value(self):
        elem = Element()
        elem.set_name("test")
        elem.add_value("string", "foo", False, "foo description")

        param = elem.get_value()
        self.assertEqual(param.get_key(), "test")
        self.assertEqual(param.get_type_name(), "string")
        self.assertEqual(param.get_default_as_string(), "foo")
        self.assertEqual(param.get_description(), "foo description")
        self.assertNotEqual(param.get_parent_element(), None)
        self.assertEqual(param.get_parent_element(), elem)

    def test_add_attribute(self):
        elem = Element()

        self.assertEqual(elem.get_attribute_count(), 0)

        elem.add_attribute("test", "string", "foo", False, "foo description")
        self.assertEqual(elem.get_attribute_count(), 1)

        elem.add_attribute("attr", "float", "0.0", False, "float description")
        self.assertEqual(elem.get_attribute_count(), 2)

        param = elem.get_attribute("test")
        self.assertTrue(param.is_type_string())
        self.assertEqual(param.get_key(), "test")
        self.assertEqual(param.get_type_name(), "string")
        self.assertEqual(param.get_default_as_string(), "foo")
        self.assertEqual(param.get_description(), "foo description")

        param = elem.get_attribute("attr")
        self.assertTrue(param.is_type_float())
        self.assertEqual(param.get_key(), "attr")
        self.assertEqual(param.get_type_name(), "float")
        self.assertEqual(param.get_default_as_string(), "0")
        self.assertEqual(param.get_description(), "float description")

    def test_get_attribute_set(self):
        elem = Element()
        self.assertEqual(elem.get_attribute_count(), 0)
        elem.add_attribute("test", "string", "foo", False, "foo description")
        self.assertEqual(elem.get_attribute_count(), 1)

        self.assertFalse(elem.get_attribute_set("test"))
        elem.get_attribute("test").set_string("asdf")
        self.assertTrue(elem.get_attribute_set("test"))

    def test_remove_attribute(self):
        elem = Element()
        self.assertEqual(elem.get_attribute_count(), 0)

        elem.add_attribute("test", "string", "foo", False, "foo description")
        elem.add_attribute("attr", "float", "0.0", False, "float description")
        self.assertEqual(elem.get_attribute_count(), 2)

        elem.remove_attribute("test")
        self.assertEqual(elem.get_attribute_count(), 1)
        self.assertEqual(elem.get_attribute("test"), None)
        self.assertNotEqual(elem.get_attribute("attr"), None)

    def test_remove_all_attributes(self):
        elem = Element()
        self.assertEqual(elem.get_attribute_count(), 0)

        elem.add_attribute("test", "string", "foo", False, "foo description")
        elem.add_attribute("test2", "string", "foo", False, "foo description")
        elem.add_attribute("attr", "float", "0.0", False, "float description")
        self.assertEqual(elem.get_attribute_count(), 3)

        elem.remove_all_attributes()
        self.assertEqual(elem.get_attribute_count(), 0)
        self.assertEqual(elem.get_attribute("test"), None)
        self.assertEqual(elem.get_attribute("test2"), None)
        self.assertEqual(elem.get_attribute("attr"), None)

    def test_include(self):
        elem = Element()

        include_elem_to_store = Element()
        include_elem_to_store.set_name("include")
        uri_desc = Element()
        uri_desc.set_name("uri")
        uri_desc.add_value("string", "", True)
        include_elem_to_store.add_element_description(uri_desc)

        include_elem_to_store.add_element("uri").set_string("foo.txt")
        elem.set_include_element(include_elem_to_store)

        include_elem = elem.get_include_element()
        self.assertNotEqual(None, include_elem)
        self.assertTrue(include_elem.has_element("uri"))
        self.assertEqual("foo.txt",
                         include_elem.find_element("uri").get_string())

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
        desc = Element()

        parent.set_name("parent")
        child.set_name("child")

        parent.insert_element(child)
        self.assertIsNotNone(parent.get_first_element())

        parent.add_element_description(desc)
        self.assertEqual(parent.get_element_description_count(), 1)

        parent.add_attribute("test", "string", "foo", False, "foo description")
        self.assertEqual(parent.get_attribute_count(), 1)

        parent.add_value("string", "foo", False, "foo description")

        parent.set_file_path("/path/to/file.sdf")
        parent.set_line_number(12)
        parent.set_xml_path("/sdf/world[@name=\"default\"]")
        parent.set_original_version("1.5")

        include_elem_to_store = Element()
        include_elem_to_store.set_name("include")
        parent.set_include_element(include_elem_to_store)

        newelem = parent.clone()

        self.assertEqual("/path/to/file.sdf", newelem.file_path())
        self.assertIsNotNone(newelem.line_number())
        self.assertEqual(12, newelem.line_number())
        self.assertEqual("/sdf/world[@name=\"default\"]", newelem.xml_path())
        self.assertEqual("1.5", newelem.original_version())
        self.assertIsNotNone(newelem.get_first_element())
        self.assertEqual(newelem.get_element_description_count(), 1)
        self.assertEqual(newelem.get_attribute_count(), 1)
        self.assertIsNotNone(newelem.get_include_element())
        self.assertEqual("include", newelem.get_include_element().get_name())
        self.assertTrue(newelem.get_explicitly_set_in_file())

        self.assertIsNotNone(parent.get_value().get_parent_element())
        self.assertEqual(parent, parent.get_value().get_parent_element())

        self.assertIsNotNone(newelem.get_value().get_parent_element())
        self.assertEqual(newelem, newelem.get_value().get_parent_element())

        clonedAttribs = newelem.get_attributes()
        self.assertEqual(newelem, clonedAttribs[0].get_parent_element())

    def test_clear_elements(self):
        parent = Element()
        child = Element()

        parent.set_file_path("/path/to/file.sdf")
        parent.set_line_number(12)
        parent.set_xml_path("/sdf/world[@name=\"default\"]")
        parent.set_original_version("1.5")
        child.set_parent(parent)
        parent.insert_element(child)

        self.assertEqual("/path/to/file.sdf", parent.file_path())
        self.assertEqual(12, parent.line_number())
        self.assertEqual("/sdf/world[@name=\"default\"]", parent.xml_path())
        self.assertEqual("1.5", parent.original_version())
        self.assertNotEqual(parent.get_first_element(), None)
        self.assertEqual("/path/to/file.sdf",
                         parent.get_first_element().file_path())
        self.assertEqual("1.5", parent.get_first_element().original_version())

        parent.clear_elements()

        self.assertEqual(parent.get_first_element(), None)
        self.assertEqual("/path/to/file.sdf", parent.file_path())
        self.assertEqual(12, parent.line_number())
        self.assertEqual("1.5", parent.original_version())

    def test_clear(self):
        parent = Element()
        child = Element()

        parent.set_file_path("/path/to/file.sdf")
        parent.set_line_number(12)
        parent.set_xml_path("/sdf/world[@name=\"default\"]")
        parent.set_original_version("1.5")
        child.set_parent(parent)
        parent.insert_element(child)

        self.assertEqual("/path/to/file.sdf", parent.file_path())
        self.assertEqual(12, parent.line_number())
        self.assertEqual("/sdf/world[@name=\"default\"]", parent.xml_path())
        self.assertEqual("1.5", parent.original_version())
        self.assertNotEqual(parent.get_first_element(), None)
        self.assertEqual("/path/to/file.sdf",
                         parent.get_first_element().file_path())
        self.assertEqual("1.5", parent.get_first_element().original_version())

        parent.clear()

        self.assertEqual(parent.get_first_element(), None)
        self.assertEqual(parent.file_path(), "")
        self.assertIsNone(parent.line_number())
        self.assertEqual(parent.xml_path(), "")
        self.assertEqual(parent.original_version(), "")

    def test_to_string_elements(self):
        parent = Element()
        child = Element()

        parent.set_name("parent")
        child.set_name("child")

        parent.insert_element(child)
        self.assertNotEqual(parent.get_first_element(), None)

        parent.add_attribute("test", "string", "foo", False, "foo description")
        self.assertEqual(parent.get_attribute_count(), 1)

        # attribute won't print unless it has been set
        self.assertFalse(parent.get_attribute_set("test"))
        self.assertEqual(
            parent.to_string("<!-- prefix -->"), "<!-- prefix --><parent>\n"
            "<!-- prefix -->  <child/>\n"
            "<!-- prefix --></parent>\n")

        test = parent.get_attribute("test")
        self.assertNotEqual(None, test)
        self.assertFalse(test.get_set())
        self.assertTrue(test.set_from_string("foo"))
        self.assertTrue(test.get_set())
        self.assertTrue(parent.get_attribute_set("test"))

        self.assertEqual(
            parent.to_string("<!-- prefix -->"),
            "<!-- prefix --><parent test='foo'>\n"
            "<!-- prefix -->  <child/>\n"
            "<!-- prefix --></parent>\n")

    def test_set(self):
        elem = Element()

        elem.add_value("string", "val", False, "val description")

        self.assertTrue(elem.set_string("hello"))
        self.assertEqual(elem.get_string(), "hello")

    def test_copy(self):
        src = Element()
        dest = Element()

        src.set_name("test")
        src.set_file_path("/path/to/file.sdf")
        src.set_line_number(12)
        src.set_xml_path("/sdf/world[@name=\"default\"]")
        src.set_original_version("1.5")
        src.add_value("string", "val", False, "val description")
        src.add_attribute("test", "string", "foo", False, "foo description")
        src.insert_element(Element())

        include_elem_to_store = Element()
        include_elem_to_store.set_name("include")
        src.set_include_element(include_elem_to_store)

        dest.copy(src)

        self.assertEqual("/path/to/file.sdf", dest.file_path())
        self.assertEqual(12, dest.line_number())
        self.assertEqual("/sdf/world[@name=\"default\"]", dest.xml_path())
        self.assertEqual("1.5", dest.original_version())

        param = dest.get_value()
        self.assertTrue(param.is_type_string())
        self.assertEqual(param.get_key(), "test")
        self.assertEqual(param.get_type_name(), "string")
        self.assertEqual(param.get_default_as_string(), "val")
        self.assertEqual(param.get_description(), "val description")
        self.assertNotEqual(param.get_parent_element(), None)
        self.assertEqual(param.get_parent_element(), dest)

        self.assertEqual(dest.get_attribute_count(), 1)
        self.assertTrue(dest.get_explicitly_set_in_file())
        param = dest.get_attribute("test")
        self.assertTrue(param.is_type_string())
        self.assertEqual(param.get_key(), "test")
        self.assertEqual(param.get_type_name(), "string")
        self.assertEqual(param.get_default_as_string(), "foo")
        self.assertEqual(param.get_description(), "foo description")
        self.assertNotEqual(param.get_parent_element(), None)
        self.assertEqual(param.get_parent_element(), dest)

        self.assertNotEqual(dest.get_first_element(), None)
        self.assertNotEqual(dest.get_include_element(), None)
        self.assertEqual("include", dest.get_include_element().get_name())

    def test_get_next_element(self):
        child = Element()
        parent = Element()

        child.set_parent(parent)

        self.assertEqual(child.get_next_element("foo"), None)

    # Helper used in test_find_element
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
        self.assertEqual("first_child",
                         child_elem_b.get_attribute("name").get_as_string())


if __name__ == '__main__':
    unittest.main()
