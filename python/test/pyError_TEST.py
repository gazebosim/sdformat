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
from sdformat import Error
import sdformat as sdf
import unittest

class ErrorColor(unittest.TestCase):

  def test_default_construction(self):
    error = Error()

    self.assertEqual(error.is_valid(), False)
    self.assertEqual(error.code(), sdf.ErrorCode.NONE)
    self.assertFalse(error.message())
    self.assertFalse(error.xml_path())
    self.assertFalse(error.file_path())
    self.assertFalse(error.line_number())

    error.set_xml_path("/sdf/world")
    self.assertTrue(error.xml_path())

    self.assertEqual("/sdf/world", error.xml_path())

    error.set_file_path("/tmp/test_file.sdf")
    self.assertTrue(error.file_path())
    self.assertEqual("/tmp/test_file.sdf", error.file_path())

    error.set_line_number(5)
    self.assertTrue(error.line_number())
    self.assertEqual(5, error.line_number())


  def test_value_construction_without_file(self):
    error = Error(sdf.ErrorCode.FILE_READ, "Unable to read a file")
    self.assertEqual(error.is_valid(), True)
    self.assertEqual(error.code(), sdf.ErrorCode.FILE_READ)
    self.assertEqual(error.message(), "Unable to read a file")
    self.assertFalse(error.xml_path())
    self.assertFalse(error.file_path())
    self.assertFalse(error.line_number())


  def test_deepcopy(self):
    error = Error(sdf.ErrorCode.FILE_READ, "Unable to read a file")

    error2 = copy.deepcopy(error)
    self.assertEqual(error.is_valid(), error2.is_valid())
    self.assertEqual(error.code(), error2.code())
    self.assertEqual(error.message(), error2.message())
    self.assertEqual(error.xml_path(), error2.xml_path())
    self.assertEqual(error.file_path(), error2.file_path())
    self.assertEqual(error.line_number(), error2.line_number())

    error.set_file_path("file.sdf")
    error.set_line_number(5)
    error.set_xml_path("/sdf/mode")
    self.assertEqual(error.file_path(), "file.sdf")
    self.assertEqual(error2.file_path(), None)
    self.assertEqual(error.line_number(), 5)
    self.assertEqual(error2.line_number(), None)
    self.assertEqual(error.xml_path(), "/sdf/mode")
    self.assertEqual(error2.xml_path(), None)

  def test_value_construction_with_file(self):
    emptyfile_path = "Empty/file/path";
    error = Error(
        sdf.ErrorCode.FILE_READ,
        "Unable to read a file",
        emptyfile_path)
    self.assertEqual(error.is_valid(), True)
    self.assertEqual(error.code(), sdf.ErrorCode.FILE_READ)
    self.assertEqual(error.message(), "Unable to read a file")
    self.assertFalse(error.xml_path())
    self.assertTrue(error.file_path())
    self.assertEqual(error.file_path(), emptyfile_path)
    self.assertFalse(error.line_number())


  def test_value_construction_with_line_number(self):
    emptyfile_path = "Empty/file/path";
    line_number = 10;
    error = Error(
        sdf.ErrorCode.FILE_READ,
        "Unable to read a file",
        emptyfile_path,
        line_number)
    self.assertEqual(error.is_valid(), True)
    self.assertEqual(error.code(), sdf.ErrorCode.FILE_READ)
    self.assertEqual(error.message(), "Unable to read a file")
    self.assertFalse(error.xml_path())
    self.assertTrue(error.file_path())
    self.assertEqual(error.file_path(), emptyfile_path)
    self.assertTrue(error.line_number())
    self.assertEqual(error.line_number(), line_number)


  def test_value_construction_with_xmlpath(self):
    emptyfile_path = "Empty/file/path";
    line_number = 10;
    error = Error(
        sdf.ErrorCode.FILE_READ,
        "Unable to read a file",
        emptyfile_path,
        line_number)
    self.assertEqual(error.is_valid(), True)
    self.assertEqual(error.code(), sdf.ErrorCode.FILE_READ)
    self.assertEqual(error.message(), "Unable to read a file")
    self.assertTrue(error.file_path())
    self.assertEqual(error.file_path(), emptyfile_path)
    self.assertTrue(error.line_number())
    self.assertEqual(error.line_number(), line_number)

    emptyxml_path = "/sdf/model";
    self.assertFalse(error.xml_path())
    error.set_xml_path(emptyxml_path)
    self.assertTrue(error.xml_path())
    self.assertEqual(error.xml_path(), emptyxml_path)


if __name__ == '__main__':
    unittest.main()
