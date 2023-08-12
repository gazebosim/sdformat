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

from gz_test_deps.sdformat import (Element, Param, PrintConfig,
                                   SDFErrorsException)
from gz_test_deps.math import Vector3d
import unittest


class ParamTEST(unittest.TestCase):

    def test_bool(self):
        bool_param = Param("key", "bool", "true", False, "description")
        value, rc = bool_param.get_bool()
        self.assertTrue(value)
        value, rc = bool_param.get_int()

    def test_get_as_string(self):
        param = Param("test_key", "int", "5", False, "bla")
        param.get_as_string()
        param.get_parent_element()
        elem = Element()
        param.set_parent_element(elem)
        self.assertEqual(elem, param.get_parent_element())


if __name__ == '__main__':
    unittest.main()
