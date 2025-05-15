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
from sdformat import ParserConfig
from sdformattest import source_file, test_file
import unittest


class ParserConfigColor(unittest.TestCase):

    def test_construction(self):
        config = ParserConfig()
        self.assertFalse(config.find_file_callback())
        self.assertFalse(config.uri_path_map())
        self.assertFalse(config.find_file_callback())

        testDir = source_file()
        config.add_uri_path("file://", testDir)

        self.assertTrue(config.uri_path_map())
        it = config.uri_path_map().get("file://")
        self.assertEqual(1, len(it))
        self.assertEqual(it[0], testDir)

        def testFunc(argument):
            return "test/dir2"

        config.set_find_callback(testFunc)
        self.assertTrue(config.find_file_callback())
        self.assertEqual("test/dir2", config.find_file_callback()("empty"))


    def test_copy_construction(self):
        # The directory used in add_uri_path must exist in the filesystem
        # so we'll use the source path
        testDir1 = source_file()
        testDir2 = test_file()

        config1 = ParserConfig()
        config1.add_uri_path("file://", testDir1)
        it = config1.uri_path_map().get("file://")
        self.assertEqual(1, len(it))
        self.assertEqual(it[0], testDir1)

        config2 = ParserConfig(config1)
        it = config2.uri_path_map().get("file://")
        self.assertEqual(1, len(it))
        self.assertEqual(it[0], testDir1)

        config2.add_uri_path("file://", testDir2)
        it = config2.uri_path_map().get("file://")
        self.assertEqual(2, len(it))
        self.assertEqual(it[1], testDir2)

        # Updating config2 should not affect config1
        it = config1.uri_path_map().get("file://")
        self.assertEqual(1, len(it))
        self.assertEqual(it[0], testDir1)


    def test_deepcopy(self):
        # The directory used in add_uri_path must exist in the filesystem
        # so we'll use the source path
        testDir1 = source_file()
        testDir2 = test_file()

        config1 = ParserConfig()
        config1.add_uri_path("file://", testDir1)
        it = config1.uri_path_map().get("file://")
        self.assertEqual(1, len(it))
        self.assertEqual(it[0], testDir1)

        config2 = copy.deepcopy(config1)
        it = config2.uri_path_map().get("file://")
        self.assertEqual(1, len(it))
        self.assertEqual(it[0], testDir1)

        config2.add_uri_path("file://", testDir2)
        it = config2.uri_path_map().get("file://")
        self.assertEqual(2, len(it))
        self.assertEqual(it[1], testDir2)

        # Updating config2 should not affect config1
        it = config1.uri_path_map().get("file://")
        self.assertEqual(1, len(it))
        self.assertEqual(it[0], testDir1)

    def test_copy(self):
        # The directory used in add_uri_path must exist in the filesystem,
        # so we'll use the source path
        testDir1 = source_file()

        config1 = ParserConfig()
        config1.add_uri_path("file://", testDir1)

        it = config1.uri_path_map().get("file://")
        self.assertEqual(1, len(it))
        self.assertEqual(it[0], testDir1)

        config2 = config1
        it = config2.uri_path_map().get("file://")
        self.assertEqual(1, len(it))
        self.assertEqual(it[0], testDir1)


if __name__ == '__main__':
    unittest.main()
