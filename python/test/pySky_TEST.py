# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
from gz.math import Angle, Color
from sdformat import Sky
import unittest


class SkyTEST(unittest.TestCase):

    def test_default_construction(self):
        sky = Sky()
        self.assertAlmostEqual(10.0, sky.time())
        self.assertAlmostEqual(6.0, sky.sunrise())
        self.assertAlmostEqual(20.0, sky.sunset())
        self.assertAlmostEqual(0.6, sky.cloud_speed())
        self.assertEqual(Angle(), sky.cloud_direction())
        self.assertAlmostEqual(0.5, sky.cloud_humidity())
        self.assertAlmostEqual(0.5, sky.cloud_mean_size())
        self.assertEqual(Color(0.8, 0.8, 0.8), sky.cloud_ambient())
        self.assertEqual("", sky.cubemap_uri())


    def test_copy_construction(self):
        sky = Sky()
        sky.set_time(1.0)
        sky.set_sunrise(5.0)
        sky.set_sunset(15.0)
        sky.set_cloud_speed(0.3)
        sky.set_cloud_direction(Angle(1.2))
        sky.set_cloud_humidity(0.9)
        sky.set_cloud_mean_size(0.123)
        sky.set_cloud_ambient(Color.BLUE)
        self.assertEqual("", sky.cubemap_uri())
        sky.set_cubemap_uri("dummyUri");

        sky2 = Sky(sky)
        self.assertAlmostEqual(1.0, sky2.time())
        self.assertAlmostEqual(5.0, sky2.sunrise())
        self.assertAlmostEqual(15.0, sky2.sunset())
        self.assertAlmostEqual(0.3, sky2.cloud_speed())
        self.assertEqual(Angle(1.2), sky2.cloud_direction())
        self.assertAlmostEqual(0.9, sky2.cloud_humidity())
        self.assertAlmostEqual(0.123, sky2.cloud_mean_size())
        self.assertEqual(Color.BLUE, sky2.cloud_ambient())
        self.assertEqual("dummyUri", sky2.cubemap_uri())


    def test_assignment(self):
        sky = Sky()
        sky.set_time(1.0)
        sky.set_sunrise(5.0)
        sky.set_sunset(15.0)
        sky.set_cloud_speed(0.3)
        sky.set_cloud_direction(Angle(1.2))
        sky.set_cloud_humidity(0.9)
        sky.set_cloud_mean_size(0.123)
        sky.set_cloud_ambient(Color.BLUE)
        sky.set_cubemap_uri("dummyUri");

        sky2 = sky
        self.assertAlmostEqual(1.0, sky2.time())
        self.assertAlmostEqual(5.0, sky2.sunrise())
        self.assertAlmostEqual(15.0, sky2.sunset())
        self.assertAlmostEqual(0.3, sky2.cloud_speed())
        self.assertEqual(Angle(1.2), sky2.cloud_direction())
        self.assertAlmostEqual(0.9, sky2.cloud_humidity())
        self.assertAlmostEqual(0.123, sky2.cloud_mean_size())
        self.assertEqual(Color.BLUE, sky2.cloud_ambient())
        self.assertEqual("dummyUri", sky2.cubemap_uri())


    def test_deepcopy(self):
        sky = Sky()
        sky.set_time(1.0)
        sky.set_sunrise(5.0)
        sky.set_sunset(15.0)
        sky.set_cloud_speed(0.3)
        sky.set_cloud_direction(Angle(1.2))
        sky.set_cloud_humidity(0.9)
        sky.set_cloud_mean_size(0.123)
        sky.set_cloud_ambient(Color.BLUE)
        sky.set_cubemap_uri("dummyUri");

        sky2 = copy.deepcopy(sky)
        self.assertAlmostEqual(1.0, sky2.time())
        self.assertAlmostEqual(5.0, sky2.sunrise())
        self.assertAlmostEqual(15.0, sky2.sunset())
        self.assertAlmostEqual(0.3, sky2.cloud_speed())
        self.assertEqual(Angle(1.2), sky2.cloud_direction())
        self.assertAlmostEqual(0.9, sky2.cloud_humidity())
        self.assertAlmostEqual(0.123, sky2.cloud_mean_size())
        self.assertEqual(Color.BLUE, sky2.cloud_ambient())
        self.assertEqual("dummyUri", sky2.cubemap_uri())


    def test_set(self):
        sky = Sky()

        sky.set_time(1.0)
        self.assertAlmostEqual(1.0, sky.time())

        sky.set_sunrise(5.0)
        self.assertAlmostEqual(5.0, sky.sunrise())

        sky.set_sunset(15.0)
        self.assertAlmostEqual(15.0, sky.sunset())

        sky.set_cloud_speed(0.3)
        self.assertAlmostEqual(0.3, sky.cloud_speed())

        sky.set_cloud_direction(Angle(1.2))
        self.assertEqual(Angle(1.2), sky.cloud_direction())

        sky.set_cloud_humidity(0.9)
        self.assertAlmostEqual(0.9, sky.cloud_humidity())

        sky.set_cloud_mean_size(0.123)
        self.assertAlmostEqual(0.123, sky.cloud_mean_size())

        sky.set_cloud_ambient(Color(0.1, 0.2, 0.3))
        self.assertEqual(Color(0.1, 0.2, 0.3), sky.cloud_ambient())

        sky.set_cubemap_uri("dummyUri");
        self.assertEqual("dummyUri", sky.cubemap_uri())

if __name__ == '__main__':
    unittest.main()
