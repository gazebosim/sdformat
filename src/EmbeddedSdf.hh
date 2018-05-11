/*
 * Copyright 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef SDF_EMBEDDEDSDF_HH_
#define SDF_EMBEDDEDSDF_HH_

const std::map<std::string, std::string> embeddedSdf =
{
  {"root.sdf", R"__sdf_literal__(<element name="sdf" required="1">
  <description>SDF base element.</description>

  <attribute name="version" type="string" default="1.5" required="1">
    <description>Version number of the SDF format.</description>
  </attribute>

  <include filename="world.sdf" required="*"/>
  <include filename="model.sdf" required="*"/>
  <include filename="actor.sdf" required="*"/>
  <include filename="light.sdf" required="*"/>

</element> <!-- End SDF -->)__sdf_literal__"}

};
#endif
