/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: John Hsu */
#pragma warning(push, 0)

#include <urdf_sensor/sensor.h>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <tinyxml.h>
// #include <console_bridge/console.h>

namespace urdf{

bool parsePose(Pose &pose, TiXmlElement* xml);

bool parseCamera(Camera &camera, TiXmlElement* config)
{
  camera.clear();
  camera.type = VisualSensor::CAMERA;

  TiXmlElement *image = config->FirstChildElement("image");
  if (image)
  {
    const char* width_char = image->Attribute("width");
    if (width_char)
    {
      try
      {
        camera.width = std::stoul(width_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }
    else
    {
      return false;
    }

    const char* height_char = image->Attribute("height");
    if (height_char)
    {
      try
      {
        camera.height = std::stoul(height_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }
    else
    {
      return false;
    }

    const char* format_char = image->Attribute("format");
    if (format_char)
      camera.format = std::string(format_char);
    else
    {
      return false;
    }    

    const char* hfov_char = image->Attribute("hfov");
    if (hfov_char)
    {
      try
      {
        camera.hfov = std::stod(hfov_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }
    else
    {
      return false;
    }

    const char* near_char = image->Attribute("near");
    if (near_char)
    {
      try
      {
        camera.near = std::stod(near_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }
    else
    {
      return false;
    }

    const char* far_char = image->Attribute("far");
    if (far_char)
    {
      try
      {
        camera.far = std::stod(far_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }
    else
    {
      return false;
    }
    
  }
  else
  {
    return false;
  }
  return true;
}

bool parseRay(Ray &ray, TiXmlElement* config)
{
  ray.clear();
  ray.type = VisualSensor::RAY;

  TiXmlElement *horizontal = config->FirstChildElement("horizontal");
  if (horizontal)
  {
    const char* samples_char = horizontal->Attribute("samples");
    if (samples_char)
    {
      try
      {
        ray.horizontal_samples = std::stoul(samples_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }

    const char* resolution_char = horizontal->Attribute("resolution");
    if (resolution_char)
    {
      try
      {
        ray.horizontal_resolution = std::stod(resolution_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }   
    
    const char* min_angle_char = horizontal->Attribute("min_angle");
    if (min_angle_char)
    {
      try
      {
        ray.horizontal_min_angle = std::stod(min_angle_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }

    const char* max_angle_char = horizontal->Attribute("max_angle");
    if (max_angle_char)
    {
      try
      {
        ray.horizontal_max_angle = std::stod(max_angle_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }
  }
  
  TiXmlElement *vertical = config->FirstChildElement("vertical");
  if (vertical)
  {
    const char* samples_char = vertical->Attribute("samples");
    if (samples_char)
    {
      try
      {
        ray.vertical_samples = std::stoul(samples_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }

    const char* resolution_char = vertical->Attribute("resolution");
    if (resolution_char)
    {
      try
      {
        ray.vertical_resolution = std::stod(resolution_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }   
    
    const char* min_angle_char = vertical->Attribute("min_angle");
    if (min_angle_char)
    {
      try
      {
        ray.vertical_min_angle = std::stod(min_angle_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }

    const char* max_angle_char = vertical->Attribute("max_angle");
    if (max_angle_char)
    {
      try
      {
        ray.vertical_max_angle = std::stod(max_angle_char);
      }
      catch (std::invalid_argument &e)
      {
        return false;
      }
      catch (std::out_of_range &e)
      {
        return false;
      }
    }
  }
  return false;
}

VisualSensorSharedPtr parseVisualSensor(TiXmlElement *g)
{
  VisualSensorSharedPtr visual_sensor;

  // get sensor type
  TiXmlElement *sensor_xml;
  if (g->FirstChildElement("camera"))
  {
    Camera *camera = new Camera();
    visual_sensor.reset(camera);
    sensor_xml = g->FirstChildElement("camera");
    if (!parseCamera(*camera, sensor_xml))
      visual_sensor.reset();
  }
  else if (g->FirstChildElement("ray"))
  {
    Ray *ray = new Ray();
    visual_sensor.reset(ray);
    sensor_xml = g->FirstChildElement("ray");
    if (!parseRay(*ray, sensor_xml))
      visual_sensor.reset();
  }
  else
  {
  }
  return visual_sensor;
}


bool parseSensor(Sensor &sensor, TiXmlElement* config)
{
  sensor.clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    return false;
  }
  sensor.name = std::string(name_char);

  // parse parent_link_name
  const char *parent_link_name_char = config->Attribute("parent_link_name");
  if (!parent_link_name_char)
  {
    return false;
  }
  sensor.parent_link_name = std::string(parent_link_name_char);

  // parse origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o)
  {
    if (!parsePose(sensor.origin, o))
      return false;
  }

  // parse sensor
  sensor.sensor = parseVisualSensor(config);
  return true;
}


}

#pragma warning(pop)
