# Migration Guide for SDF Protocol
This document contains information about migrating
between different versions of the SDF protocol.
The SDF protocol version number is specified in the `version` attribute
of the `sdf` element (1.4, 1.5, 1.6, etc.)
and is distinct from sdformat library version
(2.3, 3.0, 4.0, etc.).

# Note on backward compatibility
There are `*.convert` files that allow old sdf files to be migrated
forward programmatically.
This document aims to contain similar information to those files
but with improved human-readability..

## SDFormat 4.x to 5.x

### Deletions

1. **Removed the following functions from `parser.hh`**
    + bool initDoc(TiXmlDocument *_xmlDoc, SDFPtr _sdf);
    + bool initDoc(TiXmlDocument *_xmlDoc, ElementPtr _sdf);
    + bool initXml(TiXmlElement *_xml, ElementPtr _sdf);
    + bool readDoc(TiXmlDocument *_xmlDoc, SDFPtr _sdf, const std::string &_source);
    + bool readDoc(TiXmlDocument *_xmlDoc, ElementPtr _sdf, const std::string &_source);
    + bool readXml(TiXmlElement *_xml, ElementPtr _sdf);
    + void copyChildren(ElementPtr _sdf, TiXmlElement *_xml);

## SDFormat 3.x to 4.x

### Additions

1. **New SDF protocol version 1.6**
    + Details about the 1.5 to 1.6 transition are explained below in this same
      document

### Modifications

1. **Boost pointers and boost::function**
    + All boost pointers, boost::function in the public API have been replaced
      by their std:: equivalents (C++11 standard)

1. **`gravity` and `magnetic_field` elements are moved  from `physics` to `world`**
    + In physics element: gravity and magnetic_field tags have been moved
      from Physics to World element.
    + [pull request 247](https://bitbucket.org/osrf/sdformat/pull-requests/247)
    + [gazebo pull request 2090](https://bitbucket.org/osrf/gazebo/pull-requests/2090)

1. **New noise for IMU**
    + A new style for representing the noise properties of an `imu` was implemented
      in [pull request 199](https://bitbucket.org/osrf/sdformat/pull-requests/199)
      for sdf 1.5 and the old style was declared as deprecated.
      The old style has been removed from sdf 1.6 with the conversion script
      updating to the new style.
    + [pull request 199](https://bitbucket.org/osrf/sdformat/pull-requests/199)
    + [pull request 243](https://bitbucket.org/osrf/sdformat/pull-requests/243)
    + [pull request 244](https://bitbucket.org/osrf/sdformat/pull-requests/244)

1. **Lump:: prefix in link names**
    + Changed to \_fixed_joint_lump__ to avoid confusion with scoped names
    + [Pull request 245](https://bitbucket.org/osrf/sdformat/pull-request/245)

## SDF protocol 1.5 to 1.6

### Additions

1. **link.sdf** `enable_wind` element
    + description: If true, the link is affected by the wind
    + type: bool
    + default: false
    + required: 0
    + [pull request 240](https://bitbucket.org/osrf/sdformat/pull-requests/240)

1. **model.sdf** `enable_wind` element
    + description: If set to true, all links in the model will be affected by
      the wind.  Can be overriden by the link wind property.
    + type: bool
    + default: false
    + required: 0
    + [pull request 240](https://bitbucket.org/osrf/sdformat/pull-requests/240)

1. **model_state.sdf** `scale` element
    + description: Scale for the 3 dimensions of the model.
    + type: vector3
    + default: "1 1 1"
    + required: 0
    + [pull request 246](https://bitbucket.org/osrf/sdformat/pull-requests/246)

1. **world.sdf** `wind` element
    + description: The wind tag specifies the type and properties of the wind.
    + required: 0
    + [pull request 240](https://bitbucket.org/osrf/sdformat/pull-requests/240)

1. **world.sdf** `wind::linear_velocity` element
    + description: Linear velocity of the wind.
    + type: vector3
    + default: "0 0 0"
    + required: 0
    + [pull request 240](https://bitbucket.org/osrf/sdformat/pull-requests/240)
