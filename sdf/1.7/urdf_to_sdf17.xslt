<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform" version="1.0">

  <xsl:output method="xml" indent="yes" />
  <xsl:decimal-format decimal-separator="." grouping-separator="," />
  
  <!-- Identity template, provides default behavior that copies all content. -->
  <xsl:template match="@*|node()">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()"/>
    </xsl:copy>
  </xsl:template>

  <!-- robot to sdf 1.7 model -->
  <xsl:template match="robot">
    <xsl:element name="sdf">
      <xsl:attribute name="version">
        <xsl:value-of select="1.7"/>
      </xsl:attribute>
      <xsl:element name="model">
        <xsl:apply-templates select="@*|node()"/>
      </xsl:element>
    </xsl:element>
  </xsl:template>

  <!-- origin to pose -->
  <xsl:template match="origin">
    <xsl:element name="pose">
      <xsl:value-of select="./@xyz"/>
      <xsl:text> </xsl:text>
      <xsl:value-of select="./@rpy"/>
    </xsl:element>
  </xsl:template>

  <!-- joint origin to pose relative to parent link -->
  <xsl:template match="joint/origin">
    <xsl:element name="pose">
      <xsl:attribute name="relative_to">
        <xsl:value-of select="../parent/@link"/>
      </xsl:attribute>
      <xsl:value-of select="./@xyz"/>
      <xsl:text> </xsl:text>
      <xsl:value-of select="./@rpy"/>
    </xsl:element>
  </xsl:template>

  <!-- inertial properties -->
  <xsl:template match="mass">
    <xsl:element name="mass">
      <xsl:value-of select="./@value"/>
    </xsl:element>
  </xsl:template>
  <xsl:template match="inertia/@*">
    <!-- convert attributes to elements -->
    <xsl:element name="{name()}">
      <xsl:value-of select="."/>
    </xsl:element>
  </xsl:template>

  <!-- Geometry: -->
  <xsl:template match="geometry/box/@*|geometry/cylinder/@*|geometry/sphere/@*|geometry/mesh/@scale">
    <!-- convert attributes to elements -->
    <xsl:element name="{name()}">
      <xsl:value-of select="."/>
    </xsl:element>
  </xsl:template>
  <xsl:template match="geometry/mesh/@filename[starts-with(., 'package://')]">
    <xsl:variable name="new-uri" as="xs:string"
        select="concat('model://', substring-after(., 'package://'))" />
    <xsl:element name="uri">
      <xsl:value-of select="$new-uri" />
    </xsl:element>
  </xsl:template>

  <!-- joint properties -->
  <xsl:template match="joint/axis/@xyz">
    <!-- convert attributes to elements -->
    <xsl:element name="{name()}">
      <xsl:value-of select="."/>
    </xsl:element>
  </xsl:template>
  <xsl:template match="joint/child|joint/parent">
    <xsl:element name="{name()}">
      <xsl:value-of select="./@link"/>
    </xsl:element>
  </xsl:template>

</xsl:stylesheet>
