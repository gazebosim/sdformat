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

  <!-- add collision names -->
  <xsl:template match="link/collision">
    <xsl:variable name="count">
      <xsl:number count="collision" format="1"/>
    </xsl:variable>
    <xsl:variable name="count-1" select="$count - 1"/>
    <xsl:choose>
      <xsl:when test="$count-1=0">
        <xsl:element name="{name()}">
          <xsl:attribute name="name">
            <xsl:value-of select="concat(../@name, '_', name())"/>
          </xsl:attribute>
          <xsl:apply-templates select="@*|node()"/>
        </xsl:element>
      </xsl:when>
      <xsl:otherwise>
        <xsl:element name="{name()}">
          <xsl:attribute name="name">
            <xsl:value-of select="concat(../@name, '_', name(), '_', $count-1)"/>
          </xsl:attribute>
          <xsl:apply-templates select="@*|node()"/>
        </xsl:element>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- add visual names -->
  <xsl:template match="link/visual">
    <xsl:variable name="count">
      <xsl:number count="visual" format="1"/>
    </xsl:variable>
    <xsl:variable name="count-1" select="$count - 1"/>
    <xsl:choose>
      <xsl:when test="$count-1=0">
        <xsl:element name="{name()}">
          <xsl:attribute name="name">
            <xsl:value-of select="concat(../@name, '_', name())"/>
          </xsl:attribute>
          <xsl:apply-templates select="@*|node()"/>
        </xsl:element>
      </xsl:when>
      <xsl:otherwise>
        <xsl:element name="{name()}">
          <xsl:attribute name="name">
            <xsl:value-of select="concat(../@name, '_', name(), '_', $count-1)"/>
          </xsl:attribute>
          <xsl:apply-templates select="@*|node()"/>
        </xsl:element>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- remove collision, visual group attributes -->
  <xsl:template match="link/collision/@group|link/visual/@group"/>

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
  <xsl:template match="robot/joint/axis/@xyz">
    <!-- convert attributes to elements -->
    <xsl:element name="{name()}">
      <xsl:value-of select="."/>
    </xsl:element>
  </xsl:template>
  <!-- move dynamics and limit to axis element -->
  <xsl:template match="robot/joint/axis">
    <xsl:copy>
      <xsl:apply-templates select="@*"/>
      <xsl:element name="dynamics">
        <xsl:for-each select="../dynamics/@*">
          <xsl:element name="{name()}">
            <xsl:value-of select="."/>
          </xsl:element>
        </xsl:for-each>
      </xsl:element>
      <xsl:element name="limit">
        <xsl:for-each select="../limit/@*">
          <xsl:element name="{name()}">
            <xsl:value-of select="."/>
          </xsl:element>
        </xsl:for-each>
      </xsl:element>
      <xsl:apply-templates select="node()"/>
    </xsl:copy>
  </xsl:template>
  <xsl:template match="robot/joint/dynamics" />
  <xsl:template match="robot/joint/limit" />
  <!-- convert parent and child -->
  <xsl:template match="robot/joint/child|joint/parent">
    <xsl:element name="{name()}">
      <xsl:value-of select="./@link"/>
    </xsl:element>
  </xsl:template>

  <!-- comment out joint/safety_controller and transmission -->
  <xsl:template match="robot/joint/safety_controller|transmission|transmission/*">
    <xsl:comment>
      <xsl:text> </xsl:text>
      <xsl:value-of select="local-name()" />
      <xsl:text> </xsl:text>
      <xsl:for-each select="@*">
        <xsl:value-of select="local-name()"/>
        <xsl:text>="</xsl:text>
        <xsl:value-of select="."/>
        <xsl:text>" </xsl:text>
      </xsl:for-each>
    </xsl:comment>
    <xsl:apply-templates select="./*" />
    <xsl:comment>
      <xsl:text> /</xsl:text>
      <xsl:value-of select="local-name()" />
      <xsl:text> </xsl:text>
    </xsl:comment>
  </xsl:template>

</xsl:stylesheet>
