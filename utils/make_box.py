#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospkg
import sys
from math import pi

path = os.path.dirname(__file__)
int_obj_description_path = os.path.abspath(path + "/../objects")
cnoid_path = os.path.abspath(path + "/../cnoid")

def gen_box(name, x, y, z, mass):
    x,y,z = map(float, [x,y,z])
    rsdf_dir = "{}/rsdf/{}".format(int_obj_description_path, name)
    rsdf = "{}/{}.rsdf".format(rsdf_dir, name)
    urdf = "{}/urdf/{}.urdf".format(int_obj_description_path, name)
    cnoid = "{}/{}.body".format(cnoid_path, name)
    r,g,b = [c/255. for c in [165, 136, 85]]
    mass = float(mass)
    ixx = 1/12. * mass * (y**2 + z**2)
    iyy = 1/12. * mass * (x**2 + z**2)
    izz = 1/12. * mass * (x**2 + y**2)
    urdf_template = """<?xml version="1.0" ?>
<robot name="{name}">
  <link name="base_link" />
  <joint name="box" type="fixed">
    <parent link="base_link" />
    <child link="box" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>
  <link name="box">
    <inertial>
      <origin xyz="{half_x} {half_y} {half_z}" rpy="0 0 0" />
      <mass value="{mass}" />
      <inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{iyy}" iyz="0" izz="{izz}" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="{x} {y} {z}" />
      </geometry>
      <material name="Brown">
        <color rgba="{r} {g} {b} 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="{x} {y} {z}" />
      </geometry>
    </collision>
  </link>
</robot>""".format(name = name, x = x, y = y, z = z, mass = mass, r = r, g = g, b = b, half_x = x/2, half_y = y/2, half_z = z/2, ixx = ixx, iyy = iyy, izz = izz)
    rsdf_template = """<? xml version="1.0" ?>
<robot name="{name}">
  <planar_surface name="Bottom" link="box">
    <origin rpy="0 0 0" xyz="0.0 0.0 -{half_z}" />
    <points>
      <point xy="-{half_x} {half_y}" />
      <point xy="-{half_x} -{half_y}" />
      <point xy="{half_x} -{half_y}" />
      <point xy="{half_x} {half_y}" />
    </points>
    <material name="plastic" />
  </planar_surface>
  <planar_surface name="Top" link="box">
    <origin rpy="0 0 0" xyz="0.0 0.0 {half_z}" />
    <points>
      <point xy="-{half_x} {half_y}" />
      <point xy="-{half_x} -{half_y}" />
      <point xy="{half_x} -{half_y}" />
      <point xy="{half_x} {half_y}" />
    </points>
    <material name="plastic" />
  </planar_surface>
  <planar_surface name="TopFront" link="box">
    <origin rpy="0 0 0" xyz="-{half_x} 0.0 {half_z}" />
    <points>
      <point xy="-{half_x} {half_y}" />
      <point xy="-{half_x} -{half_y}" />
      <point xy="{half_x} -{half_y}" />
      <point xy="{half_x} {half_y}" />
    </points>
    <material name="plastic" />
  </planar_surface>
  <planar_surface name="Right" link="box">
    <origin rpy="{HALF_PI} -{HALF_PI} 0" xyz="0.0 -{half_y} 0" />
    <points>
      <point xy="-{half_x} {half_z}" />
      <point xy="-{half_x} -{half_z}" />
      <point xy="{half_x} -{half_z}" />
      <point xy="{half_x} {half_z}" />
    </points>
    <material name="plastic" />
  </planar_surface>
  <planar_surface name="Left" link="box">
    <origin rpy="{PI_AND_HALF} -{HALF_PI} 0" xyz="0.0 {half_y} 0" />
    <points>
      <point xy="-{half_x} {half_z}" />
      <point xy="-{half_x} -{half_z}" />
      <point xy="{half_x} -{half_z}" />
      <point xy="{half_x} {half_z}" />
    </points>
    <material name="plastic" />
  </planar_surface>
</robot>""".format(name = name, half_x = x/2, half_y = y/2, half_z = z/2, PI = pi, HALF_PI = pi/2, PI_AND_HALF = 3*pi/2)
    cnoid_body_template = """format: ChoreonoidBody
formatVersion: 1.0

name: "{name}"
rootLink: base
links:
  -
    name: base
    jointType: free
    translation: [ 0.0, 0.0, 0.0 ]
    mass: {mass} 
    inertia: [
      {ixx}, 0, 0,
      0, {iyy}, 0,
      0, 0, {izz} ]
    elements:
      -
        type: Shape
        geometry: {{ type: Box, size: [ {x}, {y}, {z} ] }}
        appearance:
          material:
            diffuseColor: [1, 0.5, 0.3]""".format(name = name, x = x, y = y, z = z, mass = mass, ixx = ixx, iyy = iyy, izz = izz)
    if not os.path.exists(rsdf_dir):
        os.makedirs(rsdf_dir)
    if not os.path.exists(cnoid_path):
        os.makedirs(cnoid_path)
    with open(urdf, 'w') as fd:
        fd.write(urdf_template)
    with open(rsdf, 'w') as fd:
        fd.write(rsdf_template)
    with open(cnoid, 'w') as fd:
        fd.write(cnoid_body_template)

def usage(pname):
    print "{} [name] [x] [y] [z] [mass]".format(pname)

if __name__ == "__main__":
    if len(sys.argv) != 6:
        usage(sys.argv[0])
        sys.exit(1)
    gen_box(*sys.argv[1:])
