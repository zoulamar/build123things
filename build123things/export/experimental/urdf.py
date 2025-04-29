"""
Implements exporting a Thing to URDF file format.
Currently, only 3D objects are subject of export.
"""

import os
from random import random
import tempfile
from copy import copy
from pathlib import Path
from typing import IO, Any, Dict, List

from build123things import Thing, AbstractJoint
from xml.etree.ElementTree import Element, ElementTree
import xml.etree.ElementTree
import build123d as bd
from stl import mesh
import numpy as np

NAMESPACE_SEPARATOR = ":"

def export_as_robot(thing:Thing, file:str|Path|IO, short_empty_elements:bool=True, fuse_rigid_components:bool = True) -> None:
    store_links = []
    store_joints:List[Element] = []
    store_materials = []

    def process_link(thing:Thing, thing_name:str, precision:int=3):
        """ The function incrementally builds the XML element tree. The function returns the name of the given component. """
        #nonlocal store_joints, store_links, store_materials
        for joint_name, joint_obj in thing.enumerate_joints():
            print(f"---- {joint_name}: {joint_obj}")
            joint_name = f"{thing_name}{NAMESPACE_SEPARATOR}{joint_name}"
            child_link_prefix = f"{joint_name}{NAMESPACE_SEPARATOR}"
            for motor_name, motor in joint_obj.enumerate_motors_with_names(d0=False,d1=False,d2=False,d3=True):
                # Now expot the joint itself. The exporter is supposed to move the child links to the mounted point.
                store_joints.append(joint_obj.export_urdf(joint_name=joint_name, motor_name=motor_name, parent_link_name=thing_name, child_link_prefix=child_link_prefix))

                # Now I can proceed with exporting the child link, already moved to the joint-local coordinates as required by the URDF.
                process_link(motor.reference_thing, f"{child_link_prefix}{motor_name}")

        stl_fname = thing_name + ".stl"
        thing.result().export_stl("build/" + stl_fname)

        inertia, com = thing.matrix_of_inertia()
        element = Element("link", {
            "name" : thing_name
        })

        inertial = Element("inertial")
        inertial.append(Element("origin", {
            "xyz" : f"{com[0]:.{precision}f} {com[1]:.{precision}f} {com[2]:.{precision}f}"
        }))
        inertial.append(Element("mass", {
            "value" : f"{thing.mass():.{precision}f}"
        }))
        inertial.append(Element("inertia", {
            "ixx" : f"{inertia[0,0]:.{precision}f}",
            "ixy" : f"{inertia[0,1]:.{precision}f}",
            "ixz" : f"{inertia[0,2]:.{precision}f}",
            "iyy" : f"{inertia[1,1]:.{precision}f}",
            "iyz" : f"{inertia[1,2]:.{precision}f}",
            "izz" : f"{inertia[2,2]:.{precision}f}",
        }))
        element.append(inertial)

        visual = Element("visual")
        visual_name = Element("name")
        visual_name.text = thing_name
        visual.append(visual_name)
        visual.append(Element("origin", {
            "xyz": "0 0 0",
            "rpy": "0 0 0",
        }))
        visual_mesh = Element("mesh", {
            "filename": stl_fname
        })
        visual_geometry = Element("geometry")
        visual_geometry.append(visual_mesh)
        visual.append(visual_geometry)

        visual_material = Element("material")
        visual_color = Element("color", {
            "rgba": f"{random()} {random()} {random()} .5"
        })
        visual_material.append(visual_color)
        visual.append(visual_material)

        element.append(visual)

        store_links.append(element)

    process_link(thing, "base_link")

    document_root = Element("robot")
    document_root.attrib = {"name" : str(type(thing).__name__)}
    for m in reversed(store_materials):
        document_root.append(m)
    for l in reversed(store_links):
        document_root.append(l)
    for j in reversed(store_joints):
        document_root.append(j)

    document = ElementTree(document_root)
    xml.etree.ElementTree.indent(document)
    document.write(file, xml_declaration=True, short_empty_elements=short_empty_elements)



#    @abstractmethod
#    def export_urdf(self, joint_name:str, motor_name:str, parent_link_name:str, child_link_prefix:str)->Any:
#        """ This asserts that the moved "motor" Thing gets moved in accord with the URDF standard. """
#        warnings.warn(DeprecationWarning())
#        motor = getattr(self, motor_name)
#        assert isinstance(motor, AbstractJoint.Motor)
#        reference_mount = getattr(motor.reference_thing, motor.reference_name)
#        reference_thing_copy_loc = reference_mount if isinstance(reference_mount, bd.Location) else reference_mount.location
#        motor.reference_thing.move(reference_thing_copy_loc.inverse())


#    def export_urdf(self, joint_name:str, motor_name:str, parent_link_name: str, child_link_prefix:str) -> Element:
#        # Moves the "motor" such that its Origin is in accord with the URDF standard.
#        super().export_urdf(joint_name=joint_name, motor_name=motor_name, parent_link_name=parent_link_name, child_link_prefix=child_link_prefix)
#
#        motor = getattr(self, motor_name)
#        assert isinstance(motor, build123things.AbstractJoint.Motor)
#
#        stator_loc:bd.Location = self.stator_mount if isinstance(self.stator_mount, bd.Location) else self.stator_mount.location
#
#        joint_element = Element("joint", {
#                "name" : joint_name if self.specific_name is None else self.specific_name,
#                "type" : "continuous" if self.limit_angle is None else "revolute"
#            })
#        joint_element.append(Element("origin", {
#                "xyz" : f"{stator_loc.position.X} {stator_loc.position.Y} {stator_loc.position.Z}",
#                "rpy" : f"{stator_loc.orientation.X:.6f} {stator_loc.orientation.Y} {stator_loc.orientation.Z}",
#            }))
#        joint_element.append(Element("parent", {
#                "link" : parent_link_name
#            }))
#        joint_element.append(Element("child", {
#                "link" : child_link_prefix + "motor",
#            }))
#        joint_element.append(Element("axis", {
#                "xyz" : "0 0 1",
#            }))
#        if self.limit_angle is not None:
#            joint_element.append(Element("limit", {
#                    "lower" : str(self.limit_angle[0]),
#                    "upper" : str(self.limit_angle[1]),
#                    "effort" : str(self.limit_effort),
#                    "velocity" : str(self.limit_velocity),
#                }))
#        else:
#            joint_element.append(Element("limit", {
#                    "effort" : str(self.limit_effort),
#                    "velocity" : str(self.limit_velocity),
#                }))
#
#        return joint_element
