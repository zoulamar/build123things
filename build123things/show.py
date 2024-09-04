#!/usr/bin/env python3

""" This module contains single-dispatch methods to vizualize Things and Joints in cadquery-editor.

TODO: Encapsulate in "show" module (a directory with __init__.py) which can detect currently used visualizer and load appropriate visualization bindings.
"""

import build123d as bd
from functools import singledispatch
from typing import Any, Callable, NoReturn, Set
from build123things import ReferenceTransformResolver, Thing, TransformResolver
from build123things.joints import Revolute, Rigid

DEBUG:Set[str] = set()
DEBUG.add("show:Thing")
DEBUG.add("show:ThingGrounder")
DEBUG.add("show:Location")
DEBUG.add("show:Shape")

try:
    from __main__ import show_object
except ImportError:
    def show_object(*args, **kwargs):
        """ Function defined by CQ-Editor, rendering given geometry in a graphical window.

        This dummy declaration allows debugging a design in a terminal where the CQ-Editor is not available.
        """
        pass

@singledispatch
def show(x:Any, *args, **kwargs) -> NoReturn:
    """ A default showing function to visualize a Thing with its attached assembly subtree. """
    raise NotImplementedError(f"Visulaization of type {type(x)} ({args}, {kwargs}) not supported.")

@show.register
def _ (thing:Thing, recurse=False):
    if "show:Thing" in DEBUG:
        print(f"Showing {repr(thing)}")
    if recurse is False:
        result = thing.result()
        if result is not None:
            try:
                diag = result.bounding_box().diagonal
            except:
                diag = 100
        else:
            try:
                diag = thing.bounding_box(consider_assembly=True).diagonal
            except:
                diag=100
        diag = 100
        for _, construction in thing.enumerate_reference_geometries():
            if construction is not result:
                if isinstance(construction, ReferenceTransformResolver):
                    show(construction)
                else:
                    show(construction, diag)
        if result is not None:
            show_object(result, options={"color":thing.__material__.color.rgba})#, "alpha":0.1})
    else:
        show(ReferenceTransformResolver(thing, bd.Location(), None))

@show.register
def _ (thing_grounder:TransformResolver):
    if "show:ThingGrounder" in DEBUG:
        print(f"Showing {repr(thing_grounder)}")
    res = thing_grounder.result() # type: ignore
    if res is not None:
        diag = res.bounding_box().diagonal # type: ignore
        show_object(res, options={"color":thing_grounder.__material__.color.rgba, "alpha":0}) # type: ignore
    else:
        diag = thing_grounder.wrapped.bounding_box().diagonal
    for _, attached in thing_grounder.enumerate_assembly():
        attached:TransformResolver
        """ The assembly returns the Grounder. """
        #show(attached._joint, attached._orig_mount.location, diag)
        show(attached)

@show.register
def _ (where:bd.Location, diag:float):
    if "show:Location" in DEBUG:
        print(f"Showing {repr(where)}")
    gizmos(where, l = diag*.1)

@show.register
def _ (geom:bd.Shape, diag:float):
    if "show:Shape" in DEBUG:
        print(f"Showing {repr(geom)}")
    sat = 100
    if hasattr(geom, "_dim"):
        if geom._dim == 1:
            color=(0,sat,0)
        elif geom._dim == 2:
            color=(10,sat-10,10)
        else:
            color=(sat,0,sat)
    else:
        color = (sat,0,sat)
    show_object(geom, options={"color":color})#, "alpha":0.01})

@show.register
def _ (_:Rigid, where:bd.Location, diag:float)->None:
    """ Displays a marker at given location. """
    l = diag*.01
    show_object(where * bd.Cone(bottom_radius=l, top_radius=l*.5, height=l, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)), options={"color": (100,0,0)})

@show.register
def _ (joint:Revolute, where:bd.Location, diag:float)->None:
    """ Displays a marker at given location. """
    sat = 100
    opt_x = {"color" : (sat,0,0)}
    opt_y = {"color" : (0,sat,0)}
    opt_z = {"color" : (0,0,sat)}

    l = diag*.1

    base_axis = bd.Cylinder(radius=l/60,height=l,align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

    # Joint X axis
    show_object(where * bd.Location((0,0,0),(0,90,0)) * base_axis, options=opt_x)

    # Joint Y axis
    show_object(where * bd.Location((0,0,0),(-90,0,0)) * base_axis, options=opt_y)

    # Joint Z axis
    show_object(where * bd.Location((0,0,0),(0,0,0)) * base_axis, options=opt_z)
    show_object(where * bd.Location((0,0,l*.75),(0,0,0)) * bd.Cone(bottom_radius=l*.1, top_radius=l/60, height=l*.25, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)), options=opt_z)

def grid(show_object:Callable, plane:bd.Plane=bd.Plane.XY, l=1, n=10)->None:
    """ Adjusted build123d example available at https://build123d.readthedocs.io/en/latest/location_arithmetic.html """
    for i in range(-n,n+1):
        show_object(plane.location * bd.Line((i*l,-n*l),(i*l,n*l)))
        show_object(plane.location * bd.Line((-n*l,i*l),(n*l,i*l)))

def gizmos(where:bd.Location, l:float=10, options={})->None:
    """ Adjusted build123d example available at https://build123d.readthedocs.io/en/latest/location_arithmetic.html """
    sat = 100
    opt_x = {"color" : (sat,0,0)}
    opt_x.update(options)
    opt_y = {"color" : (0,sat,0)}
    opt_y.update(options)
    opt_z = {"color" : (0,0,sat)}
    opt_z.update(options)

    #base_arrow = bd.revolve(bd.Arrow(arrow_size=l, shaft_path=bd.Line((l,0),(0,0)), shaft_width=l/10).face(), bd.Axis.X) # type:ignore
    #base_arrow = bd.Arrow(arrow_size=l, shaft_path=bd.Line((l,0),(0,0)), shaft_width=l/10)
    base_arrow = bd.Cylinder(radius=l/60,height=l,align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

    show_object(where * bd.Location((0,0,0),(0,90,0)) * base_arrow, options=opt_x)
    show_object(where * bd.Location((0,0,0),(-90,0,0)) * base_arrow, options=opt_y)
    show_object(where * bd.Location((0,0,0),(0,0,0)) * base_arrow, options=opt_z)
    #show_object(self * bd.Sphere(radius=l/10),options={"color":(sat,0,sat)})

