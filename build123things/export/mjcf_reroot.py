"""
Implements exporting a Thing to MuJoCo file format, but with the possibility to select the model root arbitrarily.
"""

from collections import defaultdict
from pathlib import Path
from typing import Any, Generator
import build123d as bd
import colored
from build123things import AbstractJoint, Thing, TransformResolver
from xml.etree.ElementTree import Element, ElementTree
import xml.etree.ElementTree
from functools import singledispatch
from build123things.joints import Revolute, Rigid

def fmt_scale(how:float):
    def fmt_inner(what:float):
        return f"{what * how:.4f}"
    return fmt_inner

@singledispatch
def fmt (sth:Any) -> str:
    return str(sth)

@fmt.register
def _ (x:float) -> str:
    return f"{x:.4f}"

NAMESPACE_SEPARATOR = ":"

HACK_MINIMAL_INERTIA = "0.00001"

def export(thing:Thing, target_dir:Path, mujoco_module:bool=False) -> ElementTree:
    """ """

    thing_instance_counter:dict[int,int] = defaultdict(int)
    """ Lookup of already encountered Things; each Thing retains a count of how many times it is inthe design. """

    thing_name_check_lookup:dict[str,int] = {}
    """ Just to ensure that multiple Things do not share the same name. """

    mesh_store:list[Element] = []
    """ Elements defining the meshes. """

    material_store:dict[str,Element] = {}
    """ Elements defining the visual apperance. """

    joint_name_unique_check:set[str] = set()
    """ Just to ensure two joints do not share the same name. """

    @singledispatch
    def to_mjcf(joint:Any, name:str) -> Element:
        raise NotImplementedError(f"Exporting type {type(joint)} to MJCF is not supported.")

    @to_mjcf.register
    def _(thing:Thing, where:bd.Location)->Element:
        """ Exports one Thing located w.r.t. some parent element. """

        thing_name_generic = thing.codename()
        """ How to call this Thing. """

        # Check that the codename is unique
        while thing_name_generic in thing_name_check_lookup and thing_name_check_lookup[thing_name_generic] != id(thing):
            print(f"{colored.Back.YELLOW}{colored.Fore.RED}WARNING: {colored.Style.reset} Name {thing_name_generic} is not unique!")
            thing_name_generic = thing_name_generic + "X"
        else:
            thing_name_check_lookup[thing_name_generic] = id(thing)

        # Start with exporting the mesh.
        mesh_name = thing_name_generic
        res = thing.result()
        if id(thing) not in thing_instance_counter:
            stl_file = target_dir / "assets" / (thing_name_generic + ".stl")
            stl_file.parent.mkdir(exist_ok=True, parents=True)
            stl_name = stl_file.stem
            if res is not None:
                res.scale(0.001).export_stl(str(stl_file))
                mesh_store.append(Element("mesh", {
                    "name":mesh_name,
                    "file":str(stl_file.relative_to(target_dir))
                    }))

        # Increment the counter, prepare further processing.
        thing_instance_counter[id(thing)] += 1
        thing_name_instance = f"{thing_name_generic}:{thing_instance_counter[id(thing)] - 1:03d}"

        # Export the material.
        material_name = thing.__material__.codename
        if material_name not in material_store:
            material_store[material_name] = Element(
                "material",
                name=material_name,
                rgba=" ".join(map(fmt, thing.__material__.color.rgba))
                )

        # Define the Thing itself.
        link_element = Element("body", {
            "name" : thing_name_instance,
            #"childclass" : "???",
            #"mocap" : "false",
            "pos" : " ".join(map(fmt_scale(0.001), where.position)),
            "euler" : " ".join(map(fmt, where.orientation)),
            "gravcomp" : "0" ,
            #"user" : "???" ,
        })

        if res is not None:
            # Export the dynamic properties.
            inertia, com = thing.matrix_of_inertia()
            inertial = Element("inertial", {
                "pos" : " ".join(map(fmt_scale(0.001), com)),
                #"euler" : "0 0 0",
                "mass" : fmt(thing.mass()),
                #"fullinertia" : " ".join(map(fmt, (inertia[0,0], inertia[1,1], inertia[2,2], inertia[0,1], inertia[0,2], inertia[1,2]))),
            })
            link_element.append(inertial)

            # Export the geometry.
            geom = Element("geom", {
                #"name": "full_body",
                #"class": "default_build123things_stl",
                "type": "mesh",
                "material": material_name,
                "mesh": mesh_name,
                })

            link_element.append(geom)
        else:
            inertial = Element("inertial", {
                "pos" : "0 0 0",
                "mass" : HACK_MINIMAL_INERTIA,
                "fullinertia" : " ".join(map(fmt, (HACK_MINIMAL_INERTIA, HACK_MINIMAL_INERTIA, HACK_MINIMAL_INERTIA, 0, 0, 0))),
            })
            link_element.append(inertial)

        # Now export other Things.
        for mounted_as, transform_resolver in thing.enumerate_assembly():
            assert isinstance(mounted_as, str)
            assert isinstance(transform_resolver, TransformResolver)
            if isinstance(transform_resolver._joint, Rigid):
                link_element.append(to_mjcf(transform_resolver._wrapped, transform_resolver.transform))
            else:
                intermediate_link = Element("body",
                    name=f"{thing_name_instance}:{mounted_as}",
                    pos=" ".join(map(fmt_scale(0.001), transform_resolver._orig_mount.location.position)),
                    euler=" ".join(map(fmt, transform_resolver._orig_mount.location.orientation)),
                )
                inertial = Element("inertial", {
                    "pos" : "0 0 0",
                    "mass" : HACK_MINIMAL_INERTIA, # NOTE: Hack to allow "empty" bodies.
                    "fullinertia" : " ".join(map(fmt, (HACK_MINIMAL_INERTIA, HACK_MINIMAL_INERTIA, HACK_MINIMAL_INERTIA, 0, 0, 0))),
                })
                intermediate_link.append(inertial)
                intermediate_link.append(to_mjcf(transform_resolver._joint, f"{thing_name_instance}:{mounted_as}"))
                intermediate_link.append(to_mjcf(transform_resolver._wrapped, bd.Location((0,0,0),(180,0,90)) * transform_resolver._next_mount.location.inverse())) # type: ignore
                link_element.append(intermediate_link)
        return link_element

    @to_mjcf.register
    def _(joint:Revolute, name:str) -> Element:
        if joint.global_name is not None:
            name = joint.global_name
        assert name not in joint_name_unique_check
        joint_name_unique_check.add(name)

        joint_element = Element("joint", {
            "name": name,
            #"class":"NOT USED, I export all explicitly to the MJCF."
            "type":"hinge",
            "group":"0",
            #"pos" : " ".join(map(fmt_scale(0.001), joint.stator_mount.position)),
            "axis" : "0 0 1",
            "springdamper" : "0 0",
            "limited" : "false" if joint.limit_angle is None else "true",
            "range" : "0 0" if joint.limit_angle is None else " ".join(map(fmt, joint.limit_angle)),
            "actuatorfrclimited" : "false" if joint.limit_effort is None else "true",
            "actuatorfrcrange" : "0 0" if joint.limit_effort is None else f"0 {fmt(joint.limit_effort)}",
            # TODO: Implement more of joint dynamics
            })
        return joint_element

    # Convert the Thing to xml.
    model_element = to_mjcf(thing, bd.Location())
    model_element.append(Element("freejoint"))

    # Prepare the document
    document_root = Element("mujoco", model=type(thing).__name__)
    # TODO: compiler_element = Element("compiler", angle="degree")


    # Assets
    asset_element = Element("asset")
    for m in material_store.values():
        asset_element.append(m)
    for m in mesh_store:
        asset_element.append(m)
    document_root.append(asset_element)

    # Robot itself
    if mujoco_module:
        document_root.append(model_element)
    else:
        worldbody_element = Element("worldbody")
        worldbody_element.append(model_element)
        document_root.append(worldbody_element)
        # TODO: Default simulation parameters from conf.

    # Wrap and return
    return ElementTree(document_root)

if __name__ == "__main__":
    import argparse
    import importlib
    from datetime import datetime

    EMPTY_ELEMENTS_FMT_LOOKUP = {
            "short": True,
            "s": True,
            "expand": False,
            "e": False,
    }

    argp = argparse.ArgumentParser(description="Export a Thing to ")
    argp.add_argument("module", help="Identify the module containing the thing.")
    argp.add_argument("thing", help="Identify the thing in the module.")
    argp.add_argument("--root-selector", "-r", default=None, help="A string which selects a Thing to be considered a new root of the system. By default the same as in build123things definition.")
    argp.add_argument("--root-joint", "-j", default="free", help="How to connect the exprt-wise root")
    argp.add_argument("--param-file", "-p", default=None, type=Path, help="Yaml file containing args and kwargs to pass to thing constructor. Expects two documents (separated by ---) in the file, the first with array (args) and the second with dict (kwargs).")
    argp.add_argument("--target-dir", "-d", default=None, type=Path, help="Target directory.")
    argp.add_argument("--worldbody", "-w", action="store_true", help="Export as self-standing MuJoCo model with the worldbody element and simulation stubs. (If absent, a MuJoCo embeddable )")
    argp.add_argument("--empty-elements-fmt", "-e", type=str, choices=EMPTY_ELEMENTS_FMT_LOOKUP.keys(), default="short")
    args = argp.parse_args()

    pymod = importlib.import_module(args.module)
    thing_cls = getattr(pymod, args.thing)
    if args.param_file is None:
        thing:Thing = thing_cls()
    else:
        raise NotImplementedError("TODO: Parse the yaml and instantiate.")

    if args.target_dir is None:
        target_dir:Path = Path.cwd() / "build" / (datetime.now().isoformat()[:19].replace(":","-") + "-" + thing.codename())
    else:
        target_dir:Path = args.target_dir
    target_dir.mkdir(parents=True, exist_ok=True)

    etree = export(thing, target_dir, not args.worldbody)
    xml.etree.ElementTree.indent(etree)
    etree.write(target_dir / "robot.mjcf", xml_declaration=True, short_empty_elements=EMPTY_ELEMENTS_FMT_LOOKUP[args.empty_elements_fmt])

