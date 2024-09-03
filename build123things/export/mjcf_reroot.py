"""
Implements exporting a Thing to MuJoCo file format, but with the possibility to select the model root arbitrarily.
"""

from collections import defaultdict
from pathlib import Path
from typing import Any
import build123d
from build123things import Thing, TransformResolver
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

def fmt_loc (x:build123d.Location) -> dict[str,str]:
    return {
        "pos" : " ".join(map(fmt, x.position.to_tuple())),
        "euler" : " ".join(map(fmt, x.orientation.to_tuple())),
    }

NAMESPACE_SEPARATOR = ":"

def export(thing:Thing, target_dir:Path, mujoco_module:bool=False) -> ElementTree:
    """ The main function.
    TODO: Reformulate all bodies to origins as required by MuJoCo.
    TODO: Fuse rigidly bound bodies - eliminate implicit rigid welds.
    NOTE: during iteration, use len(edge) to identify depth, thus track the DFS.
    """

    count_thing_instances:dict[int,int] = defaultdict(int)
    """ Lookup of already encountered Things; each Thing retains a count of how many times it is inthe design. """

    check_thing_name_integrity:dict[str,int] = {}
    """ Just to ensure that multiple Things do not share the same name. """

    check_joint_names_duplicates:set[str] = set()
    """ Just to ensure each joint is exported only once. """

    xml_meshes:list[Element] = []
    """ Elements defining the meshes. """

    xml_materials:dict[str,Element] = {}
    """ Elements defining the meshes. """

    xml_by_id:dict[tuple[int,...],Element] = {(): Element(tag="worldbody",attrib=dict(name=""))}
    """ `Thing.WalkReturnType.child_identifier` -> `Element` and joint-to-origin transform which needs to be applied to all childern."""

    for traversor in thing.walk():
        # Prepare and link the XMLs, fing the parent
        xml_parent = xml_by_id[traversor.parent_identifier]
        xml_child = Element(tag="body")
        assert traversor.child_identifier not in xml_by_id
        xml_by_id[traversor.child_identifier] = xml_child
        xml_by_id[traversor.parent_identifier].append(xml_child)

        # Check that the codename is unique
        child_name = xml_parent.attrib["name"] + NAMESPACE_SEPARATOR + traversor.child.codename() + f"{count_thing_instances[id(traversor.child)]:03d}"
        count_thing_instances[id(traversor.child)] += 1
        assert child_name not in check_thing_name_integrity or id(traversor.child) == check_thing_name_integrity[child_name]
        assert traversor.child.codename() not in check_thing_name_integrity or id(traversor.child) == check_thing_name_integrity[traversor.child.codename()]
        check_thing_name_integrity[child_name] = id(traversor.child)
        check_thing_name_integrity[traversor.child.codename()] = id(traversor.child)

        # Handle the parent-to-joint and child-to-joint transforms.
        if traversor.joint is not None and traversor.parent is not None:
            if isinstance(traversor.joint, Rigid):
                total_transf = build123d.Location() \
                    * traversor.joint.get_other_mount(traversor.child).location \
                    * traversor.joint.transform(
                        traversor.joint.get_other_mount(traversor.parent),
                        traversor.joint.get_other_mount(traversor.child) ) \
                    * traversor.joint.get_other_mount(traversor.parent).location
                for n, v in fmt_loc(total_transf).values():
                    xml_child.attrib[n] = v
            elif isinstance(traversor.joint, Revolute):
                parent_to_joint = build123d.Location() \
                    * traversor.joint.get_other_mount(traversor.child).location \
                    * build123d.Location((), (180,0,90))
                for n, v in fmt_loc(parent_to_joint).values():
                    xml_child.attrib[n] = v
                child_to_joint = traversor.joint.get_other_mount(traversor.parent).location.inverse()
                child_to_joint_axis = build123d.Location((0,0,0),child_to_joint.orientation.to_tuple()) * build123d.Vector(0,0,1)
                xml_joint = Element(tag="joint", attrib=dict(
                    type="hinge" ,
                    pos=" ".join(map(fmt, child_to_joint.position)),
                    axis=" ".join(map(fmt, child_to_joint_axis)),
                    springdamper = "0 0",
                    limited = "false" if traversor.joint.limit_angle is None else "true",
                    range = "0 0" if traversor.joint.limit_angle is None else " ".join(map(fmt, traversor.joint.limit_angle)),
                    actuatorfrclimited = "false" if traversor.joint.limit_effort is None else "true",
                    actuatorfrcrange = "0 0" if traversor.joint.limit_effort is None else f"0 {fmt(traversor.joint.limit_effort)}",
                ))
                if traversor.joint.global_name is not None:
                    xml_joint.attrib.update(name = traversor.joint.global_name)
                xml_child.append(xml_joint)
            else:
                raise NotImplementedError()

        # Export the material.
        material_name = thing.__material__.codename
        if material_name not in xml_materials:
            xml_materials[material_name] = Element(
                tag="material",
                name=material_name,
                rgba=" ".join(map(fmt, thing.__material__.color.rgba))
                )
        xml_child.append(xml_materials[material_name])

        # Export the mesh.
        mesh_name = traversor.child.codename()
        if id(thing) not in count_thing_instances:
            res = thing.result()
            stl_file = target_dir / "assets" / (mesh_name + ".stl")
            stl_file.parent.mkdir(exist_ok=True, parents=True)
            if res is not None:
                res.scale(0.001).export_stl(str(stl_file))
                xml_meshes.append(Element("mesh", {
                "name":mesh_name,
                "file":str(stl_file.relative_to(target_dir))
                }))

    return ElementTree(xml_by_id[()])

if __name__ == "__main__":

    # Start the program
    import argparse
    import importlib
    from datetime import datetime
    EMPTY_ELEMENTS_FMT_LOOKUP = {
            "short": True,
            "s": True,
            "expand": False,
            "e": False,
    }
    argp = argparse.ArgumentParser(description="Export a Thing to `mjcf` XML.")
    argp.add_argument("module", help="Identify the module containing the thing.")
    argp.add_argument("thing", help="Identify the thing in the module to export.")
    argp.add_argument("--root-selector", "-r", default=None, type=str, help="An expression to select a single specific sub-Thing to use as the future root.")
    argp.add_argument("--root-joint", "-j", default="free", help="How to connect the exprt-wise root")
    argp.add_argument("--param-file", "-p", default=None, type=Path, help="Yaml file containing args and kwargs to pass to thing constructor. Expects two documents (separated by ---) in the file, the first with array (args) and the second with dict (kwargs).")
    argp.add_argument("--target-dir", "-d", default=None, type=Path, help="Target directory.")
    argp.add_argument("--worldbody", "-w", action="store_true", help="Export as self-standing MuJoCo model with the worldbody element and simulation stubs. (If absent, a MuJoCo embeddable )")
    argp.add_argument("--empty-elements-fmt", "-e", type=str, choices=EMPTY_ELEMENTS_FMT_LOOKUP.keys(), default="short", help="In XML, empty tags (with no content) may be shortened to <tag /> or left as <tag></tag>")
    args = argp.parse_args()

    # Build the Thing and select root
    pymod = importlib.import_module(args.module)
    thing_cls = getattr(pymod, args.thing)
    if args.param_file is None:
        thing:Thing = thing_cls()
    else:
        raise NotImplementedError("TODO: Parse the yaml and instantiate.")
    print(f"Created a Thing\n{thing}")
    if isinstance(args.root_selector, str):
        selectors = args.root_selector.split(".")
        assert len(selectors) > 0
        for selector in selectors:
            assert selector.isidentifier()
            thing = getattr(thing, selector)
        assert isinstance(thing, TransformResolver)
        thing = thing.wrapped
        print(f"Selected the thing.{selectors} as the root:\n{thing}")
    else:
        assert args.root_selector is None
        print("The thing is regarded as the export root.")

    # Do the export
    if args.target_dir is None:
        target_dir:Path = Path.cwd() / "build" / (datetime.now().isoformat()[:19].replace(":","-") + "-" + thing.codename())
    else:
        target_dir:Path = args.target_dir
    target_dir.mkdir(parents=True, exist_ok=True)
    etree = export(thing, target_dir, not args.worldbody)
    xml.etree.ElementTree.indent(etree)
    etree.write(target_dir / "robot.mjcf", xml_declaration=True, short_empty_elements=EMPTY_ELEMENTS_FMT_LOOKUP[args.empty_elements_fmt])

