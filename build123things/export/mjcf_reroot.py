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
from scipy.spatial.transform import Rotation

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

def fmt_loc (x:build123d.Location, scale:float=1) -> dict[str,str]:
    return {
        "pos" : " ".join(map(fmt_scale(scale), x.position.to_tuple())),
        "euler" : " ".join(map(fmt, x.orientation.to_tuple())),
    }

NAMESPACE_SEPARATOR = ":"
HACK_MINIMAL_INERTIA = "0.001"

def export(thing:Thing, target_dir:Path) -> ElementTree:
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

    xml_meshes:dict[int,Element] = {}
    """ Maps Thing to XML representing the exported geometry. """

    xml_dynamics:dict[int,Element] = {}
    """ Maps Thing to XML representing the dynamics. """

    xml_materials:dict[str,Element] = {}
    """ Elements defining the meshes. """

    xml_by_id:dict[tuple[int,...],Element] = {(): Element("worldbody",name="")}
    """ `Thing.WalkReturnType.child_identifier` -> `Element` and joint-to-origin transform which needs to be applied to all childern."""

    for traversor in thing.walk():
        print(f"Traversing {traversor}")

        # Prepare and link the XMLs, fing the parent
        xml_parent = xml_by_id[traversor.parent_identifier]
        xml_child = Element("body")
        assert traversor.child_identifier not in xml_by_id
        xml_by_id[traversor.child_identifier] = xml_child
        xml_by_id[traversor.parent_identifier].append(xml_child)
        print(f"Current parent tag:\n{xml.etree.ElementTree.tostring(xml_parent)}")

        # Check that the codename is unique
        child_name = xml_parent.attrib["name"] + NAMESPACE_SEPARATOR + traversor.child.codename() + f"{count_thing_instances[id(traversor.child)]:03d}"
        xml_child.attrib.update(name=child_name)
        count_thing_instances[id(traversor.child)] += 1
        assert child_name not in check_thing_name_integrity or id(traversor.child) == check_thing_name_integrity[child_name]
        assert traversor.child.codename() not in check_thing_name_integrity or id(traversor.child) == check_thing_name_integrity[traversor.child.codename()]
        check_thing_name_integrity[child_name] = id(traversor.child)
        check_thing_name_integrity[traversor.child.codename()] = id(traversor.child)

        # Handle the parent-to-joint and child-to-joint transforms.
        if traversor.joint is not None and traversor.parent is not None:
            if isinstance(traversor.joint, Rigid):
                t1 = traversor.joint.get_other_mount(traversor.child).location
                t2 = build123d.Location((0,0,0),(180,0,90)) * traversor.joint.transform(
                        traversor.joint.get_other_mount(traversor.parent),
                        traversor.joint.get_other_mount(traversor.child) )
                t3 = traversor.joint.get_other_mount(traversor.parent).location.inverse()
                print(f"t1: {t1}")
                print(f"t2: {t2}")
                print(f"t3: {t3}")
                total_transf = t1 * t2 * t3
                print(f"tt: {total_transf}")
                xml_child.attrib.update(fmt_loc(total_transf, 0.001).items())
            elif isinstance(traversor.joint, Revolute):
                t1 = traversor.joint.get_other_mount(traversor.child).location
                t2 = build123d.Location((0,0,0),(180,0,90)) * traversor.joint.transform(
                        traversor.joint.get_other_mount(traversor.parent),
                        traversor.joint.get_other_mount(traversor.child) )
                t3 = traversor.joint.get_other_mount(traversor.parent).location.inverse()
                print(f"t1: {t1}")
                print(f"t2: {t2}")
                print(f"t3: {t3}")
                total_transf = t1 * t2 * t3
                print(f"tt: {total_transf}")
                xml_child.attrib.update(fmt_loc(total_transf, scale=0.001).items())

                child_to_joint = t3.inverse() * t2.inverse()
                print(f"tx: {child_to_joint}")
                child_to_joint_axis = child_to_joint.to_axis().direction
                xml_joint = Element("joint",
                    type="hinge" ,
                    pos=" ".join(map(fmt_scale(0.001), child_to_joint.position)),
                    axis=" ".join(map(fmt, child_to_joint_axis)),
                    #springdamper = "0 0",
                    #limited = "false" if traversor.joint.limit_angle is None else "true",
                    #range = "0 0" if traversor.joint.limit_angle is None else " ".join(map(fmt, traversor.joint.limit_angle)),
                    #actuatorfrclimited = "false" if traversor.joint.limit_effort is None else "true",
                    #actuatorfrcrange = "0 0" if traversor.joint.limit_effort is None else f"0 {fmt(traversor.joint.limit_effort)}",
                )
                if traversor.joint.global_name is not None:
                    xml_joint.attrib.update(name = traversor.joint.global_name)
                xml_child.append(xml_joint)
            else:
                raise NotImplementedError()

        # Export the mesh.
        mesh_name = traversor.child.codename()
        if id(traversor.child) not in xml_meshes:
            res = traversor.child.result()
            if res is not None:
                res = res.scale(0.001)
                stl_file = target_dir / "assets" / (mesh_name + ".stl")
                stl_file.parent.mkdir(exist_ok=True, parents=True)
                res.export_stl(str(stl_file))
                xml_meshes[id(traversor.child)] = Element("mesh", {
                    "name":mesh_name,
                    "file":str(stl_file.relative_to(target_dir))
                })
        if id(traversor.child) in xml_meshes:
            rgba = list(traversor.child.__material__.color.rgba)
            rgba[-1] = 1
            xml_child.append(Element("geom",
                type="mesh",
                mesh=mesh_name,
                rgba=" ".join(map(fmt, rgba))
            ))

        # export the dynamic properties.
        if id(traversor.child) not in xml_dynamics:
            res = traversor.child.result()
            if res is not None:
                inertia, com = thing.matrix_of_inertia()
                xml_inertial = Element("inertial", {
                    "pos" : " ".join(map(fmt_scale(0.001), com)),
                    #"euler" : "0 0 0",
                    "mass" : fmt(thing.mass()),
                    #"fullinertia" : " ".join(map(fmt, (HACK_MINIMAL_INERTIA, HACK_MINIMAL_INERTIA, HACK_MINIMAL_INERTIA, 0, 0, 0))),
                    #"fullinertia" : " ".join(map(fmt, (inertia[0,0], inertia[1,1], inertia[2,2], inertia[0,1], inertia[0,2], inertia[1,2]))),
                })
            else:
                xml_inertial = Element("inertial", {
                    "pos" : "0 0 0",
                    "mass" : HACK_MINIMAL_INERTIA,
                    "fullinertia" : " ".join(map(fmt, (HACK_MINIMAL_INERTIA, HACK_MINIMAL_INERTIA, HACK_MINIMAL_INERTIA, 0, 0, 0))),
                })
            xml_dynamics[id(traversor.child)] = xml_inertial
        xml_child.append(xml_dynamics[id(traversor.child)])

    assets = Element("asset")
    for _, v in xml_meshes.items():
        assets.append(v)
    for _, v in xml_materials.items():
        assets.append(v)
    ret = Element("mujoco")
    ret.append(assets)
    worldbody = xml_by_id[()]
    worldbody.attrib = {}
    ret.append(worldbody)

    return ElementTree(ret)

if __name__ == "__main__":

    # start the program
    import argparse
    import importlib
    from datetime import datetime
    empty_elements_fmt_lookup = {
            "short": True,
            "s": True,
            "expand": False,
            "e": False,
    }
    argp = argparse.ArgumentParser(description="export a thing to `mjcf` xml.")
    argp.add_argument("module", help="identify the module containing the thing.")
    argp.add_argument("thing", help="identify the thing in the module to export.")
    argp.add_argument("--root-selector", "-r", default=None, type=str, help="an expression to select a single specific sub-thing to use as the future root.")
    argp.add_argument("--root-joint", "-j", default="free", help="how to connect the exprt-wise root")
    argp.add_argument("--param-file", "-p", default=None, type=Path, help="yaml file containing args and kwargs to pass to thing constructor. expects two documents (separated by ---) in the file, the first with array (args) and the second with dict (kwargs).")
    argp.add_argument("--target-dir", "-d", default=None, type=Path, help="target directory.")
    #argp.add_argument("--worldbody", "-w", action="store_true", help="export as self-standing mujoco model with the worldbody element and simulation stubs. (if absent, a mujoco embeddable )")
    argp.add_argument("--empty-elements-fmt", "-e", type=str, choices=empty_elements_fmt_lookup.keys(), default="short", help="in xml, empty tags (with no content) may be shortened to <tag /> or left as <tag></tag>")
    args = argp.parse_args()

    # build the thing and select root
    pymod = importlib.import_module(args.module)
    thing_cls = getattr(pymod, args.thing)
    if args.param_file is None:
        thing:Thing = thing_cls()
    else:
        raise NotImplementedError("todo: parse the yaml and instantiate.")
    print(f"created a thing\n{thing}:")
    if isinstance(args.root_selector, str):
        selectors = args.root_selector.split(".")
        assert len(selectors) > 0
        for selector in selectors:
            assert selector.isidentifier()
            thing = getattr(thing, selector)
        assert isinstance(thing, TransformResolver)
        thing = thing.wrapped
        print(f"selected the thing.{'.'.join(selectors)} as the root:\n{thing}")
    else:
        assert args.root_selector is None
        print("the thing is regarded as the export root.")

    # do the export
    if args.target_dir is None:
        target_dir:Path = Path.cwd() / "build" / (datetime.now().isoformat()[:19].replace(":","-") + "-" + thing.codename())
    else:
        target_dir:Path = args.target_dir
    target_dir.mkdir(parents=True, exist_ok=True)
    etree = export(thing, target_dir)
    xml.etree.ElementTree.indent(etree, space="  ")
    etree.write(target_dir / "robot.mjcf", xml_declaration=True, short_empty_elements=empty_elements_fmt_lookup[args.empty_elements_fmt])










