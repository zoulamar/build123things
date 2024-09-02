"""
Implements exporting a Thing to MuJoCo file format, but with the possibility to select the model root arbitrarily.
"""

from collections import defaultdict
from pathlib import Path
from typing import Any, Generator

import build123d
from build123d.joints import RigidJoint
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

NAMESPACE_SEPARATOR = ":"

HACK_MINIMAL_INERTIA = "0.00001"
""" A value of inertia to assingn to virtual bodies.

Currently, the MJCF is created verbosely. Therefore, there is a need for virtual bodies/reference frames as the 'child' body is represented w.r.t. its original Origin, not w.r.t. the interface joint.

"""

def export(thing:Thing, target_dir:Path, mujoco_module:bool=False) -> ElementTree:
    """ The main function.
    TODO: Reformulate all bodies to origins as required by MuJoCo.
    TODO: Fuse rigidly bound bodies - eliminate implicit rigid welds.
    NOTE: during iteration, use len(edge) to identify depth, thus track the DFS.
    """

    count_thing_instances:dict[int,int] = defaultdict(int)
    """ Lookup of already encountered Things; each Thing retains a count of how many times it is inthe design. """

    check_thing_names_duplicates:set[str] = {}
    """ Just to ensure that multiple Things do not share the same name. """

    check_joint_names_duplicates:set[str] = set()
    """ Just to ensure each joint is exported only once. """

    xml_mesh:dict[int,Element] = {}
    """ Elements defining the meshes. """

    xml_material:dict[int,Element] = {}
    """ Elements defining the meshes. """

    xml_by_id:dict[tuple[int,...],tuple[Element,build123d.Location]] = {(): (Element("root"), build123d.Location())}
    """ `Thing.WalkReturnType.child_identifier` -> `Element` and joint-to-origin transform which needs to be applied to all childern."""

    for traversor in thing.walk():

        # Extract the source data
        xml_parent, parent_pretransform = xml_by_id[traversor.parent_identifier]
        if traversor.joint is None:
            xml_joint = None
            attr_pos = parent_pretransform * build123d.Location()
            child_pretransform = build123d.Location()
        else:
            assert traversor.parent is not None
            if isinstance(traversor.joint, Rigid):
                xml_joint = None
            elif isinstance(traversor.joint, Revolute):
                xml_joint = Element(tag="joint", attrib=dict(type="hinge",axis="0 0 1"))
            attr_pos = parent_pretransform * traversor.joint.get_other_mount(traversor.child).location
            child_pretransform = traversor.joint.get_other_mount(traversor.parent).location.inverse()

        # Assemble the child XML element
        xml_child = Element(tag="body", attrib=dict())
        ... # TODO WIP

        # Track the body to the parent.
        assert traversor.child_identifier not in xml_by_id
        xml_by_id[traversor.child_identifier] = xml_child
        xml_by_id[traversor.parent_identifier].append(xml_child)

    return ElementTree()

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

