"""
Exports the Thing's inheritance diagram and intermediate parametrizations.
"""

from abc import ABC
from datetime import datetime
from functools import singledispatch
import inspect
from pathlib import Path
from typing import Any

from build123d import Location
from build123things import ORIG_INIT_REFERENCES, ReferenceTransformResolver, Thing, TransformResolver
import graphviz

from build123things.materials import Material

def export(thing:type[Thing]) -> graphviz.Digraph:
    graph:graphviz.Digraph = graphviz.Digraph(
        name=f"Subassembly of {thing.__name__}",
        engine="dot",
        graph_attr={
            "overlap_scaling":"40",
            "rankdir":"TB",
            "margin":"0",
        },
        node_attr={
            "style":"filled",
            #"fillcolor":"transparent",
            #"fixedsize":"true",
            #"width":"3",
            #"height":"3",
            "fontsize":"20",
            "fontname":"Times",
            "shape":"box",
            #"margin":"0",
        },
        edge_attr={
            #"weight":"0.01",
            #"color":"red",
            }
    )

    # TODO Style defined here.
    #graph.attr("color", "green")

    def fmt(s):
        if isinstance(s, type):
            s = s.__name__
        if isinstance(s, Material):
            s = s.codename
        else:
            s = str(s)
        return s.replace("<", "").replace(">", "")

    def chunks(lst, n):
        """Yield successive n-sized chunks from lst."""
        for i in range(0, len(lst), n):
            yield lst[i:i + n]

    type_store:dict[str,type] = {}
    def process(cls:type, next_name:str|None) -> None:
        if cls is object or cls is ABC: return
        name:str = cls.__name__

        init = ORIG_INIT_REFERENCES[cls]
        print()

        args = []
        for pname, pobj in inspect.signature(init).parameters.items():
            if pname == "self": continue
            if pobj.default is inspect._empty:
                ddf = ""
            else:
                ddf = " = " + fmt(pobj.default)
            args.append(f"{pname}:{fmt(pobj.annotation)}{ddf}")
        attr:dict[str,str] = {
                # "xlabel":"Blabla",
        }
        if next_name is None:
            attr["fillcolor"] = "lightgreen"
        elif cls is Thing:
            attr["fillcolor"] = "lightblue"
        else:
            attr["fillcolor"] = "lightgrey"

        attr_lines = []
        for chunk in chunks(args, 4):
            attr_lines.append(", ".join(chunk))
        attr_res = "\n".join(attr_lines)

        label:str = f"""Class {name} with parameters {attr_res}"""
        if name not in type_store:
            type_store[name] = cls
            graph.node(name,label,attr)
        if next_name is not None:
            graph.edge(next_name, name)
        print(f"MRO: {cls.__mro__}")
        for supercls in cls.__mro__[1:2]:
            print(f"Export {supercls}")
            process(supercls, name)
    process(thing, None)
    return graph

if __name__ == "__main__":
    import argparse
    import importlib

    argp = argparse.ArgumentParser(description="Export a Thing to ")
    argp.add_argument("module", help="Identify the module containing the thing.")
    argp.add_argument("thing", help="Identify the thing in the module.")
    argp.add_argument("--target-dir", "-d", default=None, type=Path, help="Target directory.")
    argp.add_argument("--style", "-s", default=None, type=Path, help="Yaml file with DOT style specs.")
    argp.add_argument("--format", "-f", nargs="+", default=["pdf"], type=str, help="Yaml file with DOT style specs.")
    args = argp.parse_args()

    pymod = importlib.import_module(args.module)
    thing_cls = getattr(pymod, args.thing)
    graph = export(thing_cls)

    if args.target_dir is None:
        target_dir = Path.cwd() / "build" / (datetime.now().isoformat()[:19].replace(":","-") + "-" + thing_cls.__name__)
    else:
        target_dir = args.target_dir
    target_dir.mkdir(parents=True, exist_ok=True)
    assert len(list(target_dir.glob("*"))) == 0, f"The model {repr(thing_cls)} was requested for export into {target_dir} which is not empty!"

    graph.render(directory=target_dir)

