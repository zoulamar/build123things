"""
Exports the Thing as a Directed Assembly Graph into a DOT file for further visualization.
"""

from datetime import datetime
from functools import singledispatch
from pathlib import Path
from typing import Any

from build123d import Location
from build123things import ReferenceTransformResolver, Thing, TransformResolver
import graphviz

def export(thing:Thing, include_reference=False) -> graphviz.Digraph:
    graph:graphviz.Digraph = graphviz.Digraph(
        name=f"Subassembly of {thing.codename()}",
        engine="dot",
        graph_attr={
            "mclimit":"10",
            "rankdir":"LR",
            "margin":"0",
        },
        node_attr={
            "style":"filled",
            "fillcolor":"transparent",
            #"fixedsize":"true",
            #"width":"3",
            #"height":"3",
            "fontsize":"20",
            "fontname":"Times",
            "shape":"box",
        },
        edge_attr={
            #"weight":"0.01",
            #"color":"red",
            }
    )

    # TODO Style defined here.
    #graph.attr("color", "green")

    nodes:dict[str,Thing] = {}

    def process(tr:TransformResolver, as_name:str, as_construction:bool=False):
        name = str(id(tr.wrapped))
        label = tr.wrapped.codename()

        if name not in nodes:
            nodes[name] = tr.wrapped
            graph.node(name,label,attr="val")
            recurse = True
        else:
            recurse = False

        if tr.previous is not None:
            prev_name = str(id(tr.previous.wrapped))
            if as_construction:
                graph.edge(prev_name, name, as_name, style="dotted")
            else:
                graph.edge(prev_name, name, as_name)

        if recurse:
            for nm, nxt in tr.enumerate_assembly():
                process(nxt, as_construction=False, as_name=nm)
            if include_reference:
                for nm, nxt in tr.enumerate_assembly(subassembly=False, reference=True):
                    print(nm, nxt)
                    process(nxt, as_construction=True, as_name = nm)

    process(ReferenceTransformResolver(thing, where=Location(), previous=None), "")

    return graph

if __name__ == "__main__":
    import argparse
    import importlib

    argp = argparse.ArgumentParser(description="Export a Thing to ")
    argp.add_argument("module", help="Identify the module containing the thing.")
    argp.add_argument("thing", help="Identify the thing in the module.")
    argp.add_argument("--param-file", "-p", default=None, type=Path, help="Yaml file containing args and kwargs to pass to thing constructor. Expects two documents (separated by ---) in the file, the first with array (args) and the second with dict (kwargs).")
    argp.add_argument("--target-dir", "-d", default=None, type=Path, help="Target directory.")
    argp.add_argument("--style", "-s", default=None, type=Path, help="Yaml file with DOT style specs.")
    argp.add_argument("--format", "-f", nargs="+", default=["pdf"], type=str, help="Yaml file with DOT style specs.")
    args = argp.parse_args()

    pymod = importlib.import_module(args.module)
    thing_cls = getattr(pymod, args.thing)
    if args.param_file is None:
        thing = thing_cls()
    else:
        raise NotImplementedError("TODO: Parse the yaml and instantiate.")

    graph = export(thing)


    if args.target_dir is None:
        target_dir = Path.cwd() / "build" / (datetime.now().isoformat()[:19].replace(":","-") + "-" + type(thing).__name__)
    else:
        target_dir = args.target_dir
    target_dir.mkdir(parents=True, exist_ok=True)
    assert len(list(target_dir.glob("*"))) == 0, f"The model {repr(thing)} was requested for export into {target_dir} which is not empty!"

    graph.render(directory=target_dir)




