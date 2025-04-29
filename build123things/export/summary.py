"""
Exports the Thing's inheritance diagram and intermediate parametrizations.
"""

from abc import ABC
from collections import defaultdict
from datetime import datetime
from pprint import pprint
import inspect
from pathlib import Path
from typing import Any

from build123d import Location
from build123things import ORIG_INIT_REFERENCES, ReferenceTransformResolver, Thing, TransformResolver
import graphviz

from build123things.materials import Material

def export(thing:Thing):
    counter = defaultdict(int)
    total = 0
    def process(t:Thing) -> None:
        nonlocal total
        counter[t] += 1
        total += 1
        for _, v in t.enumerate_assembly():
            process(v.wrapped)
    process(thing)
    return total, counter

if __name__ == "__main__":
    import argparse
    import importlib

    argp = argparse.ArgumentParser(description="Export a Thing to ")
    argp.add_argument("module", help="Identify the module containing the thing.")
    argp.add_argument("thing", help="Identify the thing in the module.")
    argp.add_argument("--param-file", "-p", default=None, type=Path, help="Yaml file containing args and kwargs to pass to thing constructor. Expects two documents (separated by ---) in the file, the first with array (args) and the second with dict (kwargs).")
    args = argp.parse_args()

    pymod = importlib.import_module(args.module)
    thing_cls = getattr(pymod, args.thing)
    if args.param_file is None:
        thing = thing_cls()
    else:
        raise NotImplementedError("TODO: Parse the yaml and instantiate.")

    pprint(Thing.list_of_all_existing_things)
    print(f"Total {len(Thing.list_of_all_existing_things)} unique components used in assembly or for reference.")

    total, counter = export(thing)
    pprint(counter)
    print(f"The assembly uses {total} components in total.")



