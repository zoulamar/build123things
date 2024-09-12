#  The    B U I L D    1 2 3    T H I N G S    Library.

This package contains an extension to the brilliant [build123d](https://github.com/gumyr/build123d), a Pythonic scripted-CAD system based on [Open CASCADE Technology](https://dev.opencascade.org/doc/overview/html/index.html).
In `build123things`, user designs are subclasses of `Thing`, each such subclass represents a parametric family of objects, i.e., components.
With the library, semantically related `build123d` objects are groupped in `Thing` instances with following provided functionality and features:

- Reference geometry is managed as attributes, mitigating the Topological Naming Problem.
- Adhering to DRY - Don't Repeat Yourself.
- CAD modeling semantics mapped to object-oriented paradigm
    - Model parameters are mapped to `__init__` args.
    - Model specification, simplification or parameter modification via subclassing.
    - Reference geometries and init parameters as attributes.
- Explicit semantics of the object on many levels:
    - Assembly assumes strict hierarchical assembly directed acyclic graph.
    - Reference geometry is present with each Thing and annotated with language-compatible docstrings.
    - Derived designs track the pedigree in object-oriented inheritance.
    - Thing parameters are annotated and assume meaning on their own.
    - Joints have extensible semantics with arbitrary joint transform parametrization.
- Cloning existing complex geometry with incrementally adjusted parameters.
- Safety checks all around not to accidentally mess things up.
- Modular exporting of all subcomponents to STL, MuJoCo files or assembly graphs.
- Visualization utilities to distinguish different subcomponents

The library may be seen as a distillation of conventions.
Of course everyone may break this library easily by hacking the internals of provided classes.
The library was developed using Pyright LSP.
The `build123things` library goes well with [CQ-editor fork](https://github.com/jdegenstein/jmwright-CQ-Editor).
If CQ-editor won't render, try deleting `~/.config/CadQuery/CQ-editor.conf` or sth like that. Or add current dir to PYTHONPATH.

## Installation

So far, I do not know any better way than creating a new (mini)conda environment and installing it there.
This library is not yet deployed in `pip`, the recommended way of installing is via the [VCS pip support](https://pip.pypa.io/en/stable/topics/vcs-support/).
Ordinary system-wide installation tended to fail.
The library was developed with Python 3.11.4.

```
conda create -n build123things python=3.11.4
pip3 install git+https://github.com/comrob/build123things
```

## Getting Started

First, if you do not know already, get familiar with [build123d](https://github.com/gumyr/build123d) which is the backbone for defining geometries.
The `build123things` is an overlay which manages the `build123d` geometries while providing other utilities.
To start with your design, create a class with `Thing` as its superclass, e.g.,
```python
class MyDesign (Thing):
    def __init__ (self, param_1:float=5):
        self.reference_geometry = build123d.Sphere(radius=param_1)
        self.result_geometry = build123d.Box(param_1, param_1, param_1)
    def result(self):
        return self.result_geometry
```
Here, we defined a cube design with a reference bounding sphere.
Please, see examples provided with the library to learn about assemblies and more.
Visualize the result via `cq-editor`.
Export the designs using, e.g., ``python3 -m build123things.export.assembly_graph very_simple_car VerySimpleCar``.

## Scientific Publications

This library is a considered a supplementary material for a manuscript submitted for the The 2025 IEEE International Conference on Robotics & Automation (ICRA 2025).
In case of acceptance, we will provide here a preferred bibtex record.

