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
- Modular exporting of all subcomponents to STL, MuJoCo files or assembly graphs.
- Visualization utilities to distinguish different subcomponents

## Installation

```
pip3 install git+https://github.com/zoulamar/build123things
sudo apt install graphviz # optional, needed by assembly graph exporter
```

Tested on Ubuntu 24.04 with Python 3.12.3.
We recommend installing the library in an isolated virtual environment.


## Getting Started

First, if you do not know already, get familiar with [build123d](https://github.com/gumyr/build123d) which is the backbone for defining geometries.
The `build123things` is an overlay which manages the `build123d` geometries while providing the mentioned utilities.
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

To visualize the result, run `python3 -m ocp_vscode` in a separate terminal and open the indicated webpage in a browser of your choice.
(Tested on Chromium 135.0.7049.52 snap.)
Then execute a Python call `build123things.show.show(my_design_instance)`; the rendering in the browser happens behind the scenes.
To export your design, use, e.g., `python3 -m build123things.export.mjcf serial_manip LinkBase`;
the resulting support files for the MuJoCo simulator to be found in `build` subdirectory.

