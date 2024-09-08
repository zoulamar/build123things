This file contains gathered ideas which could be one day opened in the issue tracker.

# Document the possibility and possible issues with having a forest-of-Things.
Can some sub-Thing safely be used as part of more different Things? How to distinguish it during graph traversal.
- Introduce Thing domains - each Thing will remember which subassemblies (assembly roots) it is member of. If a walk is issued, it will stay in the same/selected domain.

# Try running it in [Codon](https://github.com/exaloop/codon)
Promissing computational performance boost.

# A collection-like attribute results in double transform.
Having a Thing which aggregates some of its (mounted!) properties in a list-like property (or generator) makes subsequent accesses to such listed properties (through this list) receiving the transform twice.

# Memoization is a key tool used in the library, yet is currently extremly naive and inefficient.
Some day it would be nice to improve the string-based current approach.
Use generic memoization as the for lazy computation. Maybe even make the memoization of `.result` the default behavior. (Hijack it...)

# How to systematically access location of mount points?
- When accessed, a TransformResolver is returned. However, there is no

# MuJoCo export joints use pos properly.
Should resolve currently used hack with zero-mass virtual body.

# Assemble to Nested Mount Points.

Currently, the mount points need to be "brought" to the Thing's reference frame in order to be used in assembly.
Allow more over-the-joint references.

# Allow Declarative Assemblies?

Currently, we need to keep strict hierarchically encapsulated structure, i.e., LinkBase is in fact LinkBaseAndSubassembly.

# Automated Part Version Identification?

Internally call `git blame` to automatically identify a version of the Thing as the commit hash of the source file.

# Unify Assembly and Reference Geometry Syntax?

This is a rather conceptual decision.
I.e., everything will be assemblies, but with a binary flag whether it is assembled "for-reference".
Will require cycle detection.

# Headless Rendering.

Investigate how to invoke rendering in some driving script, maybe even headless. (Directly to file.)

# Internal Structures Modeling.

According to SOTA, there is a problem of representing internal structures.
These are often dependent on manufacturing and in practice implementation-dependent.

# Multi-Resolution / Level-of-Detail Management.

The component to be represented in a multiple-resolution manner.
E.g., a servomotor may be either assumed as a single solid body with inertia or an assembly of stator, gear and motor all the way down to discrete electrical parts.
How to address this systematically?

## Conventionalize another class-inheritance semantics?

- Subclasses with registered "zoom-in" or "zoom-out" semantics. The zoom-out would be e.g. low-poly collision model.
- A part could have these levels of predefined detail:
    - atomic (of course infeasible)
    - molecular (of course infeasible)
    - total discrete (each nameable part fully specified)
    - mechanical
    - visual
    - collision (parametrized by precision)
    - bounding box

I could imagine
```python3
class Thing:
  def lod_zoom_out() -> Thing: # Creates another Thing representing the same but with rougher LOD.
  def lod(which:LOD_ENUM) -> Thing: # Creates another Thing representing the same but with rougher LOD.
class Dynamixel (Thing):
  THING_LOD = Thing.Visual # Class variable defines which Level of Detail (LOD) this class specifies.
class Dynamixel_collision (Dynamixel):
  THING_LOD = Thing.Collision
```

## Variable LoD via Lazy Evaluation?

Given component will have all the level of detail declared as attributes to the fullest detail.
However, the attributes will not be computed until accessed. Then, such parts will be cached.
One big issue: How to address the macroscopic properties like mass or inertia tensor?
It can be either declared ad-hoc or based on rough model, but it would not match the finest-detail computation which is, however, expensive.

Moreover, how to address automated graph traversal, when to cut the recursion?

# Generalized Interfaces.

It is feasible to attach a Thing only at particular locations.
Such attachments often require some degree of consistency, e.g., you cannot fit M4 screw to M2 hole.
Or, you may attach a wheel to a rod only one way, not to collide with the car.
An interface would be a mechanism to assert such constraints, perhaps both on declarative (logical) level and on geometic level (not colliding).
Maybe even basic mechanical properties might be checked?

Programatically, an interface might be a nestesd class instance compatible with bd.Location, moreover checking whatever needs to be checked.
It sounds to me like a generalization of current Mount point mechanism.

Interaface declares
- where (reference location)
- how (type of supported relative motion)
- what (compatibility, logical or spatial constraints)

# Constraints.

Augment the current assembly tree with constraints which turn it into a generic graph.
Constraints may be simply listed in some global registry? (Plus, do we need some global registry of Things?)

## Allow designer to provide collision models.

(See also level-of-detail task.)

- Find a way to inject specific to_mjcf overridings or adjustments, providing collision boxes and such.
- A simple way would be having to_mjcf methods.
- Nicer way might be extending the singledispatch.

# Improve Inertia Matrices computation.

Currently, a nasty hack is created: A library is used to voxelize the Thing and consequenly compute the inertia there.

## Implementation possibilities.

- Currently selected method: Export to meshes and compute and combine inertial matrices with external tool.
  - [see how easy it is](https://computationalmechanics.in/obtaining-volume-centre-of-gravity-and-inertia-matrix-from-stl-file/)
  - (use python-stl package)
  - It will be definitely with some overheads both in speed (translation and meshization) adn accuracy (meshization)
  - Maybe it uses [voxelization?](https://www.reddit.com/r/Python/comments/orxw63/finding_the_moment_of_inertia_tensor_for_any_3d/?rdt=64653)
  - Yes, this is super easy and definitely a way to go now.

- If fails, ... build123d does not seem to have anything useful:
  - [Center of mass via buid123d.Shell](https://build123d.readthedocs.io/en/latest/direct_api_reference.html#topology.Shell.center)
  - [Center of mass via some build123d "builder" interface](https://build123d.readthedocs.io/en/latest/center.html#cad-object-centers)
  - Perhaps the OCP? Might be, but who knows...

- Take the original OCCT source and try to minimalistically bind it using Boost Pyton binder?
  - Afraid it will be too much work... Yet on the other hand, perhaps there would not be that much work...
  - I guess the efficiency would not suffer.
  - I would more or less expose the OCCT 1-to-1.
  - Inertia computation basically for free.
  - I'd need to reimplement a lot of build123d conventions...
  - https://dev.opencascade.org/doc/refman/html/class_b_rep_g_prop.html

- Some inertia computators which depend on xacro and whatnot...
  - https://github.com/vonunwerth/MeshLabInertiaToURDF
  - https://github.com/gstavrinos/calc-inertia

# Improve and Enhance Material Class.

The class would carry all relevant macroscopic material properties, useful for subsequent simulations...

# Pack as a NIX package.

[NIX](https://nixos.org/) is cool.

# Unit analysis libraries

Unfortunately, the library is not written in strongly typed language like C++ where unit conversion would be a piece of cake.
So far, the library assumes a generic single unit of measure, typically millimeters.
However, it would be great if any unit of measure could work.
Altough I found Python libraries for this purpose, still, after all, I gave up for now.
- One external existing list here: https://kdavies4.github.io/natu/seealso.html
- [Pint](https://pint.readthedocs.io/en/stable/)
    - It integrates with NumPy as what they say. It will integrate with everything.
    - Although it has a bit of boiler plate
    - *however, broken at the time*
- [Unyt](https://unyt.readthedocs.io/en/stable/)
    - Very promissing, alive, starred, seems mature.
    - Not working out of the box.
- [Unum - Unit numbers](https://github.com/trzemecki/Unum)
    - Multiplies the number with unit class, seems mature.
- [SymPy Units](https://docs.sympy.org/latest/modules/physics/units/)
    - possible...
- [Astropy](https://docs.astropy.org/en/stable/units/)
    - Likely depends on whole moloch, better alternatives exist
- [Barril](https://github.com/ESSS/barril)
    - Seems live, but syntax lacks nice sugar.
- [Axiompy](https://github.com/ArztKlein/Axiompy)
    - naive
- [Quantiphy](https://quantiphy.readthedocs.io/en/stable/)
    - Just playing with strings it seems.
- [Scipy Constants](https://docs.scipy.org/doc/scipy/reference/generated/scipy.constants.unit.html)
    - Different use / translates string descriptions.
- Obscure or dead
    - https://github.com/mdipierro/buckingham
    - https://bitbucket.org/adonohue/units/
    - https://github.com/blazetopher/udunitspy
    - https://github.com/kdavies4/natu
    - https://docs.sympy.org/dev/modules/physics/units.html
    - http://russp.us/scalar-guide.htm
    - http://www.inference.org.uk/db410/
    - https://bitbucket.org/adonohue/units/
- Not investigated further
    - https://scimath.readthedocs.io/en/latest/units/intro.html
    - https://pypi.org/project/quantities/
    - https://pypi.org/project/numericalunits/
    - http://juanreyero.com/open/magnitude/
- [ForAllPeople](https://github.com/connorferster/forallpeople)
    - Seems promissing

# Manufacturing Hints.

An annotation which states, e.g., how the Thing should be 3D printed, from which material, ...

# GUI with explicit dependency and reference geometry visualization and management.

# What if I want to use a servo twice, first mounted by stator and once by rotor?
