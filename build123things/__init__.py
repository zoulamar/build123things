#!/usr/bin/env python3

"""
An extension to the excelent `build123d` by Roger Maitland.
Before you start using this library, please, at least skim through https://github.com/gumyr/build123d
This module contains several utilities to enhance scripted CAD modeling using the said library.
As a starting point, see the docstring of `Thing` or the examples.
For further reference, please read our IROS 2024 paper.

The intent is to provide utilities for systematic and undistracted model coding, not to make hyper-efficient geometry computation.
Of course a good Pythonist can break this library easily.
(E.g., meddling with `__owner__` property or accessing `Thing.__dict__`.)
Perhaps consider this module as a collection of CONVENTIONS of working with the excelent `build123d` library.
"""

from ctypes import ArgumentError
from abc import ABC, ABCMeta, abstractmethod
from typing import NoReturn, Set, TypeAlias, Union, final, Any, Callable, Tuple, Generator
import build123d as bd
import numpy as np

from build123things.joints import Revolute
from .misc import memoize, random_tmp_fname
import colored
import copy
import inspect
from .materials import Material
import os
from stl import mesh
from warnings import warn

MOUNTING_LOCATION:bd.Location = bd.Location((0,0,0),(180,0,90))

DEBUG:Set[str] = set()
"List all methods which are supposed to have debugging enabled."
#DEBUG.add("Thing.__setattr__")
#DEBUG.add("Thing.__init__")
#DEBUG.add("Thing.__getattr__")
#DEBUG.add("Thing.adjust")
#DEBUG.add("TransformResolver.__getattr__")
#DEBUG.add("MountPoint.__init__")


class ThingMeta (ABCMeta):
    """ Facilitates the parameter capture functionality and the memoization.

    The __call__ method is called before even creating any objects of the future Thing instance.
    Hence, we may safely check if an instance with the very same parameters does not already exist.
    If so, the existing instance is used. (The very generic memoization is used.)

    If no existing instance exists, a new one is created and automatically fitted with the arguments which were passed there.
    """

    @memoize
    def __call__(cls, *args: Any, **kwds: Any) -> Any:
        """ This intercepts the class instantiation and checks the parameters in the cache. If found, reference to existing instance is returned. """
        ret = super(ThingMeta, cls).__call__(*args, **kwds)
        if "ThingMeta.__call__" in DEBUG:
            print(f"The __call__ returned: {ret}")
        return ret

class ParameterResolver:
    """ Class that can traverse the parameters in a Thing's inheritance hierarchy, finding the one conveniently via attribute access. """

    def __init__(self, thing:"Thing") -> None:
        self.thing = thing
        self.__parameters__ = thing.__parameters__

    def __getattr__(self, __name:str) -> float:
        for ns in self.__parameters__:
            try:
                return ns[1][__name]
            except:
                continue
        raise ArgumentError(f"Parameter {__name} not known in {self.thing}")

class MountPoint:
    """ Wraps a location in a `Thing`'s reference frame for the purpose of mounting other `Thing`s via `AbstractJoint`s.
    Stores references to all such mounted joints.
    This is a more explicit, declarative approach to assembly definitions.
    Previously, the attached Thing was unable to know where it is attached to, making further reasoning impossible.
    """
    def owned(self) -> bool:
        return self._owner is not None and self._known_as is not None

    def __init__(self, location:"bd.Location|MountPoint", owner:"Thing|None"=None, known_as:str|None=None) -> None:
        if isinstance(location, MountPoint):
            location = location.location
        elif isinstance(location, TransformResolver):
            location = location.location
        assert isinstance(location, bd.Location), str(location) + str(type(location))
        assert isinstance(owner, Thing|None)
        assert isinstance(known_as, str|None)
        if "MountPoint.__init__" in DEBUG:
            print(f"Thing.__init__ {repr(self)} with {location}")

        self._owner:Thing|None = owner
        """ A thing instance for which this mount point is defined. """
        self._known_as:str|None = known_as
        """ An attribute name in the `self.owner` which leads to `self`. """
        self._location:bd.Location = location
        """ The numerical definition of the position of this MountPoint in the reference frame of `self.owner`. """
        self._joint_outbound:AbstractJoint|None = None
        """ A mount point is required to host only one outbound joint.
        The owner is the root of subassembly containg the referenced thing,
        If, for some reason, multiple things need to be attached to a single location, may the user declare two separate mount points referencing single location.
        This might be useful, e.g., for coaxial asemblies."""
        self._joints_inbound:list[AbstractJoint] = []
        """ This mount point may be used to attach the owner to different things. I.e., owner is subassembly of referenced joints.  """

        self.adjust:Callable
        """ Mount point  For the purpose of type hinting """

    @property
    def location(self) -> bd.Location:
        return self._location

    @location.setter
    def location(self, value:bd.Location) -> None:
        assert self._owner is not None, f"The mount point {self} needs to be owned to have a meaning."
        #assert inspect.stack()[1].function in ("__init__", "__setattr__")
        assert isinstance(value, bd.Location)
        self._location:bd.Location = value

    @location.getter
    def location(self) -> bd.Location:
        return self._location

    @property
    def owner(self) -> "Thing":
        return self._owner # type: ignore

    @owner.setter
    def owner(self, value:"Thing") -> None:
        assert isinstance(value, Thing)
        assert self._owner is None
        #assert inspect.stack()[1].function in ("__init__", "__setattr__")
        self._owner:Thing|None = value

    @owner.getter
    def owner(self) -> "Thing":
        assert self._owner is not None
        return self._owner

    @property
    def known_as(self):
        return self._owner

    @known_as.setter
    def known_as(self, value) -> None:
        assert self._known_as is None
        #assert inspect.stack()[1].function in ("__init__", "__setattr__")
        assert isinstance(value, str)
        self._known_as:str|None = value

    @known_as.getter
    def known_as(self) -> str:
        assert self._known_as is not None
        return self._known_as

    def __getattr__(self, __name) -> NoReturn:
        """ Declared for the purpose of LSP - to accept access to this object. """
        raise RuntimeError("This should never happen - the TransformResolver should expose one of the mounts.")

    def __getitem__(self, params):
        """ Shorthand which attaches to the mount point a joint and another mount point. """
        owner, joint_type, other = params
        assert isinstance(owner, Thing)
        assert issubclass(joint_type, AbstractJoint)
        assert isinstance(other, MountPoint|TransformResolver), f"Other {other} is not MountPoint or TransformResolver. It is {type(other)}"
        self.owner = owner
        joint_type(self, other)
        return TransformResolver(mount_point=self)

    def align_to(self, where:"bd.Location|MountPoint"):
        return ReferenceTransformResolver(where=Thing.get_aligning_transform(target_location=where, moved_location=self.location),thing=self.owner)

class Thing (ABC, metaclass=ThingMeta):
    """ A `Thing` is a basic semantic unit of design.
        A `Thing` may have a single resulting body, accessed through `result` method.
        Further it may (and should) be annotated with arbitrary geometric features by setting instance attributes.
        These may be anyting like extrusion profiles, named locations or margin bodies, which are visible for other Things for reference.
        All such geometries are expressed in the `Thing`'s implicit coordinate frame.

        The whole masquerade assumes that the user does not in any case access Thing's `__dict__` directly and uses attribute access as was meant by the Python Gods.
        The `build123d` has a lot of great stuff implemented. Still, to the best of my knowledge, the `build123things` library provides the following conceptual additions:

        Reference Geometry Management:
            - Reference geometries are stored as attributes and they are automatically translated via hijacked attribute access.

        Explicit Semantics:
            - Each `Thing` has to resolve into a single `bd.Part` or `None` via the `result` method, which captures the essence of the `Thing`. If it returns a `Part`, it represents the semantic essence of the Thing. If the `Thing.result()` returns `None`, the `Thing` is useful as a collection of constructive elements or an assembly of other `Thing`s.
            - Joints have explicit semantics.
            - Materials, visual and physical properties are stored unambiguously.
            - It is NOT intended to implement any numerical simulations or kinematics solvers here. Instead, the build123things should offer a way of exporting the Things such that mechanical simulations may be run in different tools.

        Design Parametrizations, Derivations and Alternations:
            - Thing's parameters and their defaults are stored in `__init__`'s params.
            - The particular parameter values are stored with each Thing instance via parameter capture.
            - Named parameter presets or parameter semantic changes are facilitated through subclasses.
            - This is useful to derive different simplified versions of Things, e.g., Screws without Threads or with specialised export methods.
            - One can reinstantiate living object via `alter` method, which allows altering selected parameter values.

        Standardized Assemblies:
            - Attributes of type `bd.Location` define mount points. Mating mounts are aligned with z-axes opposing and x- and y- axes mutually perpendicular. (Rotate x-180deg, y-0deg, z-90deg.)
            - `AbstractJoint` class attaches other Things with arbitrarily parametrized movement.
            - The assembly is a DAG, each `Thing` is the root of its local assembly subtree. Nodes in the tree are Thing's attributes of `AbstractJoint` type.
            - See the ReferenceTransformResolver class.
            - Note: build123d does have assemblies as well, but these are not compatible with build123things.

        Modular Exporting:
            - Export your Thing with all subcomponents to STL files.
            - Export your Thing to MuJoCo description file.
    """

    CAPTURED_PARAMETER_ATTRIBUTE_NAME:str = "__parameters__"
    """ A Thing will have this magic attrbute which will store all patameters passed to constructor. """

    MATERIAL_ATTRIBUTE_NAME:str = "__material__"
    """ A Thing will have this magic attrbute which will store an instance of Material, denoting, ... well, the material the Thing is made of. See documentations of `self.__material__` or `Material` class.  """

    COMPONENT_TYPE:TypeAlias = Union[bd.Sketch, bd.Part, bd.Compound, bd.Curve, bd.Location]
    """ List of all types which may comprise a Thing. """

    list_of_all_existing_things:list["Thing"] = []
    """ Each Thing gets listed here, e.g., to see if everything is working well. """

    # ====================================================
    # === Attributes, Components, Hierarchy and Magic ====
    # ====================================================

    def __init_subclass__(cls) -> None:
        """ Each subclass's __init__ is fitted with the parameter-capture mechanism.
        The parameters are stored in known MRO order.
        Hence, the new_altered can lookup the altered parameters along the real parameter sequence.

        The new_altered may work with parameters altered for different base classes.
        There will be an unification algorithm, which finds the base-most class.

        Imagine there are classes

        - Screw.__init__:(body_diameter, body_length, thread_incline, thread_profile) # The most verbose parametrization in existence.
        - MetricScrew.__init__:(body_diameter, body_length) # Computes the omitted parameters.
        - M2DullScrew.__init__:(body_length, inner_diameter) # Computes the omitted parameters. However, it is also parametrized with something new, which is not known to the base classes.

        A user then instantiates
        s = M2DullScrew(20)

        Finally, the designer wants to derive another Thing. The following may happen:

        s_derived = s.new_altered(body_length__add = 20) # Constructor of M2DullScrew is called.
        s_derived = s.new_altered(body_length__add = 20, body_diameter__mul = 1.5) # This finds the base-most Screw constructor which is called. The resulting Screw is not, however, dull, due to non-pure derivation.
        s_derived = s.new_altered(body_diameter__mul = 1.5, inner_diameter__sub = 0.1) # This fails with ArgumentError because the Screw constructor does not support the inner_diameter parameter.

        The build123things allows having such non-pure Things like M2DullScrew, which break this parameter resolution. The only thing the designer loses is the possibility to call new_altered.
        """
        captured_init:Callable = cls.__init__
        ORIG_INIT_REFERENCES[cls] = cls.__init__
        method_signature:inspect.Signature = inspect.signature(captured_init)
        def __init__(self:Thing, *args, **kwargs) -> None:
            bound:inspect.BoundArguments = method_signature.bind(self, *args,**kwargs)
            bound.apply_defaults()
            if Thing.CAPTURED_PARAMETER_ATTRIBUTE_NAME not in self.__dict__:
                self.__dict__[Thing.CAPTURED_PARAMETER_ATTRIBUTE_NAME] = []
            self.__dict__[Thing.CAPTURED_PARAMETER_ATTRIBUTE_NAME].append((cls, bound.arguments))
            return captured_init(self, *args, **kwargs )
        cls.__init__ = __init__
        return super().__init_subclass__()

    def __init__(self, material:Material) -> None:
        """ It is requested to pass a material argument to the constructor.
        The material may be ForConstruction virtual material or similar.
        If the `Thing` does not compose any result (it is an Assembly), the material is irrelevant.
        Aether is suggested to denote irrelevance of the material.
        """

        if "Thing.__init__" in DEBUG:
            print(f"Thing.__init__ start on {colored.Fore.cyan} {repr(self)}{colored.Style.reset}.")

        Thing.list_of_all_existing_things.append(self)

        super().__init__()

        self.__parameters__:list[tuple[type,dict[str,Any]]]
        """ The parameters passed to constructor and base class constructors. """

        self.p = ParameterResolver(self)
        """ Convenience to access parameters. It finds the parameter value in the closest  """

        self.__material__:Material = material
        """ Each `Thing` may be anotated with miscellaneous data, like physical density, render color etc... """

        if "Thing.__init__" in DEBUG:
            print(f"Thing.__init__ end on {colored.Fore.cyan} {repr(self)}{colored.Style.reset}.")

        self.origin = MountPoint(bd.Location())
        """ Each Thing is defined in a coordinate frame with this as its origin. """

    @abstractmethod
    def result(self) -> bd.Part | None:
        """ Returns a geometry which serves as the reference incarnation, the "essence", the "ideal" or the "etalon" of the Thing. It does not consider any Manufacturing-related, Simulation-related or other issues. The designer is intentionally enforced to implement this function, either to return `None` or something useful, for the sake of explicity.

        This is to be used also as the visual reference, any optimizations during visualization is left for whatever software is used.

        If the designer needs to provide specific collision models or other derivated results, different mechanism is to be used. This is not implemented yet.
        """
        raise NotImplementedError("Never call this superclass method.")

    @final
    def __setattr__(self, __name: str, __value: Any) -> None:
        assert __name.isidentifier(), f"Name {colored.Fore.red}{__name}{colored.Style.reset} is not an identifier."
        if __name in dir(self):
            raise AttributeError(f"Attribute {__name} cannot be reassigned. Instantiate a new Thing by `new_altered` method if you need to modify it.")
        elif not(inspect.stack()[1].function == "__init__" or (inspect.stack()[1].function == "__setitem__" and inspect.stack()[2].function == "__init__")):
            raise AttributeError("You can assign only during __init__.")
        #elif __name == Thing.CAPTURED_PARAMETER_ATTRIBUTE_NAME:
        #    assert isinstance(__value, ConstNamespace), f"Property `{__name}` is reserved for storing constructor parameters."
        elif __name == Thing.MATERIAL_ATTRIBUTE_NAME:
            assert isinstance(__value, Material), f"Property `{__name}` is reserved for an instance of Material class."
        elif isinstance(__value, Thing):
            raise AttributeError("You may not attach other `Thing`s directly. Align them via `align` method instead.")
        #elif isinstance(__value, bd.Location):
        #    warnings.warn("All attributes of type bd.Location are implicitly wrapped in MountPoint. Please, turn your locatons to mount points explicitly as appropriate.", DeprecationWarning)
        #    __value = MountPoint(__value, owner=self, known_as=__name)
        elif isinstance(__value, MountPoint):
            if __value.owned():
                assert __value.owner is self and __value.known_as == __name
            else:
                __value.owner = self
                __value.known_as = __name
        if "Thing.__setattr__" in DEBUG:
            print(f"Thing.__setattr__ {colored.Fore.cyan}{repr(self)}{colored.Style.reset} . {colored.Fore.green}{__name}{colored.Style.reset} = {colored.Fore.red}{repr(__value)}{colored.Style.reset}")
            try:
                code_context = inspect.stack()[1].code_context[0].strip()[:100] # type: ignore
            except:
                code_context = "<code context not available>"
            print(f" {colored.Fore.rgb(100,100,100)}\\_> ...{inspect.stack()[1].filename[-20:]}:{inspect.stack()[1].lineno}   {code_context}{colored.Style.reset}")
        self.__dict__[__name] = __value

    @staticmethod
    def name_mangler(__name:str)->str:
        """ When dynamically assigning with integer-containing names, minus signs are good to   """
        return __name.replace("-", "_")

    def __setitem__(self, __name, __value):
        """ Convenience shorthand for dynamically-created properties. """
        self.__setattr__(Thing.name_mangler(__name), __value)

    def __getitem__(self, __name):
        """ Convenience shorthand for dynamically-created properties. """
        return getattr(self, Thing.name_mangler(__name))

    @final
    def __getattribute__(self, __name:str) -> Any:
        """ Either a construction element is retrieved, or a AbstractJointGrounder which facilitates dynamic transformation resolution.
        This is the only entry point to the Grounder mechanism.
        """
        __dict__ = super().__getattribute__("__dict__")
        if __name in __dict__:
            __value:Any = self.__dict__[__name]
            if isinstance(__value, MountPoint):
                ret = TransformResolver(__value)
            else:
                ret = __value
            if "Thing.__getattr__" in DEBUG:
                print(f"Thing.__getattr__ {colored.Fore.cyan}{repr(self)}{colored.Style.reset} . {colored.Fore.green}{__name}{colored.Style.reset} -> {repr(ret)}")
                try:
                    code_context = inspect.stack()[1].code_context[0].strip()[:100] # type: ignore
                except:
                    code_context = "<code context not available>"
                print(f" {colored.Fore.rgb(100,100,100)}\\_> ...{inspect.stack()[1].filename[-20:]}:{inspect.stack()[1].lineno}   {code_context}{colored.Style.reset}")
            return ret
        else:
            __value = super().__getattribute__(__name)
            if False and "Thing.__getattr__" in DEBUG:
                print(f"Thing.__getattr__ {colored.Fore.cyan}{repr(self)}{colored.Style.reset} . {colored.Fore.green}{__name}{colored.Style.reset} -> {repr(__value)}")
                try:
                    code_context = inspect.stack()[1].code_context[0].strip()[:100] # type: ignore
                except:
                    code_context = "<code context not available>"
                print(f" {colored.Fore.rgb(100,100,100)}\\_> ...{inspect.stack()[1].filename[-20:]}:{inspect.stack()[1].lineno}   {code_context}{colored.Style.reset}")
            return __value

    @final
    def adjust (self, cls=None, **kwargs) -> "Thing":
        """ Allows tweaking existing instance's values.

        This method leverages the passed parameters stored with @parameter_capture annotation.
        The stored parameters are inspected and replaced by the values in **kwargs.
        Special names in `**kwargs` are recognized: '*__add' adds the value to existing value instead of replacing it.

        Memoization ensures, that if a user asks several times for the same modification of a Thing, the very same instance is returned. This is in-line with the assembly DAG.

        User may select which class the arguments are to be applied to. If None, such class is attempted to be found automatically.

        TODO/NOTE: So far, one may alter only parameters among one particular class. If you want to alter parameters which were defined both in some derived class and in its base class, you should call adjust twice.

        FIXME: The resolution mechanism is broken. You don't want to get base class if you adjust some base-class-defined parameter. The resolution has to be deeply reworked. Currently, this can be overcame by enumerating the alterable parametes in the derived classes (hence, however, not being DRY) or maybe trying kwargs.
        """
        captured_param_list = getattr(self, Thing.CAPTURED_PARAMETER_ATTRIBUTE_NAME)
        if cls is None:
            #print(f"Resolving implicitly.")
            def arg_basename(arg_name:str):
                fnd = arg_name.find("__")
                return arg_name, arg_name[:fnd] if fnd > 0 else arg_name
            resolved_index = 0
            for origname, basename in map(arg_basename, kwargs.keys()):
                while basename not in captured_param_list[resolved_index][1]:
                    resolved_index += 1
                    if resolved_index >= len(captured_param_list):
                        raise ArgumentError(f"Cannot Adjust Requested Thing with Expression {origname}. The adjusted type {type(self)} might not have pure constructor. Or such constructor argument did not exist at all.")
            cls = captured_param_list[resolved_index][0]
        else:
            found = False
            resolved_index = 0
            #print(f"Want to resolve explicitly by {cls}.")
            for resolved_index, (cls_candidate, _) in enumerate(captured_param_list):
                if cls is cls_candidate:
                    found = True
                    print(f"Resolving as {cls_candidate}.")
                    break
            assert found, f"Cannot find requested adjust-bace class {cls} among captured_param_list of {self}."
        params = copy.copy(captured_param_list[resolved_index][1])
        if "self" in params:
            del params["self"]
        modified_lookup = set()
        for name, value in kwargs.items():
            # TODO: If someone does both __add and __mul, then we have an undefined behavior - what gets evaluated first?
            if name.endswith("__add"):
                name = name[:-5]
                assert name not in modified_lookup
                modified_lookup.add(name)
                value = params[name] + value
            elif name.endswith("__mul"):
                name = name[:-5]
                assert name not in modified_lookup
                modified_lookup.add(name)
                value = params[name] * value
            else:
                assert name not in modified_lookup
                modified_lookup.add(name)
                assert name in params.keys()
            params[name] = value
        if "Thing.adjust" in DEBUG:
            #print(f"{colored.Fore.cyan}{repr(self)}{colored.Style.reset} . {colored.Fore.light_green}adjust({colored.Fore.red}{kwargs}{colored.Fore.light_green}){colored.Style.reset} as {colored.Fore.red}{cls}{colored.Style.reset}")
            #print(f" - new params: {colored.Fore.dark_green}{params}{colored.Style.reset}")
            try:
                code_context = inspect.stack()[1].code_context[0].strip()[:100] # type: ignore
            except:
                code_context = "<code context not available>"
            #print(f" {colored.Fore.rgb(100,100,100)}\\_> ...{inspect.stack()[1].filename[-20:]}:{inspect.stack()[1].lineno}   {code_context}{colored.Style.reset}")
        return cls(**params)

    @final
    def enumerate_reference_geometries(self, recurse=False, d0=True, d1=True, d2=True, d3=True) -> Generator[Tuple[str,Any], None, None]:
        """ Enumerates all unique attributes regarded as this Thing's construction or reference components.
        These might or might not equal to the Thing's `result`.
        Also, lists are expanded and specific properites like __owner__ etc are ignored to prevent infinite recursion. """
        idset:Set[int] = set()
        for name, value in self.__dict__.items():
            if id(value) in idset:
                continue
            else:
                idset.add(id(value))

            try:
                if not d0 and isinstance(value, bd.Location|bd.Vector):
                    continue
                if not d1 and value._dim == 1:
                    continue
                if not d2 and value._dim == 2:
                    continue
                if not d3 and value._dim == 3:
                    continue
            except AttributeError:
                ...

            if isinstance(value, list):
                for i, subvalue in enumerate(filter(lambda x : isinstance(x, Thing.COMPONENT_TYPE) or isinstance(x, Thing) or isinstance(x, AbstractJoint), value)):
                    yield name + str(i), subvalue
            elif isinstance(value, dict):
                for subname, subvalue in filter(lambda x : isinstance(x[1], Thing.COMPONENT_TYPE) or isinstance(x[1], Thing) or isinstance(x[1], AbstractJoint), value.values()):
                    yield name + ":" + subname, subvalue
            elif isinstance(value, Thing.COMPONENT_TYPE):
                yield name, value
            elif isinstance(value, ReferenceTransformResolver):
                yield name, value

    @final
    def enumerate_mount_locations(self) -> Generator[Tuple[str,bd.Location], None, None]:
        idset:Set[int] = set()
        for name, value in self.__dict__.items():
            if isinstance(value, MountPoint):
                yield name, value.location

    @final
    def enumerate_assembly(self, subassembly = True, where_attached = False, reference = False) -> Generator[Tuple[str,"TransformResolver"], None, None]:
        """ Enumerates all other `Things` attached to `self` via an instance of `AbstractJoint`.

        NOTE: Might be deprecated in favor of `walk` method in the future.
        """
        idset:Set[int] = set()
        for name, value in self.__dict__.items():
            if isinstance(value, MountPoint):
                if subassembly and value._joint_outbound is not None:
                    yield name, TransformResolver(value)
                if where_attached:
                    for i in range(len(value._joints_inbound)):
                        yield f"{name}:{i}", TransformResolver(value, i)
            elif isinstance(value, ReferenceTransformResolver) and reference:
                 yield name, value

    WALK_RETURN_TYPE = tuple[int, "AbstractJoint|None", "Thing", list[Any]]

    @final
    def walk (
            self:"Thing",
            bfs:bool = False, # If `True`, traverse the Thing breath-first. If `False`, traverse depth-first.
            allow_traverse_upwards:bool = True, # If `True`, the algorighm will ascend also against the assembly graph - i.e., it may follow to the super-thing.
            depth_limit:int|None = None, # How many joints may be traversed. `0` yields only `thing`, `1` yields `thing` and also its immediate neighbours, and so on.
            include_mount_points:bool = False,
            include_reference_geometries:bool = False,
            expand_hag:bool = True, # Natively, the assemblies are stored by
            ) -> Generator[WALK_RETURN_TYPE, None, None]:
        """ A generic function, which is able to iterate over all components of the Thing regardless.

        yields: Pairs (int i, Joint j, Thing t, list l) where `i` denotes the traversal-index of a Thing to which `t` is connected via `j`. `l` possibly contains reference geometries and mount points of the Thing. Index `0` is for the `thing` parameter.


        """

        def id_fnc_expanding (thing:Thing, previous_id:tuple[int], *__, **___) -> tuple[int]:
            """ The function takes a previous ID and extends it with an identifier of given Thing. """
            ...

        def id_fnc_conserving (thing:Thing, *__, **___) -> tuple[int]:
            """ The function returns simple identifier of given Thing. """
            ...

        id_fnc_selected = id_fnc_expanding if expand_hag else id_fnc_conserving
        """ One of two id-making functions is stored here. """

        idset:set[tuple[int]] = set()
        """ Store identifiers of already expanded Things/paths such that we don't go there again. """

        queuestack:list[Thing] = [self]
        """ Auxiliary list for the purpose of traversal. """

        current_thing_expansion_index:int = 0
        """ Simple counter of how many times was yielded. Useful to mark adjacency. """

        while len(queuestack) > 0:
            queuestack_head = queuestack.pop(-1 if bfs else 0)
            """ Currently expanded Thing. """

            yield (1,Revolute(MountPoint(bd.Location()), MountPoint(bd.Location())), self, [])

            # Expand the search
            for name, value in self.__dict__.items():
                if isinstance(value, MountPoint):
                    ...






    @final
    def __copy__(self) -> "Thing":
        """ Since a Thing is read-only, no real copy is necessary. """
        return self

    # ====================================================
    # === Moving, Attaching and Aligning =================
    # ====================================================

    @final
    def move(self, transformation:bd.Location)->"TransformResolver":
        """ Returns self, soft-moved to given location via a TransformResolver.
        THE RESULT DOES NOT CONTRIBUTE TO THE ASSEMBLY, it is used only for construction reference. """
        return ReferenceTransformResolver(self, transformation)

    @final
    @staticmethod
    def align(primary_mount:"MountPoint|TransformResolver", secondary_mount:"MountPoint|TransformResolver") -> "ReferenceTransformResolver":
        """ Returns the secondary Thing soft-moved to align with the primary thing. Returned ThingGrounder. Note, that this does not create a joint, the result needs to be assigned to the primary's owner. """
        # TODO: Allow specifying also by locations.
        #if "Thing.align" in DEBUG:
        #    print(f"Trying to align")
        #    print(f" - PRIM {colored.Fore.green}{primary_mount  }{colored.Fore.yellow}#{id(primary_mount  )}{colored.Style.reset} owner {colored.Fore.cyan + repr(getattr(primary_mount, Thing.OWNER_ATTRIBUTE_NAME)) if hasattr(primary_mount, Thing.OWNER_ATTRIBUTE_NAME) else colored.Fore.red + 'MISSING'}")
        #    print(f" - SCND {colored.Fore.green}{secondary_mount}{colored.Fore.yellow}#{id(secondary_mount)}{colored.Style.reset} owner {colored.Fore.cyan + repr(getattr(secondary_mount, Thing.OWNER_ATTRIBUTE_NAME)) if hasattr(secondary_mount, Thing.OWNER_ATTRIBUTE_NAME) else colored.Fore.red + 'MISSING'}")
        #    print(f"{colored.Style.reset}")
        if isinstance(primary_mount, TransformResolver):
            primary_mount = primary_mount._orig_mount
        if isinstance(secondary_mount, TransformResolver):
            secondary_mount = secondary_mount._orig_mount
        assert isinstance(primary_mount, MountPoint)
        assert isinstance(secondary_mount, MountPoint)
        assert secondary_mount.owner is not primary_mount.owner
        aligning_transofmation = Thing.get_aligning_transform(primary_mount, secondary_mount)
        return ReferenceTransformResolver(secondary_mount.owner, aligning_transofmation)

    @final
    @staticmethod
    def get_aligning_transform(target_location:MountPoint|bd.Location, moved_location:MountPoint|bd.Location) -> bd.Location:
        """ Computes a transform in terms of `bd.Location` which aligns the two given locations. """
        if not isinstance(moved_location, bd.Location):
            moved_location = moved_location.location
        if not isinstance(target_location, bd.Location):
            target_location = target_location.location
        ret = target_location * bd.Location((0,0,0),(180,0,90)) * moved_location.inverse()
        return ret

    # ====================================================
    # === Explicitly Named Miscellaneous Properties ======
    # ====================================================

    @memoize
    def volume_mm3(self, recurse=False) -> float:
        """ Volume in cubic millimeters. """
        if recurse:
            raise NotImplementedError("TBI.")
        else:
            res = self.result()
            if res is None:
                return 0.0
            else:
                return res.volume

    def volume(self, recurse=False) -> float:
        """ Volume in base SI units. """
        return self.volume_mm3(recurse=recurse) * 1e-9

    def mass(self, recurse=False) -> float:
        """ Mass in base SI units. """
        if recurse:
            raise NotImplementedError("TBI.")
        else:
            res = self.result()
            if res is None:
                return 0.0
            else:
                return self.__material__.density * res.volume * 1e-9

    def density(self, recurse=False) -> float:
        """ Density in base SI units. """
        if recurse:
            raise NotImplementedError("TBI.")
        if self.mass == Thing.mass:
            return self.__material__.density
        else: # NOTE: The mass was overriden to provide in-vivo measured mass.
            return self.mass() / self.volume()

    @memoize
    def matrix_of_inertia(self, precise=False, tidy = True) -> Tuple[np.ndarray, np.ndarray]:
        """ The resulting matrix of inertia w.r.t. the part's center of gravity.
        return: matrix_of_inertia, center_of_gravity """

        # NOTE: Alternatively computed by
        #warnings.warn("The mass is not considered yet!")
        #what:bd.Part = self.result().scale(1e-3)
        #filename = "/tmp/" + ''.join(random.choices(string.ascii_uppercase + string.digits, k=10)) + ".stl"
        #what.export_stl(filename)
        #ms = pymeshlab.MeshSet() # type: ignore
        #geom = ms.get_geometric_measures()
        #tensor = geom["inertia_tensor"]


        # NOTE: As I use density regularization, then following warning is not required. warnings.warn("The mass is not considered yet!")
        res = self.result()
        if res is None:
            return np.zeros((3,3)), np.array([0,0,0])
        else:
            what:bd.Part = res.scale(1e-3)
            filename = random_tmp_fname(ext=".stl")
            what.export_stl(filename, angular_tolerance=.01 if precise else 0.1)
            _, cog, inertia = mesh.Mesh.from_file(filename).get_mass_properties()
            inertia *= self.density()
            if tidy:
                os.remove(filename)
            elif "Thing.matrix_of_inertia" in DEBUG:
                print(f"The STL for computing the matrix of inertia is in tmp file {filename}")
            return inertia, cog

    @final
    def bounding_box(self, consider_construction:bool = False, consider_assembly:bool = False) -> bd.BoundBox:
        """ Computes bounding box considering all values merged. """
        res = self.result()
        bb:bd.BoundBox = bd.Box(10,10,10).bounding_box() if res is None else res.bounding_box()
        if consider_construction:
            for _, value in self.enumerate_reference_geometries(recurse=False):
                if hasattr(value, "bounding_box"):
                    assert callable(value.bounding_box)
                    if bb is None:
                        bb:bd.BoundBox = value.bounding_box()
                    else:
                        bb.add(value.bounding_box())
        if consider_assembly:
            for _, value in self.enumerate_assembly():
                assert isinstance(value, TransformResolver)
                bb.add(value.bounding_box()) # type: ignore
        return bb

    # ====================================================
    # === Visualization and Debug ========================
    # ====================================================

    def __str__(self):
        lines = [f"Thing {colored.Fore.green}{repr(self)}{colored.Style.reset}#{colored.Fore.yellow}{id(self)}{colored.Style.reset}"]
        for key, value in self.__dict__.items():
            lines.append(f" - {colored.Fore.cyan}{key}{colored.Style.reset} is {colored.Fore.green}{repr(value)}{colored.Fore.yellow}#{id(value)}{colored.Style.reset}")
            lines[-1] += colored.Style.reset
        return "\n".join(lines)

    def codename(self)->str:
        return str(type(self).__name__)

class AbstractJoint (ABC):
    """ Represents possibly parametrized relative placement of a subordinate Thing instance w.r.t. the reference Thing, thus also defining the is-part-of subassembly semantic relation.

    NOTE: I've resigned to implement multi-ended joints. User willing to do complex joints is now supposed to declare pairwise joints, which may, however,

    An assembly is defined by a graph, with nodes=Things and edges=Joints.
    A joint may facilitate any complex motion along any number of independent Things as long as it can evalutate the transform between an arbitrary pair of the two joints.
    A joint is further responsible for maintaining and checking its parametrization, wich is set and evaluated lazily, only on demand.
    The `Thing`s are referenced by their respective selected mount points. The mount points are treated equally unless somehow distinguished in subclass.

    Why a Joint when we can do well enough just with sufficiently parametrizing the Things, do you ask?
        - Because of explicit semantics, the joints are explicitly distinguished from model parametrization.
        - Hence, we may support, e.g., URDF/MJCF exporting.

    Why this, when build123d has its Joints framework?
        - Nicer syntax utilizing the __owner__ references, return values etc.
        - Sound theoretical foundation of the Assembly Directed Acyclic Graph.
        - Easily extensible, separate the technicalities from the movement parametrization.
    """

    @abstractmethod
    def transform(self,mount_from:MountPoint, mount_to:MountPoint) -> bd.Location:
        """ Each implementation is responsible to declare the transform from one to another.
        As both mountpoints are owned by their respective Things, the implementation has to identify them by equalling against this joints attributes. Use always is operator, not the equality. This is currently regarded as necessary evil.

        If the mount point had "second owner", which would reference this joint for convenience, it would make multi-mounting very hard.
        """
        raise RuntimeError("Never call this subclass method.")

    def __init__ (self, reference_mount:"MountPoint|TransformResolver", moving_mount:"MountPoint|TransformResolver", mounts_are_peers:bool=False):
        """ For the purpose of defining hierarchy in the design, always one of the mounts is designated as the reference mount which is regarded as the local root of the induced subassembly.
        Still, it is always possible to traverse the assembly bidirectionally. """

        # Allow passing transform resolvers, which is in fact the most common case.
        if isinstance(reference_mount, TransformResolver):
            reference_mount = reference_mount._orig_mount
        if isinstance(moving_mount, TransformResolver):
            moving_mount = moving_mount._orig_mount

        # Sanity check
        super().__init__()
        assert isinstance(reference_mount, MountPoint)
        assert isinstance(moving_mount, MountPoint)
        assert moving_mount.owner is not reference_mount.owner

        # Inject the mounts to keep track how many joints are attached so far
        reference_mount._joint_outbound = self
        if mounts_are_peers:
            moving_mount._joint_outbound = self
        else:
            moving_mount._joints_inbound.append(self)

        # Now it is necessary to check if a loop is closed.
        # TODO!!!

        # Setup self
        self.reference_mount = reference_mount
        """ Each and every joint links exactly two other Things together.
        The motion is relative, in essence.
        However, for the purpose of human reckoning, it might be better to understand one of the joints as the reference joint (like a stator on a servomotor) and the other as moving relative to the reference.
        """
        self.moving_mount = moving_mount
        """ Each and every joint links exactly two other Things together.
        The motion is relative, in essence.
        However, for the purpose of human reckoning, it might be better to understand one of the joints as the reference joint (like a stator on a servomotor) and the other as moving relative to the reference.
        """
        self.global_name:None|str = None if not hasattr(self, "global_name") else self.global_name
        """ Any joint may carry some label to identify it globally among the whole design.
        This rather wild conditional assignment allows setting it in subclass constructor befor or after calling this init."""
        self.set_default()

    def get_other_mount(self, ref:MountPoint) -> MountPoint:
        """ Identifies the mount which a given Thing is attached by this Joint. """
        if ref is self.reference_mount:
            return self.moving_mount
        elif ref is self.moving_mount:
            return self.reference_mount
        else:
            raise ValueError

    def __str__(self) -> str:
        return f"{repr(self)} linking {repr(self.reference_mount.owner)}.{self.reference_mount.known_as} <-> {repr(self.moving_mount.owner)}.{self.moving_mount.known_as}"

    @final
    def __setattr__(self, name, value) -> None:
        assert name in ("__param_args__", "__param_kwargs__") or inspect.stack()[1].function == "__init__"
        self.__dict__[name] = value

    @abstractmethod
    def set_default(self) -> None:
        """ Sets a default position to this joint. """

    @abstractmethod
    def set(self, *args, **kwargs):
        """ Override this method to check semantic validity of the parameters.
        Call this superclass method to store the currently applied parameters here."""
        self.__param_args__ = args
        self.__param_kwargs__ = kwargs
        if "AbstractJoint.set" in DEBUG: print(f"{colored.Fore.cyan}{repr(self)}{colored.Style.reset} . {colored.Fore.green}set{colored.Style.reset} ( {args} / {kwargs} )")

    @final
    def __call__(self, *args, **kwargs):
        """ Alias for `self.set`. """
        return self.set(*args, **kwargs)

class TransformResolver:
    """ A class which wraps objects in the assembly, gradually tracking an expression chain together with particular incremental transforms.
    Thankfully, the job is quite simple due to binary joints. Wrap another Thing which is unambiguously referenced in constructor.
    """
    def __init__(self, mount_point:MountPoint, which:int|None=None) -> None:
        """ Finds a Thing attached to the mount point according to the `which` argument.

        By default, it finds the only subjugate thing, i.e., traverses down the subassembly.

        If `which` is an integer, it resolves to the `which`-th Thing where `self` is attached to as its subassembly.
        """
        if which is None:
            joint = mount_point._joint_outbound
        else:
            try:
                joint = mount_point._joints_inbound[which]
            except Exception as e:
                joint = None

        if joint is not None:
            assert isinstance(mount_point, MountPoint)
            assert isinstance(mount_point.location, bd.Location), f"Instead: {mount_point} - {mount_point.location} of type {type(mount_point.location)}"
            next_mount = joint.get_other_mount(mount_point)
            joint_transform = joint.transform(mount_point, next_mount)
            full_transform = mount_point.location * joint_transform * bd.Location((0,0,0),(180,0,90)) * next_mount.location.inverse()
        else:
            full_transform = bd.Location() # mount_point.location
            next_mount = None

        if "TransformResolver.__init__" in DEBUG:
            print(f"{colored.Fore.cyan}{repr(self)}{colored.Style.reset} . {colored.Fore.green}__init__{colored.Style.reset} : {colored.Fore.red}{mount_point.location}{colored.Style.reset}")

        self._joint = joint
        """ The joint through which the thing was accessed. """

        self._transform:bd.Location = full_transform
        """ The (cummulative) transform.

        In constructor, the transform is computed as
        ```
        mount_point.owner -----> mount_point.location -----> joint_transform -----> mating_transform -----> next_mount.location.inverse() -----> next_mount.owner
        ```

        However, accessing another Thing through this wrapper modifies the transform, inceremening towards another step.
        The chain of things can be accessed through `self.previous`. """

        self._orig_mount:MountPoint = mount_point
        """ The mount point from which this Resolver was built. """

        self._next_mount:MountPoint|None = next_mount
        """ The mount point from which this Resolver was built. """

        self._wrapped:Thing|None = None if next_mount is None else next_mount.owner
        """ The Thing which is virtually moved by this joint. """

        self._previous:TransformResolver|None = None
        """ Reference to the last resolver in the chain. """

    def align_to(self, where:bd.Location|MountPoint):
        return self._orig_mount.align_to(where)

    def set(self, *args, **kwargs):
        """ Allows manipulating the Joint. """
        assert self._joint is not None
        self._joint.set(*args, **kwargs)

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        """ Allows manipulating the Joint. """
        assert self._joint is not None
        self._joint.set(*args, **kwargs)

    def enumerate_assembly(self, *args, **kwargs) -> Generator[tuple[str, "TransformResolver"], None, None]:
        for n, v in self.wrapped.enumerate_assembly(*args, **kwargs):
            assert isinstance(v, TransformResolver)
            v._transform = self.transform * v.transform
            v._previous = self
            yield n, v

    @property
    def location(self) -> bd.Location:
        """ Alias to `self._transform`. But may be also used for self._orig_mount.location """
        #warn("The semantics of this property is not clear. It may lead to unexpected behavior. Please report your usage of this property such it can be fixed.")
        return self._transform * self._orig_mount.location

    @property
    def transform(self) -> bd.Location:
        """ The cummulative transform. Alias to `self._transform`. """
        return self._transform

    @property
    def wrapped(self) -> Thing:
        assert self._wrapped is not None, f"With TR {self}, prev: {self.previous}, prev wrapped {self.previous.wrapped if self.previous is not None else 'None'}"
        return self._wrapped

    @property
    def previous(self) -> "TransformResolver|None":
        return self._previous

    @previous.setter
    def previous(self, __value:"TransformResolver") -> None:
        assert self._previous is None
        assert isinstance(__value, TransformResolver)
        self._previous = __value

    def __getattr__(self, __name:str):
        __value = getattr(self.wrapped, __name)
        if "TransformResolver.__getattr__" in DEBUG:
            print(f"{colored.Fore.cyan}{repr(self)}{colored.Style.reset} . {colored.Fore.green}{__name}{colored.Style.reset} -> {colored.Fore.red}{str(__value)}{colored.Style.reset}")
        if isinstance(__value, ParameterResolver):
            return __value
        elif isinstance(__value, TransformResolver):
            __value._transform = self.transform * __value.transform
            __value._previous = self
            return __value
        elif callable(__value):
            def proxy (*args, **kwargs):
                result = __value(*args, **kwargs)
                if "TransformResolver.__getattr__" in DEBUG:
                    print(f"{colored.Fore.cyan}{repr(self)}{colored.Style.reset} . {colored.Fore.green}{__name} PROXY CALL{colored.Style.reset} -> {colored.Fore.red}{repr(result)}{colored.Style.reset}")
                if isinstance(result, Thing):
                    #raise NotImplementedError(f"!!!")
                    return ReferenceTransformResolver(result, self.transform, self.previous)
                elif isinstance(result, TransformResolver):
                    result._transform = self.transform * result.transform
                    result._previous = self
                    return result
                elif hasattr(result, "move") and callable(result.move):
                    return copy.copy(result).move(self.transform)
                elif isinstance(result, bd.Location):
                    return self.transform * copy.copy(result)
                else:
                    return result
            return proxy
        elif hasattr(__value, "move") and callable(__value.move):
            return self.transform * copy.copy(__value)
        elif isinstance(__value, bd.Location):
            return self.transform * copy.copy(__value)
        else:
            if "TransformResolver.__getattr__" in DEBUG:
                print(f"{colored.Fore.rgb(100,100,100)} \\> direct return {colored.Style.reset}")
            return __value

    def __str__(self) -> str:
        return f"{repr(self)} @ {self.transform}"# / {self._orig_mount} -> {self._next_mount}"

class ReferenceTransformResolver(TransformResolver):
    """ Convenience to soft-move a foreign Thing. """
    def __init__(self, thing:Thing, where:bd.Location, previous:TransformResolver|None=None) -> None:
        self._transform = where
        self._wrapped = thing
        self._previous = previous

ORIG_INIT_REFERENCES:dict[type["Thing"],Callable] = {Thing: Thing.__init__}
