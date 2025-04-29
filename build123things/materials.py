import warnings
import build123things.colors
from build123things.colors import Color
from typing import Any, Union

class Material:
    """ Defines the Thing's embodiment. Currently, this means that it stores values for
    - Density
    - Color

    In the future, it might also facilitate
    - Texture (for the purpose of exporting)
    - Electrical properties / Pinout (although this is rather for some circuit solving effort which is out of scope now)
    - Roughness and such (which would be nice research topic, but related to Thing learning.)

    Note that the Thing defines the geometry and semantics of component hierarchy; this class serves as a codified Material annotation.
    """

    def __init__(self, density:float, color:None | Color) -> None:
        self.__owner__:Any
        """ The owner is set automatically via Thing's __setattr__ mechanism.
        This declaration is for type hinting only. """
        setattr(self, "density", density)
        setattr(self, "color", color)

    @property
    def density (self):
        """ Density in SI units kilograms per cubic meter. If set to `NaN`, the material is considered mass-less, the same as if set to zero. Negative and infinite values are rejected. """
        return self.__dict__["__density__"]

    @density.setter
    def density (self, value:float):
        if isinstance(value, int):
            value = float(value)
        assert isinstance(value, float)
        if value == float("NaN"):
            value = 0.0
        assert value >= 0.0 and value != float("inf")
        self.__dict__["__density__"] = value

    @property
    def color (self):
        """ A color or None, if the Material is not to be rendered at all. """
        return self.__dict__["__color__"]

    @color.setter
    def color (self, value:Union[build123things.colors.Color, None]):
        #assert isinstance(value, build123things.colors.Color) or value is None
        if value is None:
            value = build123things.colors.Color(0,0,0,1)
        self.__dict__["__color__"] = value

    @property
    def codename(self) -> str:
        return type(self).__name__

class Aether (Material):
    """ A material which is not a material in fact. It is used to annotate Joints. """
    def __init__(self) -> None:
        super().__init__(density = 0, color = None)

class MixedMaterial (Material):
    def __init__(self) -> None:
        warnings.warn("The PETG density is a guess. Fix me.")
        super().__init__(density= 0, color=build123things.colors.AQUA)

class Steel (Material):
    def __init__(self, color=build123things.colors.STEELBLUE) -> None:
        super().__init__(density=7800.0, color=color)

class Rubber (Material):
    def __init__(self, color=build123things.colors.BURLYWOOD) -> None:
        super().__init__(density=800.0, color=color)

class Brass (Material):
    def __init__(self, color=build123things.colors.ORANGERED3) -> None:
        super().__init__(density=8500, color=color)

class PETG (Material):
    # TODO: Allow passing print properites
    def __init__(self, color:Color=build123things.colors.LAVENDER) -> None:
        super().__init__(density=1230, color=color)

class PCB (Material):
    def __init__(self, color=build123things.colors.GREEN1) -> None:
        super().__init__(density=1000, color=color)

