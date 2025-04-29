import build123d as bd
from build123things.materials import Steel
from build123things.misc import is_in_cq_editor
from build123things.env import *

class TNP (Thing):
    def __init__(self,
                 parameter = 1
        ) -> None:
        super().__init__(material=Steel())

        self.body = bd.Box(10,10,1) + bd.Box(5,5,2) - bd.Cylinder(radius=parameter, height=100)

        self.ref_plane = L((0,0,1.035)) * bd.Rectangle(5,5)

    def result(self) -> bd.Part:
        return self.body # type:ignore

if __name__ == "__main__" or is_in_cq_editor():
    from build123things.show import show
    x = TNP(2)
    show(x)
