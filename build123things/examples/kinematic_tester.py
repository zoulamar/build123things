"""
A very simple mechanism to test and verify build123things algorithms and implementation.
"""

from build123things import misc
from build123things.colors import Color
from build123things.env import *
from build123things.joints import Revolute, Rigid
from build123things.materials import PETG

class LinkA (Thing):
    def __init__(self) -> None:
        super().__init__(PETG(color=Color(1,0,0,0)))
        self.body = bd.Box(1000,1000,1000) + L((0,0,500)) * bd.Box(100,100,100) + L((500,0,0)) * bd.Sphere(100)

        self.linkb = M(L((1500,0,0), (0,0,0)))
        self.joint = Rigid(self.linkb, LinkB().linka)

    def result(self) -> bd.Part:
        return self.body

class LinkB (Thing):
    def __init__(self) -> None:
        super().__init__(PETG(color=Color(0,1,0,0)))
        self.body = bd.Box(1000,1000,1000) + L((0,0,500)) * bd.Box(100,100,100) + L((500,0,0)) * bd.Sphere(100)

        self.linka = M(L((-1500,0,0),(180,0,90)))

        self.linkc = M(L((1500,0,0), (0,90,0)))
        self.joint = Revolute(self.linkc, LinkC().linkb)

    def result(self) -> bd.Part:
        return self.body

class LinkC (Thing):
    def __init__(self) -> None:
        super().__init__(PETG(color=Color(0,0,1,0)))
        self.body = bd.Box(1000,1000,1000) + L((0,0,500)) * bd.Box(100,100,100) + L((500,0,0)) * bd.Sphere(100)

        self.linkb = M(L((-1500,0,0),(180,-90,90)))

    def result(self) -> bd.Part:
        return self.body

if misc.is_in_cq_editor() or __name__ == "__main__":
    from build123things.show import show
    r = LinkA()

