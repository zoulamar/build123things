import build123things
import build123d as bd
from typing import final, Tuple

class Rigid(build123things.AbstractJoint):
    """ A specical type of joint which facilitates rigid assemblies.

    As we enforce explicit (single) result of each Thing, which is essential for correct and unambiguous mass and inertia computation, this is the way how to create rigid assemblies.
    Due to "lazy" Grounding expression mechanism, the computation should be also relatively fast.
    """
    def __init__(self, a: build123things.MountPoint, b: build123things.MountPoint, peers = False):
        super().__init__(a, b, peers)

    def transform(self, _: build123things.MountPoint, __: build123things.MountPoint) -> bd.Location:
        return bd.Location()

    @final
    def set_default(self, *_, **__)->None:
        pass

    @final
    def set(self)->None:
        pass

class Revolute(build123things.AbstractJoint):
    """ The joint that facilitates rotation of a single secondary object around primary z axis.
    Note, If you want to rotate more objects, bring them all together under a single Thing.
    """

    def __init__(self,
                 stator: build123things.MountPoint,
                 rotor: build123things.MountPoint,
                 peers:bool = False,
                 limit_velocity:float|None=None,
                 limit_effort:float|None=None,
                 limit_angle:Tuple[float,float]|None=None,
                 global_name:None|str = None
        )->None:
        assert limit_angle is None or len(limit_angle) == 2
        self.limit_angle:Tuple[float,float]|None = limit_angle
        self.limit_effort:float|None = limit_effort
        self.limit_velocity:float|None = limit_velocity
        self.global_name:None|str = global_name
        super().__init__(stator, rotor, peers)

    def transform(self, mount_static: build123things.MountPoint, mount_aligned: build123things.MountPoint) -> bd.Location:
        t:bd.Location = bd.Location((0,0,0), (0,0,self.__param_kwargs__["alpha"]))
        if mount_static is self.reference_mount and mount_aligned is self.moving_mount:
            return t
        elif mount_static is self.moving_mount and mount_aligned is self.reference_mount:
            return t.inverse()
        else:
            raise ValueError

    def set(self, alpha:float) -> None:
        if self.limit_angle is not None:
            assert len(self.limit_angle) == 2 and self.limit_angle[0] <= alpha <= self.limit_angle[1]
        return super().set(alpha=alpha)

    def set_default(self) -> None:
        return self.set(0)

