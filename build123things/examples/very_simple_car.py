from build123things.joints import Revolute, Rigid
from build123things.materials import Steel, Rubber
from build123things import MountPoint, Thing
import build123d as bd
from build123things.misc import is_in_cq_editor

class VerySimpleCar (Thing):
    def __init__(self,
                 width = 2000,
                 length = 4000,
                 height = 1800,
                 nose_offset = 1500,
                 nose_skew = 500,
                 wheel_radius = 300,
                 wheel_thickness = 250,
                 wheel_inset = 100,
                 wheel_cutout = 100,
        ) -> None:
        super().__init__(material=Steel())

        profile:bd.Sketch = bd.Plane.XZ * (bd.Sketch() + bd.make_face(bd.Polyline( # type: ignore
            (length/2, 100),
            (-length/2, 100),
            (-length/2, height/2),
            (-length/2 + nose_offset - nose_skew, height/2),
            (-length/2 + nose_offset + nose_skew, height),
            (length/2, height),
            (length/2, 100),
        )))

        body:bd.Part = bd.extrude(profile, amount=width/2, both=True)

        wheel = Wheel(wheel_radius, wheel_thickness)

        self.wheel_fl = MountPoint(bd.Location((-length/2*0.7, -width/2 + wheel_inset, 0), (90,0,0)))
        Revolute(self.wheel_fl, wheel.mount, limit_effort=100, limit_velocity=100)
        body -= self.wheel_fl.adjust(radius__add = wheel_cutout).body

        self.wheel_fr= MountPoint(bd.Location((-length/2*0.7, width/2 - wheel_inset, 0),( -90,0,0)))
        Revolute(self.wheel_fr, wheel.mount, limit_effort=100, limit_velocity=100)
        body -= self.wheel_fr.adjust(radius__add = wheel_cutout).body

        self.wheel_bl= MountPoint(bd.Location((length/2*0.7, -width/2 + wheel_inset, 0),( 90,0,0)))
        Revolute(self.wheel_bl, wheel.mount, limit_effort=100, limit_velocity=100)
        body -= self.wheel_bl.adjust(radius__add = wheel_cutout).body

        self.wheel_br= MountPoint(bd.Location((length/2*0.7, width/2 - wheel_inset, 0),( -90,0,0)))
        Revolute(self.wheel_br, wheel.mount, limit_effort=100, limit_velocity=100)
        body -= self.wheel_br.adjust(radius__add = wheel_cutout).body

        self.antenna = MountPoint(bd.Location((100,width/4,height)))
        Rigid(self.antenna, Antenna(radius=20).mount)

        self.body = body

    def result(self):
        return self.body

class Antenna (Thing):
    def __init__(self, length=400, radius=5) -> None:
        super().__init__(material=Rubber())

        self.body = bd.Cylinder(radius=radius, height=length, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)) + bd.Sphere(radius=2*radius)
        self.mount = MountPoint(bd.Location((0,0,length), (0,0,0)))

    def result(self) -> bd.Part:
        return self.body

class Wheel (Thing):
    def __init__(self, radius = 400, thickness = 100) -> None:
        super().__init__(material=Rubber())

        self.body = bd.Cylinder(radius=radius, height=thickness, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)) - bd.Box(width=10000, length=10, height=10, align=(bd.Align.CENTER,bd.Align.MIN,bd.Align.CENTER))

        #self.screw_mount = bd.Location((radius*.5,0,thickness*.1), (180,0,0))
        #self.screw = self.align(self.screw_mount, MetricScrew(10, 100).tip)

        #for i in range(10):
        #    setattr(self, f"screw_mount_{i}", bd.Location((i*15,0,0), (180,0,0)))# * bd.Location((radius*.8,0,0), (180,0,0)))
        #    setattr(self, f"screw_{i}", )

        self.mount = MountPoint(bd.Location((0,0,thickness),(0,0,0)))

    def result(self) -> bd.Part:
        return self.body

if __name__ == "__main__" or is_in_cq_editor():
    print("\n\n C O N S T R U C T I O N   S T A R T \n\n")

    car = VerySimpleCar()

    print("\n C O N S T R U C T I O N   D  O N E \n")

    car.wheel_bl.set(45)
    car.wheel_br(-45)
    car.wheel_fl.set(90)
    car.wheel_fr.set(270)

    from build123things.show import show

    show(car, recurse=True)

    #print(f"Wheel - refthing (refname {car.wheel_fl.secondary_mount_0__refname__})")
    #print(car.wheel_fl.secondary_mount_0__refthing__)
    #print("Wheel - result")
    #print(car.wheel_fl.secondary_mount_0)
    #car.wheel_fl(90)
    #car.wheel_fl(0)
    #car.wheel_fl(-10)
    #car.wheel_fl(-180)
    ##car.wheel_bl(95)
    #car.wheel_br(-95)
    #car.show_nice(show_object) # type: ignore

    #wheel = Wheel()
    #wheel.show_everything(show_object, recurse=False) # type: ignore

    #print(car.mass())
    #wheel = Wheel()
    #wheel.show_everything(show_object) # type: ignore

    #print("\n\n R E N D E R   F I N I S H E D \n\n")

    #show(car, recurse=True)
