#!/usr/bin/env python3

"""
This file contains fixtures to attach two Dynamixel servos together. Some of the codes are very ugly!
"""

from math import sqrt
import build123d as bd
from build123things import MountPoint, ReferenceTransformResolver, Thing
from build123things.joints import Rigid
from build123things.materials import PETG
from build123things.partlib.dynamixel import XM430, XM540
import build123things

class BracketRotorBody (Thing):
    def __init__(self,
            distance_between_rotors = 60,
            thickness = 4,
        )->None:

        super().__init__(material=PETG())

        servo_by_rotor_ref:XM430 = ReferenceTransformResolver(XM430(), bd.Location()) # type: ignore
        servo_by_body_ref:XM430 = ReferenceTransformResolver(XM430(), bd.Location((0,distance_between_rotors,0),(0,90,0))) # type: ignore

        self.servo_by_body_ref = servo_by_body_ref
        self.servo_by_rotor_ref = servo_by_rotor_ref

        loc_rotor:bd.Location = servo_by_rotor_ref.rotor_sketch.location
        self.mount_rotor:MountPoint = MountPoint(loc_rotor * bd.Location((0,0,0),(180,0,90)))
        self.sketch_attach_rotor:bd.Sketch = bd.Sketch() + loc_rotor * (
            bd.Circle(radius=25/2) +
            bd.Rectangle(width=25,height=25/2,align=(bd.Align.CENTER,bd.Align.MIN)) -
            bd.Circle(radius=8.5/2)# -
            #[bd.Location((0,0,0),(0,0,a*45)) * bd.Location((8,0,0)) * bd.Circle(radius=2.5/2) for a in range(8)] # type: ignore
        )
        a:bd.Part = bd.extrude(self.sketch_attach_rotor, amount=thickness)

        # Exemplary reference geometries.
        self.servo_by_body_ref_screw_1 = servo_by_body_ref.screw_right_top_rear()
        self.servo_by_body_ref_screw_2 = servo_by_body_ref.screw_right_top_front()
        self.servo_by_body_ref_screw_3 = servo_by_body_ref.screw_right_bottom_rear()
        self.servo_by_body_ref_screw_4 = servo_by_body_ref.screw_right_bottom_front()


        self.illustrative_servo_face = bd.Location((.1,0,0)) * servo_by_body_ref.rotor_sketch

        self.servo_by_rotor_ref_screw_0 = servo_by_rotor_ref.screw_rotor_row1_0()
        self.servo_by_rotor_ref_screw_1 = servo_by_rotor_ref.screw_rotor_row1_1()
        self.servo_by_rotor_ref_screw_2 = servo_by_rotor_ref.screw_rotor_row1_2()
        self.servo_by_rotor_ref_screw_3 = servo_by_rotor_ref.screw_rotor_row1_3()
        self.servo_by_rotor_ref_screw_4 = servo_by_rotor_ref.screw_rotor_row1_4()
        self.servo_by_rotor_ref_screw_5 = servo_by_rotor_ref.screw_rotor_row1_5()
        self.servo_by_rotor_ref_screw_6 = servo_by_rotor_ref.screw_rotor_row1_6()
        self.servo_by_rotor_ref_screw_7 = servo_by_rotor_ref.screw_rotor_row1_7()

        for ref in [self.servo_by_body_ref_screw_1,
                    self.servo_by_body_ref_screw_2,
                    self.servo_by_body_ref_screw_3,
                    self.servo_by_body_ref_screw_4,
                    self.servo_by_rotor_ref_screw_0,
                    self.servo_by_rotor_ref_screw_1,
                    self.servo_by_rotor_ref_screw_2,
                    self.servo_by_rotor_ref_screw_3,
                    self.servo_by_rotor_ref_screw_4,
                    self.servo_by_rotor_ref_screw_5,
                    self.servo_by_rotor_ref_screw_6,
                    self.servo_by_rotor_ref_screw_7]:
            a -= ref.result()

        loc_body:bd.Location = servo_by_body_ref.right.location
        self.mount_body:MountPoint = MountPoint(loc_body * bd.Location((0,0,0),(180,0,90)))
        self.sketch_attach_body:bd.Sketch = bd.Sketch() + loc_body * (
            bd.Rectangle(width=25,height=40) -
            bd.Rectangle(width=10,height=15)
            #bd.Rectangle(width=34,height=40/2,align=[bd.Align.CENTER,bd.Align.MIN])
        )
        b:bd.Part = bd.extrude(self.sketch_attach_body, amount=thickness)

        self.path = bd.Curve() + bd.Spline(
            (0,25/2,loc_rotor.position.Z),
            (0,25/2+5,loc_rotor.position.Z),
            (0,loc_body.position.Y-20,loc_body.position.Z),
            #(0,loc_body.position.Y-20,loc_body.position.Z),
        )

        #bd.loft(bd.Sketch() + [a.faces().group_by(bd.Axis.Y)[-1][0], b.faces().group_by(bd.Axis.Y)[0][0]])
        self.body = a + b + bd.sweep(a.faces().group_by(bd.Axis.Y)[-1], self.path.wires()[0]) - [servo_by_body_ref.screw(which, length_above=1).adjust(width__add=0.1, head_width__add=0.1).body_hull for which in ["-x+y+z", "-x+y-z", "-x-y+z", "-x-y-z"]] # type: ignore

    def result(self) -> bd.Part:
        return self.body

    def mass(self, recurse=False) -> float:
        return 0.006 # Measured 2024-02-07

class BracketRotor (Thing):
    """ A part which attaches to a bearing and a rotor (primary plane) and provides attachment facilities (secondary plane).

    It is possible to align two such brackets to obtain a rotor-rotor interface.
    """

    def __init__(
            self,
            servo_type = XM430,
            clearance = 20,
            primary_thickness=4.5,
            primary_span_clearance=0.15,
            primary_rotor_clearance=0.15,
            secondary_thickness=6,
            secondary_allow_counterfit=True,
            secondary_screw_diameter=3,
            secondary_screw_diameter_clearance=0.2,
            secondary_attach_knobs=False, # If false, there are holes. Else, there are knobs.
            secondary_counterfit_clearance=0.1,
            secondary_override_width=None,
            secondary_force_430_knobs=False
        ) -> None:

        super().__init__(material=PETG())

        servo = ReferenceTransformResolver(servo_type(), bd.Location())
        """ A dummy servo. TODO: Make as parameter? """

        clearance_min = sqrt(servo.p.y_centering_nominal**2 + (servo.p.width/2)**2) # type: ignore
        assert clearance > clearance_min, f"The clearance is too small! The bracket could not rotate around the servo. Requested {clearance}, needed {clearance_min}"

        primary_radius_outer = servo.p.rotor_radius_1 + 1.5 # type: ignore
        primary_radius_inner = servo.p.rotor_radius_2+primary_rotor_clearance # type: ignore

        primary_span = abs(servo.rotor_sketch.location.position.Z - servo.bearing_sketch.location.position.Z)
        """ The distance between primary planes. """
        print(f"primary_span = {primary_span}")

        self.mount_a = MountPoint(servo.rotor_sketch.location * bd.Location((0,0,0),(180,0,90)))
        """ A rotor or bearing can be attached here. """

        self.mount_b = MountPoint(servo.bearing_sketch.location * bd.Location((0,0,0),(180,0,-90)))
        """ A rotor or bearing can be attached here. """

        secondary_base = bd.Location((0, clearance, 0), (-90,0,0))
        """ Extrusion base for the secondary plane. """

        self.mount_c = MountPoint(secondary_base * bd.Location((0,0,secondary_thickness/2 if secondary_allow_counterfit else secondary_thickness)))
        """ Another bracket or a different part may be attached here. """

        sketch_face = bd.Sketch() + \
            bd.Circle(radius=primary_radius_outer) + \
            bd.Rectangle(height=clearance+secondary_thickness, width=2*primary_radius_outer, align=(bd.Align.CENTER, bd.Align.MAX)) - \
            bd.Circle(radius=primary_radius_inner) - \
            bd.make_face(bd.Polyline( # type: ignore
                (-primary_radius_inner,0),
                (-primary_radius_inner-2,3),
                (-8,4),
                (-primary_radius_outer,4),
                (-primary_radius_outer,30),
                (+primary_radius_outer,30),
                (+primary_radius_outer,4),
                (+8,4),
                (+primary_radius_inner+2,3),
                (+primary_radius_inner,0),
                ))

        sketch_body = secondary_base * (bd.Sketch() + bd.Rectangle(height=primary_span+primary_thickness*2 , width=2*primary_radius_outer))
        sketch_body_cross = self.mount_c.location * (bd.Sketch() + bd.Rectangle(height=(2*primary_radius_outer if secondary_override_width is None else secondary_override_width)+2*secondary_counterfit_clearance, width=2*primary_radius_outer+2*secondary_counterfit_clearance))
        #sketch_body_cable = self.mount_c.location * (bd.Sketch() + bd.Circle(radius=7))
        sketch_body_cable = self.mount_c.location * (bd.Sketch() + bd.Rectangle(width=13,height=4.5) + bd.Rectangle(width=4.5,height=13))
        #sketch_body_slots = self.mount_c.location * (bd.Sketch() + [bd.Location((0,0,0),(0,0,a*360/secondary_attach_slots)) * bd.Location((secondary_attach_span/2,0,0)) * bd.Circle(radius=secondary_screw_diameter/2 + secondary_screw_diameter_clearance * 0.5 * (-1 if secondary_attach_knobs else 1)) for a in range(secondary_attach_slots)]) # type: ignore

        if secondary_force_430_knobs or isinstance(servo, XM430):
            sketch_body_slots = self.mount_c.location * (bd.Sketch() + [bd.Location((xy[0], xy[1])) * bd.Circle(radius=secondary_screw_diameter/2 + secondary_screw_diameter_clearance * 0.5 * (-1 if secondary_attach_knobs else 1)) for xy in [(6,8),(-6,8),(6,-8),(-6,-8)]])
            sketch_body_slots += self.mount_c.location * (bd.Sketch() + [bd.Location((xy[1], xy[0])) * bd.Circle(radius=secondary_screw_diameter/2 + secondary_screw_diameter_clearance * 0.5 * (-1 if secondary_attach_knobs else 1)) for xy in [(6,8),(-6,8),(6,-8),(-6,-8)]])
            #sketch_body_slots += self.mount_c.location * (bd.Sketch() + [bd.Location((0,0,0),(0,0,a*90+45)) * bd.Location((sqrt(8**2+6**2),0,0)) * bd.Circle(radius=secondary_screw_diameter/5 + secondary_screw_diameter_clearance * 0.5 * (-1 if secondary_attach_knobs else 1)) for a in range(4)])
        if isinstance(servo, XM540) and not secondary_force_430_knobs:
            sketch_body_slots = self.mount_c.location * (bd.Sketch() + [bd.Location((xy[0], xy[1])) * bd.Circle(radius=secondary_screw_diameter/2 + secondary_screw_diameter_clearance * 0.5 * (-1 if secondary_attach_knobs else 1)) for xy in [(10,8),(-10,8),(10,-8),(-10,-8)]])
            sketch_body_slots += self.mount_c.location * (bd.Sketch() + [bd.Location((xy[1], xy[0])) * bd.Circle(radius=secondary_screw_diameter/2 + secondary_screw_diameter_clearance * 0.5 * (-1 if secondary_attach_knobs else 1)) for xy in [(10,8),(-10,8),(10,-8),(-10,-8)]])
        sketch_face_a = self.mount_a.location * bd.Location((0,0,-primary_span_clearance),(0,0,-90)) * sketch_face
        sketch_face_b = self.mount_b.location * bd.Location((0,0,-primary_span_clearance),(0,0,90)) * sketch_face

        ziptie_span = 13 # primary_radius_outer * .4 * 2

        result = bd.extrude(sketch_body, amount=secondary_thickness) - \
                bd.extrude(sketch_body_cross, amount=secondary_thickness/2) + \
                bd.extrude(sketch_face_a, amount=-primary_thickness) + \
                bd.extrude(sketch_face_b, amount=-primary_thickness) - \
                bd.extrude(sketch_body_cable, amount=-secondary_thickness * (.5 if secondary_allow_counterfit else 1)) - \
                bd.Location((ziptie_span/2,clearance*0.7)) * bd.Box(width=3, length=1.5, height=100) - \
                bd.Location((-ziptie_span/2,clearance*0.7)) * bd.Box(width=3, length=1.5, height=100) - \
                bd.Location((0,clearance*0.7)) * bd.Box(width=3, length=ziptie_span, height=primary_span+3) -\
                [servo.screw(which=sname, length_above=2.5).adjust(width__add=0.1, head_width__add=0.1, head_length=1000).body_hull for sname in servo.screw_definitions.keys() if sname.startswith("ra") or sname.startswith("b")]
                #bd.extrude(sketch_cable_holder_a1, amount=-cable_holder_thickness_tip) + \
                #bd.extrude(sketch_cable_holder_a2, amount=+cable_holder_thickness_tip)
                #bd.extrude(bd.Location((0,0,0),(90,0,0)) * bd.Circle(radius=extrude_amount/2), amount=thickness/2, both=True) - \
                #bd.extrude(bd.Location((0,0,0),(90,0,0)) * bd.Circle(radius=extrude_amount/4), amount=thickness/2, both=True)
        #if secondary_attach_knobs:
        #    result += bd.extrude(sketch_body_slots, amount=secondary_thickness * (.5 if secondary_allow_counterfit else 1))
        #else:
        #    result -= bd.extrude(sketch_body_slots, amount=-secondary_thickness * (.5 if secondary_allow_counterfit else 1))

        #self.servo = servo
        self.sketch_body = sketch_body
        self.sketch_body_cross = sketch_body_cross
        self.sketch_body_cable = sketch_body_cable
        #self.sketch_body_slots = sketch_body_slots
        self.sketch_face_a = sketch_face_a
        self.sketch_face_b = sketch_face_b
        self.primary_radius_outer = primary_radius_outer
        self.primary_radius_inner = primary_radius_inner
        self.body = result

    def result(self) -> bd.Part:
        return self.body

    def mass(self, recurse=False) -> float:
        return 0.012 # Measured 2024-02-07

class BracketRotorBended (Thing):
    def __init__(self,
                 servo_type:type = XM540,
                 clearance_total:float = 22,
                 clearance_face_to_face=0.15,
                 primary_rotor_clearance=0.15,
                 secondary_screws_span=15,
                 secondary_screws_diameter=3,
                 secondary_hole_diameter=15,
                 bending_zone_additional_length=1,
        ) -> None:
        super().__init__(material=PETG())

        servo = servo_type()
        clearance_min = sqrt(servo.p.y_centering_nominal**2 + (servo.p.width/2)**2)
        assert clearance_total > clearance_min, f"The clearance is too small! The bracket could not rotate around the servo. Requested {clearance_total}, needed {clearance_min}"
        primary_radius_outer = servo.p.rotor_radius_1 + 1.5
        primary_radius_inner = servo.p.rotor_radius_2+primary_rotor_clearance
        primary_span = abs(servo.rotor_sketch.location.position.Z - servo.bearing_sketch.location.position.Z)

        core = bd.Sketch() + bd.Rectangle(width=primary_radius_outer*2, height=primary_span) - \
                bd.Circle(radius=secondary_hole_diameter/2) -\
                [bd.Location((secondary_screws_span/2*a[0], secondary_screws_span/2*a[1])) * bd.Circle(radius=secondary_screws_diameter/2+clearance_face_to_face) for a in ((1,1),(1,-1),(-1,1),(-1,-1))]
        bending_zone = bd.Sketch() + bd.Rectangle(width=primary_radius_outer*2, height=bending_zone_additional_length, align=(bd.Align.CENTER, bd.Align.MIN))
        bending_zone_a = bd.Location((0,primary_span/2),(0,0,0)) * bending_zone
        bending_zone_b = bd.Location((0,-primary_span/2),(0,0,180)) * bending_zone
        flip = bd.Sketch() + \
            bd.Circle(radius=primary_radius_outer) + \
            bd.Rectangle(height=clearance_total, width=2*primary_radius_outer, align=(bd.Align.CENTER, bd.Align.MAX)) - \
            bd.Circle(radius=primary_radius_inner) - \
            bd.make_face(bd.Polyline( # type: ignore
                (-primary_radius_inner,0),
                (-primary_radius_inner-2,3),
                (-8,4),
                (-primary_radius_outer,4),
                (-primary_radius_outer,30),
                (+primary_radius_outer,30),
                (+primary_radius_outer,4),
                (+8,4),
                (+primary_radius_inner+2,3),
                (+primary_radius_inner,0),
                )) - \
            [bd.Location((0,0,0),(0,0,a * 360 / servo.p.rotor_row_1_screw_count)) * bd.Location((servo.p.rotor_row_1_spanning_radius, 0)) * bd.Circle(radius=servo.p.rotor_row_1_screw_diameter/2+.1) for a in range(servo.p.rotor_row_1_screw_count)]


        self.core = core
        self.bending_zone_a = bending_zone_a
        self.bending_zone_b = bending_zone_b
        self.flip_a = bd.Location((0, (clearance_total + primary_span/2 + bending_zone_additional_length)),(0,0,0)) * flip
        self.flip_b = bd.Location((0,-(clearance_total + primary_span/2 + bending_zone_additional_length)),(0,0,180)) * flip

        self.kopyto:bd.Part = bd.Box(width=primary_span, height=40, length=primary_radius_outer*2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)) + \
            bd.Cylinder(radius=secondary_hole_diameter/2 - clearance_face_to_face, height=4, align=(bd.Align.CENTER,bd.Align.CENTER, bd.Align.CENTER)) + \
            [bd.Location((secondary_screws_span/2*a[0], secondary_screws_span/2*a[1])) * bd.Cylinder(radius=secondary_screws_diameter/2, height=4, align=(bd.Align.CENTER,bd.Align.CENTER, bd.Align.CENTER)) for a in ((1,1),(1,-1),(-1,1),(-1,-1))]
        self.kopyto = bd.fillet(self.kopyto.edges().filter_by(bd.Axis.X), radius=2)

class BracketRotorRotorBended (Thing):
    def __init__(self,
                 servo_type_a:type = XM540,
                 servo_type_b:type = XM430,
                 distance_a:float = 22,
                 distance_b:float = 20,
        ) -> None:
        super().__init__(material=PETG())

        self.a = BracketRotorBended(servo_type=servo_type_a, clearance_total=distance_a)
        self.b = BracketRotorBended(servo_type=servo_type_b, clearance_total=distance_b).move(bd.Location((0,0,0), (0,0,90)))

        result = bd.Sketch()
        for a in (self.a, self.b):
            for name, value in a.__dict__.items():
                if isinstance(value, bd.Sketch):
                    result += value
        self.body = result

    def result(self) -> bd.Part:
        return self.body

class AssemblyBracketRotorRotor (Thing):
    """ New and better way to attah two servos rotor-to-rotor. """

    def __init__(
            self,
            bracket1:BracketRotor,
            bracket2:BracketRotor,
        ) -> None:
        super().__init__(material=PETG())

        self.bracket1 = MountPoint(bd.Location())
        Rigid(self.bracket1, bracket1.mount_c)

        self.bracket2 = MountPoint(bd.Location(bd.Location((0,0,0), (180,0,90))))
        Rigid(self.bracket2, bracket2.mount_c)

        self.mount_rotor_1 = MountPoint(self.bracket1.mount_a.location)
        self.mount_rotor_2 = MountPoint(self.bracket2.mount_a.location)

    def result(self):
        return None

class AssemblyBracketRotorRotor_2XM430 (AssemblyBracketRotorRotor):
    def __init__(
            self,
            clearance_XM430 = 20.0,
        ) -> None:
        a = BracketRotor(servo_type=XM430, clearance=clearance_XM430, secondary_force_430_knobs=True, secondary_attach_knobs=False, primary_thickness=2, secondary_thickness=3)
        b = BracketRotor(servo_type=XM430, clearance=clearance_XM430, secondary_attach_knobs=True, primary_thickness=2, secondary_thickness=3)
        super().__init__(a,b)

class AssemblyBracketRotorRotor_XM530_XM430 (AssemblyBracketRotorRotor):
    """ New and better way to attah two servos rotor-to-rotor. """

    def __init__(
            self,
            clearance_XM540 = 22.0,
            clearance_XM430 = 20.0,
        ) -> None:
        a = BracketRotor(servo_type=XM540, clearance=clearance_XM540, secondary_force_430_knobs=True, secondary_attach_knobs=False, primary_thickness=4.5, secondary_thickness=6)
        b = BracketRotor(servo_type=XM430, clearance=clearance_XM430, secondary_attach_knobs=True, secondary_override_width = a.primary_radius_outer*2, primary_thickness=4.5, secondary_thickness=6)
        a = a.adjust(secondary_override_width=b.primary_radius_outer*2)
        super().__init__(a,b)

if __name__ == "__main__" or build123things.misc.is_in_cq_editor():

    print("\n\n C O N S T R U C T I O N   S T A R T \n\n")

    #b = BracketRotor()
    b = BracketRotorBody()
    #b = BracketRotor(XM540, secondary_force_430_knobs=True)
    #b = AssemblyBracketRotorRotor_XM530_XM430()
    #b = BracketRotorRotorBended()
    #b = BracketRotorBended()

    print("\n C O N S T R U C T I O N   D  O N E \n")


    #b.show_nice(show_object, recurse=True) # type: ignore
    #b.show_sketches(show_object, recurse=True) # type: ignore
    #b.show_locations(show_object, recurse=False, size=10) # type: ignore

    from build123things.show import show
    show(b, recurse=False)


