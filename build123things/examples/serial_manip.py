from build123things import MountPoint, Thing, misc, MOUNTING_LOCATION
import build123d as bd
from build123things.colors import Color
from build123things.joints import Revolute, Rigid
from build123things.materials import PETG, Aether
from build123things.partlib.dynamixel import XM430
from build123things.partlib.fasteners import MetricScrew

# Declare aliases.
MIN=bd.Align.MIN
MAX=bd.Align.MAX
CENTER=bd.Align.CENTER

# Prepare a slightly bigger version of the XM430 servomotor to account for manufacturing imprecisions.
xm430_clearance = XM430().adjust(
    rotor_radius_2__add=0.2,
    width__add=0.2,
    height__add=0.2,
)

# Set default angles of the manipulator
ANGLES=(30,-60,120,-30)

class LinkBase (Thing):
    def __init__(self) -> None:
        super().__init__(Aether())
        self.base_plate = MountPoint(bd.Location())
        Rigid(self.base_plate,Connector_Base_Plate().mount_origin)
        self.servo = MountPoint(self.base_plate.mount_servo_back)
        Rigid(self.servo, XM430().back)
        self.shoulder = MountPoint(self.servo.rotor_center)
        self.joint = Revolute(self.shoulder, LinkShoulder().base)
        self.joint.set(ANGLES[0])
    def result(self) -> None:
        return None

class LinkShoulder (Thing):
    def __init__(self) -> None:
        super().__init__(Aether())
        self.connector = MountPoint(bd.Location())
        Rigid(self.connector, Connector_Yaw_Pitch().origin)
        self.servo = MountPoint(self.connector.servo_pitch_mount)
        Rigid(self.servo, XM430().top)
        self.humerus = MountPoint(self.servo.rotor_center)
        self.joint = Revolute(self.humerus, LinkHumerus().mount_by_braces)
        self.joint.set(ANGLES[1])
        self.base = MountPoint(self.connector.servo_yaw_mount)
    def result(self) -> None:
        return None

class LinkProtoArm (Thing):
    def __init__(self) -> None:
        super().__init__(Aether())
        self.servo = MountPoint(bd.Location())
        Rigid(self.servo, XM430().origin)
        self.brace_a = MountPoint(self.servo.bottom)
        Rigid(self.brace_a, Connector_Rotor_Body().mount_bottom)
        self.brace_b = MountPoint(self.servo.bottom.location * bd.Location((0,0,0), (0,0,180)))
        Rigid(self.brace_b, Connector_Rotor_Body().mount_bottom)
        self.mount_by_braces = MountPoint(self.brace_a.mount_rotor)
    def result(self) -> None:
        return None

class LinkHumerus (LinkProtoArm):
    def __init__(self) -> None:
        super().__init__()
        self.forearm = MountPoint(self.servo.rotor_center.location * bd.Location((0,0,0),(0,0,180)))
        self.joint = Revolute(self.forearm, LinkForearm().mount_by_braces)
        self.joint.set(ANGLES[2])

class LinkForearm (LinkProtoArm):
    def __init__(self) -> None:
        super().__init__()
        self.ee = MountPoint(self.servo.rotor_center)
        self.joint = Revolute(self.ee, EndEffector().mount)
        self.joint.set(ANGLES[3])

class PenDummy (Thing):
    def __init__(self, length:float=100,diameter:float=15) -> None:
        super().__init__(PETG(color=Color(.1,.1,.9,0)))
        self.body = bd.fillet(bd.Cylinder(height=length,radius=diameter/2).edges().sort_by(bd.Axis.Z)[0], radius=diameter*.5) + bd.Location((0,0,length/2)) * bd.Cone(bottom_radius=diameter/2,top_radius=0,height=length*.1, align=(CENTER,CENTER,MIN))

    def result(self) -> bd.Part | None:
        return bd.Part() + self.body

class EndEffector (Thing):
    """  """
    def __init__(self, servo_clearance=22, thickness=3, pen_diameter=15) -> None:
        super().__init__(PETG())
        self.servo_ref:XM430 = xm430_clearance.move(bd.Location())
        in_front = self.servo_ref.rotor_center.location.position + bd.Vector(0,servo_clearance,0)
        sweep_trajectory = bd.Polyline(
            self.servo_ref.rotor_center.location.position,
            in_front,
            self.servo_ref.bearing_center.position + bd.Vector(0,servo_clearance,0),
        )
        sweep_profile = self.servo_ref.rotor_center.location * bd.Location((0,0,0),(90,0,0)) * bd.Rectangle(width=self.servo_ref.p.rotor_radius_1*2+thickness*2, height=thickness, align=(CENTER,MIN))
        body = bd.sweep(sweep_profile,sweep_trajectory,transition=bd.Transition.ROUND) + bd.extrude(self.servo_ref.rotor_center.location * bd.Circle(self.servo_ref.p.rotor_radius_1+thickness), amount=thickness) - bd.extrude(self.servo_ref.rotor_center.location * bd.Circle(self.servo_ref.p.rotor_radius_2), amount=thickness)
        for i in range(8):
            body -= getattr(self.servo_ref, f"screw_rotor_row1_{i}")(1.5).body_hull
        self.cap = MountPoint(bd.Location(bd.Vector(0,servo_clearance+thickness,0), (90,-90,0)))
        Rigid(self.cap, EndEffectorCap().mount)
        body -= self.cap.screw_1.body_hull
        body -= self.cap.screw_2.body_hull
        self.mount = MountPoint(self.servo_ref.rotor_center.location * MOUNTING_LOCATION)
        self.body = body

    def result(self) -> bd.Part | None:
        return self.body # type: ignore

class EndEffectorCap (Thing):
    def __init__(self, thickness=3, pen_diameter=15) -> None:
        super().__init__(PETG(Color(1.,.4,.4,0)))

        start_x = pen_diameter/2
        screw_x = start_x + thickness*2
        limit_x = screw_x + thickness
        self.sweep_trajectory = bd.Curve() + bd.Polyline(
            (limit_x,0),
            (start_x,0),
            (start_x,pen_diameter/2),
        ) + bd.ThreePointArc(
            (start_x,pen_diameter/2),
            (0,pen_diameter),
            (-start_x,pen_diameter/2),
        ) + bd.Polyline(
            (-start_x,pen_diameter/2),
            (-start_x,0),
            (-limit_x,0),
        )
        self.sweep_profile = bd.Location((limit_x, 0, 0), (90,-90,90)) * bd.Rectangle(width=10, height=thickness, align=(CENTER,MIN))
        self.screw_1 = MountPoint(bd.Location((screw_x,1.5,0),(-90,0,0)))
        Rigid(self.screw_1, MetricScrew(3,5).base)
        self.screw_2 = MountPoint(bd.Location((-screw_x,1.5,0),(-90,0,0)))
        Rigid(self.screw_2, MetricScrew(3,5).base)
        self.body = (
            bd.sweep(self.sweep_profile, self.sweep_trajectory, transition=bd.Transition.ROUND) -
            self.screw_1.body_hull -
            self.screw_2.body_hull
        )
        self.mount = MountPoint(bd.Location((0,0,0),(0,0,0)))
        self.pen = MountPoint(bd.Location((0,pen_diameter/2,0),(0,0,0)))
        Rigid(self.pen, PenDummy().origin)

    def result(self) -> bd.Part | None:
        return self.body # type: ignore

class Connector_Yaw_Pitch (Thing):
    def __init__(self, servo_extra_clearance=4, material_thickness=3) -> None:
        super().__init__(PETG())
        self.servo_yaw_ref:XM430 = xm430_clearance.move(bd.Location())
        self.servo_yaw_mount = MountPoint(self.servo_yaw_ref.rotor_center.location * MOUNTING_LOCATION)

        self.servo_pitch_mount = MountPoint(self.servo_yaw_ref.rotor_center.location * bd.Location((0,0,servo_extra_clearance)))
        self.servo_pitch_ref = Thing.align(self.servo_pitch_mount, xm430_clearance.top)

        box = self.servo_yaw_ref.rotor_sketch.location * bd.Box(
            width=xm430_clearance.p.width + 2*material_thickness,
            length=xm430_clearance.p.depth - 2*material_thickness,
            height=xm430_clearance.p.depth * 0.8,
            align=(CENTER,CENTER,MIN)
        )

        self.body = (
            bd.fillet(box.edges(), material_thickness)
            - self.servo_pitch_ref.hull # type: ignore
            - self.servo_yaw_ref.hull # type: ignore
            - bd.extrude(self.servo_yaw_ref.rotor_base.location * bd.Circle(self.servo_yaw_ref.p.rotor_radius_2), amount=100, both=True)
            - self.servo_yaw_ref.screw_rotor_row1_0(1.5).adjust(width__add=.1).body_hull # type: ignore
            - self.servo_yaw_ref.screw_rotor_row1_2(1.5).adjust(width__add=.1).body_hull # type: ignore
            - self.servo_yaw_ref.screw_rotor_row1_4(1.5).adjust(width__add=.1).body_hull # type: ignore
            - self.servo_yaw_ref.screw_rotor_row1_6(1.5).adjust(width__add=.1).body_hull # type: ignore
            - self.servo_pitch_ref.screw_right_top_rear(1.5).adjust(width__add=.1).body_hull # type: ignore
            - self.servo_pitch_ref.screw_right_top_front(1.5).adjust(width__add=.1).body_hull # type: ignore
            - self.servo_pitch_ref.screw_left_top_rear(1.5).adjust(width__add=.1).body_hull # type: ignore
            - self.servo_pitch_ref.screw_left_top_front(1.5).adjust(width__add=.1).body_hull # # type: ignore
        )

    def result(self) -> bd.Part | None:
        return self.body

class Connector_Rotor_Body (Thing):
    def __init__(self, rotor_rotor_length=100, thickness=3) -> None:
        super().__init__(material=PETG())
        self.servo_a:XM430 = xm430_clearance.move(bd.Location((0,0,0),(0,0,180)))
        self.servo_b:XM430 = xm430_clearance.move(bd.Location((0,rotor_rotor_length,0),(0,0,0)))

        sweep_start = self.servo_a.rotor_center.location.position
        sweep_end = (0, self.servo_b.bottom.location.position.Y - thickness, 1)
        sweep_mid = bd.Vector(0, self.servo_b.bottom.location.position.Y - thickness, sweep_start.Z)
        sweep_curve = bd.Line(sweep_start, sweep_mid) + bd.Line(sweep_mid, sweep_end)

        body = (bd.sweep(
                xm430_clearance.rotor_center.location * bd.Location((0,0,0),(90,0,0)) * bd.Rectangle(
                    width=xm430_clearance.p.width,
                    height=thickness,
                    align=(CENTER,MIN)
                ),
                sweep_curve,
                transition=bd.Transition.ROUND
            )
        )
        rotor_mount = ( bd.extrude(
                    self.servo_a.rotor_center.location * bd.Circle(radius=xm430_clearance.p.width/2),
                    amount=thickness
                )
            )

        body += rotor_mount
        body -= self.servo_a.hull
        body -= bd.extrude(
            self.servo_a.rotor_center_sketch,
            amount=thickness
        )
        body = bd.fillet(body.edges().group_by(bd.Axis.Z)[-1], radius=2) # type: ignore
        body = bd.fillet(body.edges().group_by(bd.Axis.Y)[-1], radius=2.5) # type: ignore

        for i in range(8):
            body -= getattr(self.servo_a, f"screw_rotor_row1_{i}")(1.5).body_hull
        body -= self.servo_b.screw_bottom_left_front(1.5).body_hull
        body -= self.servo_b.screw_bottom_right_front(1.5).body_hull

        self.body=body

        self.mount_rotor = MountPoint(self.servo_a.rotor_center.location * MOUNTING_LOCATION)
        self.mount_bottom = MountPoint(self.servo_b.bottom.location * MOUNTING_LOCATION)

    def result(self) -> bd.Part | None:
        return self.body

class Connector_Base_Plate (Thing):
    def __init__(self, thickness:float = 5, size:float=100) -> None:
        super().__init__(PETG())
        self.servo = xm430_clearance.back.align_to(bd.Location())
        body = bd.Box(size,size,thickness,align=(CENTER,CENTER,MAX)) - self.servo.hull

        pad = bd.Box(self.servo.p.depth, self.servo.p.height, thickness, align=(CENTER,CENTER,MIN))

        body += self.servo.left.location * pad
        body += self.servo.right.location * pad
        body = bd.fillet(body.edges().filter_by(bd.Axis.Z), 1)
        body = bd.fillet(body.edges().group_by(bd.Axis.Z)[-1], 1) # type: ignore

        body -= self.servo.screw_left_top_rear(thickness*.8).body_hull
        body -= self.servo.screw_left_top_front(thickness*.8).body_hull
        body -= self.servo.screw_left_bottom_rear(thickness*.8).body_hull
        body -= self.servo.screw_left_bottom_front(thickness*.8).body_hull

        body -= self.servo.screw_right_top_rear(thickness*.8).body_hull
        body -= self.servo.screw_right_top_front(thickness*.8).body_hull
        body -= self.servo.screw_right_bottom_rear(thickness*.8).body_hull
        body -= self.servo.screw_right_bottom_front(thickness*.8).body_hull

        self.body = body

        self.mount_servo_back = MountPoint(self.servo.back.location * MOUNTING_LOCATION)
        self.mount_origin = MountPoint(bd.Location() * MOUNTING_LOCATION)

    def result(self) -> bd.Part | None:
        return self.body

if misc.is_in_cq_editor() or __name__ == "__main__":
    from build123things.show import show
    #r = LinkBase()
    #r = Connector_Yaw_Pitch()
    #r = LinkHumerus()
    r = EndEffector()
    #r = EndEffectorCap()

