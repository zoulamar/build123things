#!/usr/bin/env python3

from typing import Callable
import build123d as bd
from build123things import MountPoint, ReferenceTransformResolver, Thing
from build123things.colors import BLACK, RED2
from build123things.materials import Aether, Steel
from build123things.partlib.fasteners import MetricScrew, Screw
import build123things.misc

class XSeries (Thing):
    """ Model based on file:///home/zoulamar/Downloads/XM,H,D-540.N101.I101.pdf """

    def __init__(self,
                 height,
                 height_nominal,
                 width,
                 width_nominal,
                 depth,
                 depth_nominal,
                 y_centering_nominal,
                 front_screws_y_to_axis,
                 front_screws_x_span,
                 side_screws_z_span,
                 side_screws_y_to_z_axis,
                 side_screws_y_span,
                 top_screws_x_span,
                 top_screws_z_to_front_edge,
                 bottom_screws_z_span,
                 bottom_screws_x_span,
                 rotor_thickness_1_nominal,
                 rotor_thickness_2_nominal,
                 rotor_thickness_3_nominal,
                 rotor_radius_1,
                 rotor_radius_2,
                 rotor_radius_3,
                 rotor_row_1_spanning_radius,
                 rotor_row_1_screw_diameter,
                 rotor_row_1_screw_count,
                 rotor_row_2_enabled,
                 rotor_row_2_spanning_radius,
                 rotor_row_2_screw_diameter,
                 rotor_row_2_screw_count,
                 bearing_thickness_nominal,
                 bearing_radius,
                 bearing_row_spanning_radius,
                 bearing_row_screw_diameter,
                 bearing_row_screw_count,
                 chamfer_a,
                 chamfer_b,
                 body_screw_depth,
                 body_screw_diameter,
        ) -> None:
        super().__init__(material=Steel(color=BLACK))

        y_centering:float = height * (y_centering_nominal / height_nominal)

        # The body itself.
        hull:bd.Solid = bd.Solid() + bd.Location((0,y_centering,0)) * bd.Box(width, height, depth, align=(bd.Align.CENTER, bd.Align.MAX, bd.Align.CENTER))
        hull:bd.Solid = bd.Solid() + bd.chamfer(hull.edges().filter_by(bd.Axis.Z), chamfer_b)
        hull:bd.Solid = bd.Solid() + bd.chamfer(hull.edges() - hull.edges().filter_by(bd.Axis.Z), chamfer_a)

        # The main plane centers.
        self.front        :MountPoint = MountPoint(bd.Location((0,+y_centering-height/2,depth/2)))
        self.back         :MountPoint = MountPoint(bd.Location((0,+y_centering-height/2,-depth/2), (0,180,0)))
        self.left         :MountPoint = MountPoint(bd.Location((width/2,+y_centering-height/2,0), (0,90,0)))
        self.left_top     :MountPoint = MountPoint(bd.Location((width/2,+y_centering,0), (0,90,0)))
        self.left_bottom  :MountPoint = MountPoint(bd.Location((width/2,+y_centering-height,0), (0,90,0)))
        self.right        :MountPoint = MountPoint(bd.Location((-width/2,+y_centering-height/2,0), (0,-90,0)))
        self.right_top    :MountPoint = MountPoint(bd.Location((-width/2,+y_centering,0), (0,-90,0)))
        self.right_bottom :MountPoint = MountPoint(bd.Location((-width/2,+y_centering-height,0), (0,-90,0)))
        self.top          :MountPoint = MountPoint(bd.Location((0,+y_centering,0), (-90,0,0)))
        self.bottom       :MountPoint = MountPoint(bd.Location((0,+y_centering-height,0), (90,0,0)))

        # The rotor and bearing shafts.
        rotor_thickness_1:float = depth * (rotor_thickness_1_nominal/depth_nominal)
        rotor_thickness_2:float = depth * (rotor_thickness_2_nominal/depth_nominal)
        rotor_thickness_3:float = depth * (rotor_thickness_3_nominal/depth_nominal)
        bearing_thickness = depth * (bearing_thickness_nominal/depth_nominal)
        self.rotor_base = MountPoint(self.front.location * bd.Location((0,height/2-y_centering, 0)))
        self.rotor_center = MountPoint(self.front.location * bd.Location((0,height/2-y_centering, rotor_thickness_1)))
        self.rotor_tip = MountPoint(self.front.location * bd.Location((0,height/2-y_centering, rotor_thickness_1+rotor_thickness_3)))
        self.rotor_sketch:bd.Sketch = bd.Sketch() + self.rotor_center.location * bd.Circle(radius=rotor_radius_1)
        self.rotor_center_sketch:bd.Sketch = bd.Sketch() + self.rotor_center.location * bd.Circle(radius=rotor_radius_2)
        self.bearing_center = self.back.location * bd.Location((0,height/2-y_centering, bearing_thickness))
        self.bearing_sketch:bd.Sketch = bd.Sketch() + self.bearing_center * bd.Circle(radius=bearing_radius)
        hull += bd.Solid() + \
                bd.extrude(self.rotor_sketch, amount=-rotor_thickness_1) + \
                bd.extrude(bd.Sketch() + self.rotor_sketch.location * bd.Circle(radius=rotor_radius_2), amount=rotor_thickness_2) + \
                bd.extrude(bd.Sketch() + self.rotor_sketch.location * bd.Circle(radius=rotor_radius_3), amount=rotor_thickness_3) + \
                bd.extrude(self.bearing_sketch, amount=-bearing_thickness)

        # Now the screws...
        def screw_factory(where:bd.Location, screw_diam:float, screw_depth:float) -> Callable:
            def by_length_above (body_length_above:float=2) -> ReferenceTransformResolver:
                return Thing.align(
                    primary_mount = MountPoint(where * bd.Location((0,0,-screw_depth)), owner=self),
                    secondary_mount = MetricScrew(width=screw_diam, length=body_screw_depth + body_length_above, head_type="hex_inner").tip
                )
            return by_length_above

        self.screw_left_top_front     = screw_factory(bd.Location((width/2, -side_screws_y_to_z_axis, side_screws_z_span/2), (0,90,0)), body_screw_diameter, body_screw_depth)
        self.screw_left_top_rear      = screw_factory(bd.Location((width/2, -side_screws_y_to_z_axis, -side_screws_z_span/2), (0,90,0)), body_screw_depth, body_screw_diameter)
        self.screw_left_bottom_front  = screw_factory(bd.Location((width/2, -side_screws_y_to_z_axis-side_screws_y_span, side_screws_z_span/2), (0,90,0)), body_screw_depth, body_screw_diameter)
        self.screw_left_bottom_rear   = screw_factory(bd.Location((width/2, -side_screws_y_to_z_axis-side_screws_y_span, -side_screws_z_span/2), (0,90,0)), body_screw_depth, body_screw_diameter)
        self.screws_left = [self.screw_left_bottom_front, self.screw_left_bottom_rear, self.screw_left_top_front, self.screw_left_top_rear]

        self.screw_right_top_front    = screw_factory(bd.Location((-width/2, -side_screws_y_to_z_axis, side_screws_z_span/2), (0,-90,0)), body_screw_depth, body_screw_diameter)
        self.screw_right_top_rear     = screw_factory(bd.Location((-width/2, -side_screws_y_to_z_axis, -side_screws_z_span/2), (0,-90,0)), body_screw_depth, body_screw_diameter)
        self.screw_right_bottom_front = screw_factory(bd.Location((-width/2, -side_screws_y_to_z_axis-side_screws_y_span, side_screws_z_span/2), (0,-90,0)), body_screw_depth, body_screw_diameter)
        self.screw_right_bottom_rear  = screw_factory(bd.Location((-width/2, -side_screws_y_to_z_axis-side_screws_y_span, -side_screws_z_span/2), (0,-90,0)), body_screw_depth, body_screw_diameter)
        self.screws_right = [self.screw_right_bottom_front, self.screw_right_bottom_rear, self.screw_right_top_front, self.screw_right_top_rear]

        self.screw_front_left         = screw_factory(bd.Location((front_screws_x_span/2, -front_screws_y_to_axis, depth/2), (0,0,0)), body_screw_depth, body_screw_diameter)
        self.screw_front_right        = screw_factory(bd.Location((-front_screws_x_span/2, -front_screws_y_to_axis, depth/2), (0,0,0)), body_screw_depth, body_screw_diameter)

        self.screw_top_left           = screw_factory(bd.Location((top_screws_x_span/2, y_centering, depth/2-top_screws_z_to_front_edge), (-90,0,0)), body_screw_depth, body_screw_diameter)
        self.screw_top_right          = screw_factory(bd.Location((-top_screws_x_span/2, y_centering, depth/2-top_screws_z_to_front_edge), (-90,0,0)), body_screw_depth, body_screw_diameter)

        self.screw_bottom_left_front  = screw_factory(bd.Location((bottom_screws_x_span/2, y_centering-height, bottom_screws_z_span/2), (90,0,0)), body_screw_depth, body_screw_diameter)
        self.screw_bottom_left_rear   = screw_factory(bd.Location((bottom_screws_x_span/2, y_centering-height, -bottom_screws_z_span/2), (90,0,0)), body_screw_depth, body_screw_diameter)
        self.screw_bottom_right_front = screw_factory(bd.Location((-bottom_screws_x_span/2, y_centering-height, bottom_screws_z_span/2), (90,0,0)), body_screw_depth, body_screw_diameter)
        self.screw_bottom_right_rear  = screw_factory(bd.Location((-bottom_screws_x_span/2, y_centering-height, -bottom_screws_z_span/2), (90,0,0)), body_screw_depth, body_screw_diameter)

        self.screw_rotor_row1_:MetricScrew
        self.screw_rotor_row2_:MetricScrew
        self.screw_rotor_bearing:MetricScrew

        for i in range(bearing_row_screw_count):
            setattr(self, f"screw_bearing_{i}", screw_factory(self.bearing_sketch.location * bd.Location((0,0,0),(360/bearing_row_screw_count*i)) * bd.Location((bearing_row_spanning_radius,0)), bearing_thickness, bearing_row_screw_diameter))

        for i in range(rotor_row_1_screw_count):
            setattr(self, f"screw_rotor_row1_{i}", screw_factory(self.rotor_sketch.location * bd.Location((0,0,0),(360/rotor_row_1_screw_count*i)) * bd.Location((rotor_row_1_spanning_radius,0)), rotor_thickness_1, rotor_row_1_screw_diameter))

        if rotor_row_2_screw_count is not None:
            for i in range(rotor_row_2_screw_count):
                setattr(self, f"screw_rotor_row2_{i}", screw_factory(self.rotor_sketch.location * bd.Location((0,0,0),(360/rotor_row_2_screw_count*i)) * bd.Location((rotor_row_2_spanning_radius,0)), rotor_thickness_2, rotor_row_2_screw_diameter))

        # NOTE: Deprecation warning. Use direct attributes instead.
        self.screw_definitions = {
                # Left
                "+x+y+z" : (body_screw_depth, body_screw_diameter, bd.Location((width/2, -side_screws_y_to_z_axis, side_screws_z_span/2), (0,90,0))),
                "+x+y-z" : (body_screw_depth, body_screw_diameter, bd.Location((width/2, -side_screws_y_to_z_axis, -side_screws_z_span/2), (0,90,0))),
                "+x-y+z" : (body_screw_depth, body_screw_diameter, bd.Location((width/2, -side_screws_y_to_z_axis-side_screws_y_span, side_screws_z_span/2), (0,90,0))),
                "+x-y-z" : (body_screw_depth, body_screw_diameter, bd.Location((width/2, -side_screws_y_to_z_axis-side_screws_y_span, -side_screws_z_span/2), (0,90,0))),

                # Right
                "-x+y+z" : (body_screw_depth, body_screw_diameter, bd.Location((-width/2, -side_screws_y_to_z_axis, side_screws_z_span/2), (0,-90,0))),
                "-x+y-z" : (body_screw_depth, body_screw_diameter, bd.Location((-width/2, -side_screws_y_to_z_axis, -side_screws_z_span/2), (0,-90,0))),
                "-x-y+z" : (body_screw_depth, body_screw_diameter, bd.Location((-width/2, -side_screws_y_to_z_axis-side_screws_y_span, side_screws_z_span/2), (0,-90,0))),
                "-x-y-z" : (body_screw_depth, body_screw_diameter, bd.Location((-width/2, -side_screws_y_to_z_axis-side_screws_y_span, -side_screws_z_span/2), (0,-90,0))),

                # Front
                "+z+x" : (body_screw_depth, body_screw_diameter, bd.Location((front_screws_x_span/2, -front_screws_y_to_axis, depth/2), (0,0,0))),
                "+z-x" : (body_screw_depth, body_screw_diameter, bd.Location((-front_screws_x_span/2, -front_screws_y_to_axis, depth/2), (0,0,0))),

                # Top
                "+y+x" : (body_screw_depth, body_screw_diameter, bd.Location((top_screws_x_span/2, y_centering, depth/2-top_screws_z_to_front_edge), (-90,0,0))),
                "+y-x" : (body_screw_depth, body_screw_diameter, bd.Location((-top_screws_x_span/2, y_centering, depth/2-top_screws_z_to_front_edge), (-90,0,0))),

                # Bottom
                "-y+x+z" : (body_screw_depth, body_screw_diameter, bd.Location((bottom_screws_x_span/2, y_centering-height, bottom_screws_z_span/2), (90,0,0))),
                "-y+x-z" : (body_screw_depth, body_screw_diameter, bd.Location((bottom_screws_x_span/2, y_centering-height, -bottom_screws_z_span/2), (90,0,0))),
                "-y-x+z" : (body_screw_depth, body_screw_diameter, bd.Location((-bottom_screws_x_span/2, y_centering-height, bottom_screws_z_span/2), (90,0,0))),
                "-y-x-z" : (body_screw_depth, body_screw_diameter, bd.Location((-bottom_screws_x_span/2, y_centering-height, -bottom_screws_z_span/2), (90,0,0))),
        }

        # Bearing row
        self.screw_definitions.update({
            f"b{i}" : (bearing_thickness, bearing_row_screw_diameter, self.bearing_sketch.location * bd.Location((0,0,0),(360/bearing_row_screw_count*i)) * bd.Location((bearing_row_spanning_radius,0))) for i in range(bearing_row_screw_count)
        })

        # Rotor row 1
        self.screw_definitions.update({
            f"ra{i}" : (rotor_thickness_1, rotor_row_1_screw_diameter, self.rotor_sketch.location * bd.Location((0,0,0),(360/rotor_row_1_screw_count*i)) * bd.Location((rotor_row_1_spanning_radius,0))) for i in range(rotor_row_1_screw_count)
        })

        # Rotor row 2
        if rotor_row_2_enabled:
            self.screw_definitions.update({
                f"rb{i}" : (rotor_thickness_1, rotor_row_2_screw_diameter, self.rotor_sketch.location * bd.Location((0,0,0),(360/rotor_row_2_screw_count*i)) * bd.Location((rotor_row_2_spanning_radius,0))) for i in range(rotor_row_2_screw_count)
            })

        # Assign the resulting body.
        self.hull = hull

    @property
    def hull_with_screws(self, clearance_face_to_face=0.1):
        return self.hull - [self.screw(name, 0).adjust(width__add=clearance_face_to_face*2).body for name in self.screw_definitions.keys()] # type: ignore

    def screw(self, which:str, length_above:float, head_type="hex_inner") -> Screw:
        # NOTE: Deprecation.
        depth, diameter, loc = self.screw_definitions[which]
        mount = MountPoint(loc * bd.Location((0,0,-depth)), self, None)
        ret = MetricScrew(diameter,depth+length_above,head_type=head_type)
        return ret.align(mount, ret.tip)

    def result(self) -> bd.Part:
        return self.hull # type: ignore

class XM540 (XSeries):
    def __init__(self) -> None:
        super().__init__(
                height = 58.5,
                height_nominal = 58.5,
                width = 33.5,
                width_nominal = 33.5,
                depth = 44.0,
                depth_nominal = 44.0,
                y_centering_nominal = 13.75,
                front_screws_y_to_axis = 10.5,
                front_screws_x_span = 27.0,
                side_screws_z_span = 16.0,
                side_screws_y_to_z_axis = 6.0,
                side_screws_y_span = 32.0,
                top_screws_x_span = 20.0,
                top_screws_z_to_front_edge = 14.0,
                bottom_screws_z_span = 16.0,
                bottom_screws_x_span = 20.0,
                rotor_thickness_1_nominal = 2.6,
                rotor_thickness_2_nominal = 2.3,
                rotor_thickness_3_nominal = 3.0,
                rotor_radius_1 = 26.0/2.0,
                rotor_radius_2 = 10.0/2.0,
                rotor_radius_3 = 5.3/2.0,
                rotor_row_1_spanning_radius=22.0/2.0,
                rotor_row_1_screw_diameter=2.5,
                rotor_row_1_screw_count=8,
                rotor_row_2_enabled=True,
                rotor_row_2_spanning_radius=16.0/2.0,
                rotor_row_2_screw_diameter=2.0,
                rotor_row_2_screw_count=4,
                bearing_thickness_nominal=2.5,
                bearing_radius=26.0/2.0,
                bearing_row_spanning_radius=22.0/2.0,
                bearing_row_screw_diameter=2.5,
                bearing_row_screw_count=8,
                chamfer_a=0.5,
                chamfer_b=2.0,
                body_screw_depth = 4.0,
                body_screw_diameter = 2.5
            )

    def mass(self, _=False) -> float:
        return 0.165

class XM430 (XSeries):
    def __init__(self) -> None:
        super().__init__(
                height = 46.5,
                height_nominal = 46.5,
                width = 28.5,
                width_nominal = 28.5,
                depth = 34.0,
                depth_nominal = 34.0,
                y_centering_nominal = 11.25,
                front_screws_y_to_axis = 8.0,
                front_screws_x_span = 22.0,
                side_screws_z_span = 12.0,
                side_screws_y_to_z_axis = 4.0,
                side_screws_y_span = 24.0,
                top_screws_x_span = 16.0,
                top_screws_z_to_front_edge = 11.0,
                bottom_screws_z_span = 12.0,
                bottom_screws_x_span = 16.0,
                rotor_thickness_1_nominal = 2.0,
                rotor_thickness_2_nominal = 2.0,
                rotor_thickness_3_nominal = 2.5,
                rotor_radius_1 = 19.5/2.0,
                rotor_radius_2 = 8.0/2.0,
                rotor_radius_3 = 4.3/2.0,
                rotor_row_1_spanning_radius=16.0/2.0,
                rotor_row_1_screw_diameter=2.0,
                rotor_row_1_screw_count=8,
                rotor_row_2_enabled=False,
                rotor_row_2_spanning_radius=None,
                rotor_row_2_screw_diameter=None,
                rotor_row_2_screw_count=None,
                bearing_thickness_nominal=2.0,
                bearing_radius=19.5/2.0,
                bearing_row_spanning_radius=16.0/2.0,
                bearing_row_screw_diameter=2.0,
                bearing_row_screw_count=8,
                chamfer_a=0.5,
                chamfer_b=2.0,
                body_screw_depth = 3.0,
                body_screw_diameter = 2.5
            )

    def mass(self, _=False) -> float:
        return 0.082

if __name__ == "__main__" or build123things.misc.is_in_cq_editor():
    d5 = XM540()

    #show_object(d5.result())

    #for n, v in d5.enumerate_reference_geometries():
    #    show_object(v)

    #print("Created: ", d5)
    #d5_ii = XM540()
    #print("Created: ", d5_ii)
    #d5_m = d5.adjust(height__add=10)
    #print("Created: ", d5_m)
    #d5_m2 = d5.adjust(height__add=10)
    #print("Created: ", d5_m2)
    from build123things.show import show
