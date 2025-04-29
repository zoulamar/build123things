import build123d as bd
from build123things import MountPoint, Thing
from build123things.materials import Material, Steel, Brass
import build123things

"""
A library containing metric screw mockups, i.e., without thread.

TODO: Add symbolic thread object for copmpatibility purposes.
"""

class Screw (Thing):
    """ A simple geometric dummy screws without threads.

    TODO: Allow thread type annotation for part list and compatibility check purposes.
    """

    def __init__(self, width:float, length:float, head_width:float, head_length:float, head_depth:float, head_type:str, material:Material=Steel()) -> None:
        #, thread_profile:str, thread_depth:float, thread_angle:float
        super().__init__(material=material)

        cutout_start_loc = bd.Location((0,0,-head_length+head_depth), (180,0,0))

        self.sketch_body:bd.Sketch = bd.Circle(radius=width/2)
        self.cutout_start = MountPoint(cutout_start_loc)
        self.sketch_head_circumcircle:bd.Sketch = cutout_start_loc * bd.Circle(head_width/2) #type:ignore
        if head_type == "hex_inner":
            self.sketch_head_outline:bd.Sketch = bd.Circle(head_width/2)
            self.sketch_head_cutout:bd.Sketch = bd.Sketch() +  cutout_start_loc * bd.RegularPolygon(radius=head_width*.4,side_count=6)
        elif head_type == "hex_outer":
            self.sketch_head_outline:bd.Sketch = bd.RegularPolygon(radius=head_width/2,side_count=6)
        elif head_type == "line":
            self.sketch_head_outline:bd.Sketch = bd.Circle(head_width/2)
            self.sketch_head_cutout:bd.Sketch = bd.Sketch() + cutout_start_loc * bd.Rectangle(head_width*.8, head_width*.1)
        elif head_type == "cross":
            self.sketch_head_outline:bd.Sketch = bd.Circle(head_width/2)
            self.sketch_head_cutout:bd.Sketch = bd.Sketch() + cutout_start_loc * (bd.Rectangle(head_width*.8, head_width*.1) + bd.Rectangle(head_width*.1, head_width*.8))
        else:
            raise ValueError(f"Screw heads type {head_type} not supported.")

        self.tip = MountPoint(bd.Location((0,0,length),(0,0,0)))
        self.base = MountPoint(bd.Location((0,0,0),(0,0,0)))
        self.head = MountPoint(bd.Location((0,0,-head_length),(180,0,0)))

        self.body_hull:bd.Part = bd.extrude(self.sketch_body, amount=length) + bd.extrude(self.sketch_head_outline, amount=-head_length)
        if hasattr(self, "sketch_head_cutout"):
            self.body = self.body_hull - bd.extrude(self.sketch_head_cutout, amount=head_depth)
        else:
            self.body = self.body_hull

    def result(self) -> bd.Part:
        return self.body

    def nut_location(self, t:float) -> bd.Location:
        """  """
        return bd.Location((0,0,self.p.length * (1-t)), (0,0,0))

    def get_insert(self, wall_thickness, length=None):
        if length is None:
            length = self.p.length
        return bd.extrude(bd.Circle(radius=(self.p.width + wall_thickness)/2), amount=length) # type: ignore

    def adjust_fdm_cutin(self):
        """ Returns a Thing useful for subtracting, such that in practice the screw can cut in the FDM-printed mass."""
        return self.adjust(width__add=0.15)

    def adjust_fdm_clear(self):
        """ Returns a Thing useful for subtracting, such that in practice the screw goes smoothly through the FDM-printed mass."""
        return self.adjust(width__add=0.3)

class MetricScrew(Screw):
    def __init__(self, width:float, length:float, head_type:str="hex_outer", material:Material=Steel()) -> None:
        super().__init__(
            material=material,
            width=width,
            length=length,
            head_width=1.8*width,
            head_length=width,
            head_depth=0.5*width,
            head_type=head_type)

class ScrewM2(MetricScrew):
    def __init__(self, length:float, head_type:str="hex_outer", material:Material=Steel()) -> None:
        super().__init__(2, length, head_type, material)

class Insert (Thing):
    def __init__(self, diam_thread, diam_outer, length, material=Brass()) -> None:
        raise NotImplementedError
        super().__init__(material=material)
        self.sketch = bd.RegularPolygon(radius=head_width, side_count=6) - bd.Circle(radius=width/2)
        self.result = bd.extrude(
            self.sketch,
            amount=-height
        )

class Nut (Thing):
    def __init__(self, width, head_width, height, material_type=Steel) -> None:
        super().__init__(material=material_type())
        self.sketch = bd.RegularPolygon(radius=head_width, side_count=6) - bd.Circle(radius=width/2)
        self.result = bd.extrude(
            self.sketch,
            amount=-height
        )

class MetricNut (Nut):
    def __init__(self, width) -> None:
        super().__init__(width, width*1.2, width*.5)

if __name__ == "__main__" or build123things.misc.is_in_cq_editor():
    s1 = MetricScrew(width=2.5,length=30,head_type="hex_inner")
    s2 = MetricScrew(width=2.5,length=30,head_type="hex_inner")
    sm = s1.adjust(width__add = 1)
    from build123things.show import show
