
    @final
    def get_sketches(self) -> Dict[str, bd.Sketch]:
        """ Retrieves a list of all sketches such that they may be exported. """

        raise NotImplementedError("This function needs to be checked if it still does the right thing.")

        print(f"Fetching sketches objects for {self}")

        def annotate_dict(olddict:Dict[str, bd.Sketch], annotation:str) -> Dict[str,bd.Sketch]:
            """ Helper function that renames all element names, prepending a namespace-like annotation prefix. """
            ret:Dict[str,bd.Sketch] = {}
            for name, value in olddict.items():
                assert isinstance(value, bd.Sketch)
                ret[annotation + "_" + name] = value
            return ret

        ret = {}

        for name, value in self.__dict__.items():
            if isinstance(value, bd.Sketch):
                ret[name] = value
            elif isinstance(value, Thing):
                ret.update(annotate_dict(value.get_sketches(), name))

        return ret
    @staticmethod
    def export_sketches_to_dxf (partdict:Dict[str,bd.Sketch], output_dir:Union[None,Path]=None):
        t = datetime.datetime.now()
        if output_dir is None:
            output_dir = Path(f"./build/{str(t.year)[2:]}{t.month:02}{t.day:02}_{t.hour:02}{t.minute:02}{t.second:02}")
        output_dir.mkdir(parents=True, exist_ok=True)
        for name, part in partdict.items():
            part.export_dxf(str((output_dir / (name+".dxf")).resolve())) # type: ignore

