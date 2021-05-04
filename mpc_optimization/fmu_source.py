from pathlib import Path

from OMPython import ModelicaSystem


class FmuSource:
    def __init__(self, fmu_path):
        self.fmu_path: Path = fmu_path

    @classmethod
    def from_modelica(cls, model_location: Path, model_name: str):
        if not model_location.exists():
            raise ValueError('No such file: ' + str(model_location))
        modelica_model = ModelicaSystem(str(model_location),
                                        model_name, ["Modelica"],
                                        commandLineOptions="--fmiFlags=s:cvode")
        fmu_path = modelica_model.convertMo2Fmu(fmuType='cs')
        if not len(fmu_path):
            raise RuntimeError("Couldn't compile FMU")
        return FmuSource(Path(fmu_path))

    @classmethod
    def from_fmu(cls, fmu_path: Path):
        return FmuSource(fmu_path)
