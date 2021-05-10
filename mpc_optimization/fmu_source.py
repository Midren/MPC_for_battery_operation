from dataclasses import dataclass
from pathlib import Path

from OMPython import ModelicaSystem


@dataclass
class ModelicaModelInfo:
    location: Path
    name: str


class FmuSource:
    def __init__(self, fmu_path):
        self.fmu_path: Path = fmu_path

    @classmethod
    def from_modelica(cls, model_info: ModelicaModelInfo):
        if not model_info.location.exists():
            raise ValueError('No such file: ' + str(model_info.location))
        modelica_model = ModelicaSystem(str(model_info.location),
                                        model_info.name, ["Modelica"],
                                        commandLineOptions="--fmiFlags=s:cvode -d=initialization")
        modelica_model.setSimulationOptions(["startTime=0", "stopTime=0"])
        modelica_model.simulate()
        fmu_path = modelica_model.convertMo2Fmu(fmuType='cs')
        if not len(fmu_path):
            raise RuntimeError("Couldn't compile FMU")
        return FmuSource(Path(fmu_path))

    @classmethod
    def from_fmu(cls, fmu_path: Path):
        return FmuSource(fmu_path)
