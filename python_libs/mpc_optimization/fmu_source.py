from dataclasses import dataclass
from pathlib import Path
import tempfile
import os
import shutil

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
        cwd = os.getcwd()
        with tempfile.TemporaryDirectory() as temp_dir:
            os.chdir(temp_dir)
            modelica_model = ModelicaSystem(str(model_info.location),
                                            model_info.name, ["Modelica"],
                                            commandLineOptions="--fmiFlags=s:cvode -d=initialization")
            modelica_model.setSimulationOptions(["startTime=0", "stopTime=0"])
            modelica_model.simulate()
            fmu_path = modelica_model.convertMo2Fmu(fmuType='me')
            if len(fmu_path) == 0:
                raise RuntimeError("Couldn't compile FMU")
            new_loc = Path(cwd)/Path(fmu_path).name
            shutil.move(Path(fmu_path), new_loc)
            os.chdir(cwd)
        return FmuSource(new_loc)

    @classmethod
    def from_fmu(cls, fmu_path: Path):
        return FmuSource(fmu_path)
