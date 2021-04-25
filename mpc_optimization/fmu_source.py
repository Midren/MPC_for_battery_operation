from pathlib import Path


class FmuSource:
    def __init__(self, fmu_path):
        self.fmu_path: Path = fmu_path

    @classmethod
    def from_modelica(cls, model_name: str, model_location: Path):
        fmu_path = model_name + model_location
        # TODO: add compiling via OMPython
        # fmu_path = compile_fmu(model_name,
        #                        model_location,
        #                        target='cs',
        #                        version="2.0",
        #                        compiler_log_level='warning',  # 'info', 'warning',
        #                        compiler_options={"generate_html_diagnostics": True,
        #                                           "nle_solver_tol_factor": 1e-2})  # 1e-2 is the default
        return FmuSource(Path(fmu_path))

    @classmethod
    def from_fmu(cls, fmu_path: Path):
        return FmuSource(fmu_path)
