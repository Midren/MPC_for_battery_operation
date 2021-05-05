import os
import tempfile
import typing as t
from pathlib import Path

import numpy as np
import pandas as pd
from OMPython import OMCSessionZMQ
from pyfmi import fmi, load_fmu

from .fmu_source import FmuSource, ModelicaModelInfo


def _generate_control_csv(control_df: pd.DataFrame) -> Path:
    fd, filepath = tempfile.mkstemp()
    # filepath = '/home/developer/ipynotebooks/inputs.csv'
    try:
        control_df.to_csv(filepath, index=False, line_terminator=',\n', sep=',')
    except:
        os.remove(filepath)
        raise
    return Path(filepath)


def linearize_model(model_info: ModelicaModelInfo, control_df: t.Optional[pd.DataFrame] = None) -> fmi.FMUModelCS2:
    omc = OMCSessionZMQ()
    is_loaded: bool = omc.sendExpression(f'loadFile("{model_info.location}")')
    if not is_loaded:
        raise RuntimeError("Could not load model: ")
    if control_df is None:
        linearization_result = omc.sendExpression(
            f'linearize({model_info.name}, startTime=0, stopTime=0, outputFormat="csv")'
        )
    else:
        path_to_csv = _generate_control_csv(control_df.reset_index())
        stopTime = control_df.index[-1]
        linearization_result = omc.sendExpression(
            f'linearize({model_info.name}, startTime=0, stopTime={stopTime}, simflags="-csvInput {path_to_csv}", outputFormat="csv")'
        )
        os.remove(path_to_csv)

    if linearization_result is None or not len(linearization_result['resultFile']):
        raise RuntimeError("Could not linearize a model: ")

    fmu_path = FmuSource.from_modelica(
        ModelicaModelInfo(Path("linearized_model.mo"), "linearized_model")).fmu_path
    model = load_fmu(str(fmu_path))

    # FMU exported from OpenModelica doesn't estimate from time 0,
    # so simulation from 0 to 0 helps
    opts = model.simulate_options()
    opts['silent_mode'] = True
    model.simulate(0, 0, options=opts)

    return model


def _read_model_matrix(model: fmi.FMUModelCS2, matrix_name: str, shape: t.Tuple[int, int]):
    M = np.zeros(shape)
    for i in range(1, shape[0] + 1):
        for j in range(1, shape[1] + 1):
            M[i - 1, j - 1] = model.get(f'{matrix_name}[{i},{j}]')
    return M


def get_linear_model_matrices(model: fmi.FMUModelCS2):
    x_num = int(model.get('n'))
    u_num = int(model.get('m'))
    y_num = int(model.get('p'))

    A = _read_model_matrix(model, 'A', (x_num, x_num))
    B = _read_model_matrix(model, 'B', (x_num, u_num))
    C = _read_model_matrix(model, 'C', (y_num, x_num))
    D = _read_model_matrix(model, 'D', (y_num, u_num))
    return A, B, C, D
