import numpy as np
import pandas as pd
import typing as t

from filterpy.kalman import KalmanFilter


def V_ocv(soc):
    a = [-5.863e-1, 21.9, 3.414, 1.102e-1, -1.718e-1, 8.0e-3]
    return a[0] * np.exp(-a[1] * soc) + a[2] + a[3] * soc + a[4] * np.exp(-a[5] / (1 - soc))


def create_param_estimation_filter() -> KalmanFilter:
    estimator = KalmanFilter(dim_x=4, dim_z=1)
    estimator.Q = np.diag([0.1, 1e-7, 1e-7, 1e-13])
    estimator.R = np.ones((1, 1)) * 3e-2
    estimator.P = np.identity(4) * 1e5

    estimator.x = np.array([0.99, 6.1e-3, -1.2e-2, 6.1e-3])

    estimator.F = np.identity(4)
    estimator.B = np.zeros(4)
    return estimator


def create_ocv_estimation_filter() -> KalmanFilter:
    estimator = KalmanFilter(dim_x=2, dim_z=1)
    estimator.Q = np.diag([1e-6, 1e-4])
    estimator.R = np.ones((1, 1)) * 3e-7
    estimator.P = np.diag([1, 100])

    estimator.x = np.array([3.5, 0])

    estimator.H = np.ones((1, 2))
    return estimator


def prepare_estimation_inputs(sim_res: t.Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]) -> t.Tuple[np.ndarray, t.Sequence[float], float]:
    current = sim_res[1]['I_req'].values
    voltage = sim_res[2]['batteryOutput'].values
    sample_period = np.diff(sim_res[0]['time'].values, 1)[0]

    measurement_projs = np.vstack([
        np.diff(voltage, 1, prepend=0),
        np.roll(current, 2),
        np.roll(current, 1),
        np.roll(current, 0),
    ]).T[2:]

    return measurement_projs, voltage, sample_period


def estimate_ocv_and_internal_resis(measurement_projs: np.ndarray, voltages: t.Sequence[float], sample_period: float) -> t.Tuple[t.Sequence[float], t.Sequence[float]]:
    param_est_filter = create_param_estimation_filter()
    ocv_est_filter = create_ocv_estimation_filter()

    logs = []
    res = []

    for v_k, phi in zip(voltages[2:], measurement_projs):
        param_est_filter.predict()
        param_est_filter.update(v_k, H=phi.reshape(-1, 4))

        a2, b0, b1, b2 = param_est_filter.x

        R_0 = b0
        R_1 = (a2 * a2 * b0 + a2 * b1 + b2) / (1 - a2)**2
        C_1 = sample_period * (1 - a2) / (a2 * a2 * b0 + a2 * b1 + b2)
        C_ocv = sample_period * (1 - a2) / (b0 + b1 + b2)

        y = phi.reshape(-1, 4).dot(param_est_filter.x)

        ocv_est_filter.F = np.diag([1, 1 - sample_period / (R_1 * C_1)])
        ocv_est_filter.B = np.array([sample_period / C_ocv, sample_period / C_1])

        ocv_est_filter.predict(u=phi[1])
        ocv_est_filter.update(v_k - R_0 * phi[1])
        v_ocv = ocv_est_filter.x[0]

        logs.append((y, v_k, R_0, R_1, C_1, C_ocv, v_ocv))
        res.append((v_ocv, R_0))

    open_circuit_voltage, internal_resis = list(zip(*res))
    return open_circuit_voltage, internal_resis


def calculate_soc(open_circuit_voltage: float) -> float:
    lookup_table = []
    for i in np.linspace(1e-6, 1 - 1e-6, 1000):
        lookup_table.append((i, V_ocv(i)))
    lookup_table = np.array(lookup_table)

    soc = lookup_table[np.argmin(np.abs(lookup_table[:, 1] - open_circuit_voltage)), 0]

    return soc


def calculate_soh(internal_resis: float) -> float:
    # TODO: substitute by adequate values
    R_0 = internal_resis
    R_0_end = np.nan
    R_0_start = np.nan
    soh = (R_0_end - R_0)/(R_0_end - R_0_start)*100
    return soh


def estimate_state(sim_res: t.Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]) -> t.Tuple[float, float]:
    measurement_projs, voltage, sample_period = prepare_estimation_inputs(sim_res)
    open_circuit_voltage, internal_resis = estimate_ocv_and_internal_resis(measurement_projs, voltage, sample_period)

    soc = calculate_soc(open_circuit_voltage[-1])
    soh = calculate_soh(internal_resis[-1])

    return soc, soh


if __name__ == "__main__":
    from pathlib import Path

    from mpc_optimization.battery_model_constants import Battery_state_vars, State_vars_aliases_dict, C_rate_per_second
    from mpc_optimization.fmu_source import FmuSource, ModelicaModelInfo
    from mpc_optimization.mpc_optimizer import MPCOptimizer, ModelVariables, VariableConstraint

    mpc_optimizer = MPCOptimizer(
        model_info=ModelicaModelInfo(Path("/home/developer/modelica/BatteryWithFullCycle.mo"),
                                     "BatteryMPC.BatteryWithFullCycle"),
#     fmu_path="/home/developer/ipynotebooks/BatteryMPC.BatteryWithFullCycle.fmu",
        initial_parameters={
            'theveninBasedBattery.coulombSocCounter.SOC_init.k': 0.04
        },
        state_variables=Battery_state_vars,
        input_vec=['I_req'],
        output_vec=['SoC', 'SoH'],
        points_per_sec=.1
    )

    start = 360
    end = 60*60

    control_df = pd.DataFrame({"Time": range(start, end)})
    control_df['I_req'] = 0
    control_df.set_index('Time', inplace=True)

    sim_res = mpc_optimizer.simulate(start, end, control_df)

    soc, soh = estimate_state(sim_res)
    print(f"SoC: {soc}")
