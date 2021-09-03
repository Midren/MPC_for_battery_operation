import functools
from pathlib import Path

import numpy as np
import pandas as pd
import neptune.new as neptune

from mpc_optimization.battery_model_constants import Battery_state_vars, C_rate_per_second
from mpc_optimization.fmu_source import ModelicaModelInfo
from mpc_optimization.mpc_optimizer import MPCOptimizer

from utils import stop_if_charged, visualize_output

run = neptune.init(
    project='midren/mpc-battery-charging',
    api_token=
    'eyJhcGlfYWRkcmVzcyI6Imh0dHBzOi8vYXBwLm5lcHR1bmUuYWkiLCJhcGlfdXJsIjoiaHR0cHM6Ly9hcHAubmVwdHVuZS5haSIsImFwaV9rZXkiOiIwNTZhNzExNS01MmY2LTRjZjctYjQyMS03Y2QxYTM4ZGQ5YWYifQ=='
)

mpc_optimizer = MPCOptimizer(model_info=ModelicaModelInfo(Path("/home/developer/modelica/BatteryWithFullCycle.mo"),
                                                          "BatteryMPC.BatteryWithFullCycle"),
                             fmu_path="/home/developer/ipynotebooks/BatteryMPC.BatteryWithFullCycle.fmu",
                             initial_parameters={'theveninBasedBattery.coulombSocCounter.SOC_init.k': 0.0},
                             state_variables=Battery_state_vars,
                             input_vec=['I_req'],
                             output_vec=['SoC', 'SoH'],
                             points_per_sec=.1)

start_time = 0
final_time = 60 * 60
control_df = pd.DataFrame({"Time": range(start_time, final_time)})
control_df['I_req'] = 0
control_df.set_index('Time', inplace=True)

ws = np.geomspace(1e-3, 1e3, 7)
# w = ws[6]*50000 # -- for discharging
# 3-12
w = ws[6] * 30  # -- for charging
# w = 0

whole_ref_soc_2_c = np.array([
    0., 0.00218717, 0.00437434, 0.00656151, 0.00874868, 0.01093585, 0.01312302, 0.01721757, 0.02131354, 0.0254095, 0.02950547, 0.03360143, 0.03769739,
    0.0438396, 0.04998358, 0.05612756, 0.06227154, 0.06841552, 0.0745595, 0.08070351, 0.08684751, 0.09299152, 0.09913552, 0.10527953, 0.11142353,
    0.11756754, 0.12371155, 0.12985556, 0.13599956, 0.14214357, 0.14828758, 0.15443087, 0.16057415, 0.16671743, 0.17286072, 0.179004, 0.18514728,
    0.19129127, 0.19743528, 0.20357929, 0.2097233, 0.21586731, 0.22201132, 0.22815533, 0.23429934, 0.24044335, 0.24658737, 0.25273138, 0.25887539,
    0.2650194, 0.27116342, 0.27730743, 0.28345145, 0.28959546, 0.29573948, 0.30188349, 0.30802751, 0.31417152, 0.32031554, 0.32645956, 0.33260358,
    0.3387476, 0.34489162, 0.35103564, 0.35717966, 0.36332368, 0.3694677, 0.37561172, 0.38175575, 0.38789977, 0.3940438, 0.40018782, 0.40633185,
    0.4124744, 0.41861678, 0.42475916, 0.43090155, 0.43704393, 0.44318632, 0.44933019, 0.45547423, 0.46161826, 0.4677623, 0.47390634, 0.48005038,
    0.48619442, 0.49233846, 0.4984825, 0.50462655, 0.5107706, 0.51691464, 0.52305869, 0.52920275, 0.5353468, 0.54149086, 0.54763491, 0.55377897,
    0.55992238, 0.56606572, 0.57220906, 0.5783524, 0.58449574, 0.59063908, 0.59678304, 0.60292711, 0.60907119, 0.61521527, 0.62135936, 0.62750345,
    0.63364754, 0.63979163, 0.64593573, 0.65207983, 0.65822393, 0.66436803, 0.67051168, 0.67665525, 0.68279883, 0.68894241, 0.69508599, 0.70122957,
    0.70737362, 0.71351776, 0.7196619, 0.72580604, 0.73195019, 0.73809435, 0.74423851, 0.75038268, 0.75652685, 0.76267102, 0.7688152, 0.77495939,
    0.78110357, 0.78724774, 0.79339193, 0.79953612, 0.80568031, 0.81182452, 0.8179678, 0.8241109, 0.830254, 0.83639711, 0.84254023, 0.84868335,
    0.85482745, 0.86097174, 0.86711604, 0.87326034, 0.87940466, 0.88554899, 0.89169333, 0.89783767, 0.90398203, 0.9101264, 0.91627078, 0.92241517,
    0.92855958, 0.93470399, 0.94084842, 0.94699286, 0.95313732, 0.95928179, 0.96542627, 0.97157077, 0.97771528, 0.98385981, 0.99000435
])

whole_ref_soh_2_c = np.array([
    1., 1., 0.99999999, 0.99999998, 0.99999998, 0.99999996, 0.99999996, 0.99999995, 0.99999993, 0.99999992, 0.9999999, 0.99999988, 0.99999987,
    0.99999985, 0.99999982, 0.99999979, 0.99999976, 0.99999973, 0.9999997, 0.99999967, 0.99999963, 0.9999996, 0.99999956, 0.99999952, 0.99999948,
    0.99999944, 0.9999994, 0.99999935, 0.99999931, 0.99999926, 0.99999921, 0.99999916, 0.99999911, 0.99999905, 0.999999, 0.99999894, 0.99999888,
    0.99999882, 0.99999876, 0.99999869, 0.99999862, 0.99999855, 0.99999848, 0.9999984, 0.99999833, 0.99999825, 0.99999816, 0.99999808, 0.99999799,
    0.9999979, 0.99999781, 0.99999771, 0.99999761, 0.99999751, 0.99999741, 0.9999973, 0.99999719, 0.99999707, 0.99999695, 0.99999683, 0.9999967,
    0.99999657, 0.99999644, 0.9999963, 0.99999616, 0.99999602, 0.99999587, 0.99999571, 0.99999555, 0.99999539, 0.99999522, 0.99999505, 0.99999487,
    0.99999468, 0.9999945, 0.9999943, 0.9999941, 0.9999939, 0.99999368, 0.99999347, 0.99999324, 0.99999301, 0.99999278, 0.99999253, 0.99999228,
    0.99999203, 0.99999176, 0.99999149, 0.99999121, 0.99999092, 0.99999063, 0.99999032, 0.99999001, 0.99998969, 0.99998936, 0.99998903, 0.99998868,
    0.99998832, 0.99998796, 0.99998758, 0.99998719, 0.99998679, 0.99998639, 0.99998597, 0.99998554, 0.99998509, 0.99998464, 0.99998417, 0.9999837,
    0.9999832, 0.9999827, 0.99998218, 0.99998165, 0.9999811, 0.99998054, 0.99997996, 0.99997937, 0.99997877, 0.99997814, 0.99997751, 0.99997685,
    0.99997618, 0.99997549, 0.99997478, 0.99997405, 0.9999733, 0.99997254, 0.99997175, 0.99997095, 0.99997012, 0.99996927, 0.9999684, 0.9999675,
    0.99996659, 0.99996565, 0.99996468, 0.99996369, 0.99996268, 0.99996164, 0.99996057, 0.99995947, 0.99995835, 0.9999572, 0.99995602, 0.99995481,
    0.99995356, 0.99995229, 0.99995098, 0.99994964, 0.99994827, 0.99994686, 0.99994542, 0.99994394, 0.99994242, 0.99994086, 0.99993926, 0.99993763,
    0.99993595, 0.99993423, 0.99993246, 0.99993066, 0.9999288, 0.9999269, 0.99992496, 0.99992296, 0.99992092, 0.99991882, 0.99991447
])

changed = 0


def cost_func(step_num: int, state: pd.DataFrame, input: pd.DataFrame, output: pd.DataFrame, run: neptune.Run) -> float:
    w = ws[6] * 11

    ref_soc = whole_ref_soc_2_c[step_num * 2:step_num * 2 + len(state)]
    ref_soc = np.concatenate([ref_soc, whole_ref_soc_2_c[-1] * np.ones(len(state) - len(ref_soc))])

    ref_soh_diff = whole_ref_soh_2_c[step_num * 2:step_num * 2 + len(state)]

    step = whole_ref_soh_2_c[-2] - whole_ref_soh_2_c[-1]
    appended = np.arange(whole_ref_soh_2_c[-1], whole_ref_soh_2_c[-1] - step * (len(state) - len(ref_soh_diff)), -step)
    ref_soh_diff = np.concatenate([ref_soh_diff, appended])

    output = output.copy()
    output['SoH_diff'] = output['SoH'] - output['SoH'].shift(1, fill_value=1)
    soc_term = np.linalg.norm(output['SoC'] - ref_soc)
    soh_term = w * np.linalg.norm(output['SoH'] - ref_soh_diff)
    run['cost/SoC'].log(soc_term)
    run['cost/SoH'].log(soh_term)
    #     logging.info(f'{soc_term}, {soh_term}')
    return soc_term + soh_term

new_control_df = mpc_optimizer.optimize(
    start=0,
    end=3600,
    step=20,
    control_horizon=1,
    simulate_horizon=30,
    initial_guess=control_df,
    objective_func=functools.partial(cost_func, run=run),
    bounds={'I_req': (-10 * C_rate_per_second, 0)},
    iteration_callbacks=[functools.partial(visualize_output, run=run)],
    early_stopping_funcs=[stop_if_charged],
    #     constraints=[StateConstraint('SoC', 0.01, 0.99)]
)

states = mpc_optimizer.simulate(0, new_control_df.index[-1], new_control_df, save_all=True)
np.save('mpc_charging_res.npy', new_control_df)
