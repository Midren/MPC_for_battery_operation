from functools import partial
import itertools
from pathlib import Path
import math

import numpy as np
import pandas as pd
import datetime
import matplotlib.pyplot as plt
import neptune.new as neptune

from battery_optimizations.date_splitting import get_day_load, get_interval_load
from battery_optimizations.reference_signal import calculate_ref_load

from mpc_optimization.battery_model_constants import Battery_state_vars, State_vars_aliases_dict, C_rate_per_second
from mpc_optimization.fmu_source import FmuSource, ModelicaModelInfo
from mpc_optimization.mpc_optimizer import MPCOptimizer, ModelVariables, StateConstraint

def get_load_df() -> pd.DataFrame:
    df = pd.read_csv('~/ipynotebooks/data/sweden_load_2005_2017.csv', parse_dates=['cet_cest_timestamp'])
    df['cet_cest_timestamp'] = df['cet_cest_timestamp'].apply(lambda x: x.replace(tzinfo=None))
    df = df.rename({'cet_cest_timestamp': 'time', 'SE_load_actual_tso': 'load'}, axis=1)
    df = df.set_index('time')
    df = df.loc[~df.index.duplicated(keep='first')]
    return df

def show_week_interval(df):
    load = get_interval_load(df, datetime.date(2005, 4, 20), datetime.date(2005, 4, 27))
    plt.plot(load)
    base = datetime.date(2005, 4, 20)
    for x in [base + datetime.timedelta(days=x) for x in range(7)]:
        plt.axvline(x, color='grey')
    plt.show()

def get_power_load(load_df, time: int):
    # start_date = get_day_load(load_df, datetime.date(2017, 11, 6)).iloc[0].name
    # start_date = pd.to_datetime('2017-11-06 00:00:00')
    start_date = load_df.iloc[0].name
    time = start_date + pd.to_timedelta(time, 's')
    return load_df.iloc[load_df.index.get_loc(time, method='pad')].values[0]

def get_per_mw_price(load_df, time: int):
    pload = get_power_load(load_df, time)
    return 5 + 0.5 * pload + 0.05*pload**2

def get_energy_mwh_required(load_df, time: int):
    pload = get_power_load(load_df, time)
    return pload*step/1000/3600

num_of_batteries = 40

nominal_v = 3.3
capacity_wh = 400*num_of_batteries
capacity_ah = capacity_wh/nominal_v

max_charging_wh = 100*num_of_batteries
max_charging_ah = max_charging_wh/nominal_v

c_rate_num = max_charging_ah/capacity_ah

def get_ref_price(expected_load, step, time):
    pload = expected_load[math.floor(time//3600)]
    return pload*step/1000/3600

if __name__ == "__main__":
    run = neptune.init(project='midren/mpc-peak-shaving',
                       api_token='eyJhcGlfYWRkcmVzcyI6Imh0dHBzOi8vYXBwLm5lcHR1bmUuYWkiLCJhcGlfdXJsIjoiaHR0cHM6Ly9hcHAubmVwdHVuZS5haSIsImFwaV9rZXkiOiIwNTZhNzExNS01MmY2LTRjZjctYjQyMS03Y2QxYTM4ZGQ5YWYifQ==',
                       source_files=[]
                       )

    Battery_state_vars = {
            "theveninBasedBattery.coulombSocCounter.integrator.y": "start_Q_cur",
            "theveninBasedBattery.C_tl.v": "start_U_tl",
            "theveninBasedBattery.C_ts.v": "start_U_ts",
            "theveninBasedBattery.capacityFadingCalc.SoC_avg.y": "start_SoC_avg",
            "theveninBasedBattery.capacityFadingCalc.SoC_dev.y": "start_SoC_dev",
            "theveninBasedBattery.capacityFadingCalc.SoC_avg.t_0": "start_t_of_last_cycle",
            "theveninBasedBattery.capacityFadingCalc.capacityFade": "start_capacity_fade",
            "theveninBasedBattery.capacityFadingCalc.Ah_throughput.y": "start_ah_througput"
            }

    mpc_optimizer = MPCOptimizer(
        model_info=ModelicaModelInfo(Path("/home/developer/modelica/BatteryWithFullCycle.mo"),
                                     "BatteryMPC.BatteryWithFullCycle"),
        # fmu_path="/home/developer/ipynotebooks/BatteryMPC.BatteryWithFullCycle.fmu",
        initial_parameters={
            'theveninBasedBattery.coulombSocCounter.SOC_init.k': 0.0
        },
        state_variables=Battery_state_vars,
        input_vec=['I_req'],
        output_vec=['SoC', 'SoH'],
        points_per_sec=.1
    )

    start_time = 0
# final_time = 60*60
    step=360
# final_time = (2*60+47)*60
    final_time = 24*60*60
    control_df = pd.DataFrame({"Time": range(start_time, final_time)})
    control_df['I_req'] = 0
    control_df.set_index('Time', inplace=True)

    df = get_load_df()
    load_df = get_day_load(df, datetime.date(2017, 11, 6))
    expected_load = calculate_ref_load(load_df.values.flatten(), battery_capacity=capacity_ah)

    def cost_func(step_num: int, states: pd.DataFrame, input: pd.DataFrame, output: pd.DataFrame, run: neptune.Run) -> float:
        time = step_num*step
        # controlled_timepoints = [time + step*i for i in range(0, step//10*3)]
        controlled_idx = states['time'].sub(time+3*step).abs().idxmin()

        controlled_timepoints = states['time'][:controlled_idx]
        get_energy = partial(get_energy_mwh_required, load_df)
        controlled_energy = np.array(list(map(get_energy, controlled_timepoints)))/100
        controlled_discharged_energy = input[:controlled_idx]['I_req'].values*capacity_wh*step/3600/100
        price = (controlled_energy - controlled_discharged_energy)

        controlled_price_ref = np.array(list(map(partial(get_ref_price, expected_load, step), controlled_timepoints)))/100
        controlled_horizon_cost = np.linalg.norm(price - controlled_price_ref)

        run['cost/controlled_price'].log(np.mean(price))
        run['cost/controlled_ref'].log(np.mean(controlled_price_ref))

        predicted_timepoints = states['time'][controlled_idx:]
        predicted_energy = np.array(list(map(get_energy, predicted_timepoints)))/100
        predicted_battery_capacity = output[controlled_idx:]['SoC'].values*capacity_wh/100
        predicted_discharged = predicted_battery_capacity/len(predicted_energy)/100
        predicted_price = (predicted_energy - predicted_discharged)

        predicted_ref = np.array(list(map(partial(get_ref_price, expected_load, step), predicted_timepoints)))/100
        predicted_horizon_cost = np.linalg.norm(predicted_price - predicted_ref)

        run['cost/predicted_energy'].log(np.mean(predicted_energy))
        run['cost/predicted_discharged'].log(np.mean(predicted_discharged))

        run['cost/predicted_price'].log(np.mean(predicted_price))
        run['cost/predicted_ref'].log(np.mean(predicted_ref))

        run['cost/step_num'].log(step_num)

        # controlled_output = output.iloc[:controlled_idx]
        # predicted_output = output.iloc[controlled_idx:]

        # controlled_cost = np.linalg.norm(controlled_output['SoC'].values - np.ones(len(controlled_output)))
        # predicted_cost = np.linalg.norm(predicted_output['SoC'].values - np.ones(len(predicted_output)))

        alpha = 0.5
        return controlled_horizon_cost + alpha*predicted_horizon_cost


    def visualize_output(step_num: int, states: pd.DataFrame, run: neptune.Run):
        timepoints = states['time']

        required = np.array(list(map(lambda time: get_power_load(load_df, time)/1000, timepoints.values)))
        discharged = np.maximum(states['I_req'].values*capacity_ah*nominal_v, 0)

        for _, state in itertools.islice(states.iterrows(), len(states) - 1):
            run['time'].log(state['time'])
            run['state/SoC'].log(state['SoC'])
            run['state/SoH'].log(state['SoH'])
            run['input/I_req'].log(state['I_req'])

        for from_net, from_bat in itertools.islice(zip(required-discharged, discharged), len(timepoints)-1):
            run['output/from_network'].log(from_net)
            run['output/from_battery'].log(from_bat)

        for _, state in itertools.islice(states.iterrows(), len(states) - 1):
            for state_var in state.keys():
                if state_var == 'time':
                    continue
                run[f'state_var/{state_var}'].log(state[state_var])



    new_control_df = mpc_optimizer.optimize(
        start=start_time, end=final_time, step=step,
        control_horizon=3,
        simulate_horizon=9,
        initial_guess=control_df,
        objective_func=partial(cost_func, run=run),
        bounds={'I_req': (-c_rate_num/3600, c_rate_num/3600)},
        iteration_callbacks=[partial(visualize_output, run=run)],
        constraints=[StateConstraint('SoC', 0.02, 0.98)],
    )
