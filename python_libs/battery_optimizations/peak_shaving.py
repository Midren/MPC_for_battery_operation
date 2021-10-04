import traceback
import logging
import typing as t

import os
import shutil
import matplotlib.pyplot as plt

from pathlib import Path

import pandas as pd
import numpy as np
import altair as alt
import seaborn as sns

logger = logging.getLogger()
logger.setLevel(logging.INFO)

alt.data_transformers.disable_max_rows()
sns.set_theme()

start_date: t.Final[str] = '2017-11-06'


def get_load_n_days(load_path: Path, n: int = 1):
    if n > 25:
        raise ValueError('max 25 days')
    df = pd.read_csv(load_path, parse_dates=['cet_cest_timestamp'])
    df['cet_cest_timestamp'] = df['cet_cest_timestamp'].apply(lambda x: x.replace(tzinfo=None))

    df_tmp = df.loc[(df['cet_cest_timestamp'] >= pd.to_datetime(start_date))
            & (df['cet_cest_timestamp'] < pd.to_datetime(f'{start_date[:3]}-{6+n:02}'))].rename(columns={
                        "cet_cest_timestamp": 'time',
                        'SE_load_actual_tso': 'load'
                    }).set_index('time')

    week_load = df_tmp.resample('10S').asfreq().interpolate()

    return week_load

def get_discharging_charging_bounds(load, charging_percentile, discharging_percentile):
    discharging_bound = np.percentile(load.load, q=discharging_percentile, interpolation='linear')
    charging_bound = np.percentile(load.load, q=charging_percentile, interpolation='linear')
    return discharging_bound, charging_bound

def get_expected_load(load, charging_percentile, discharging_percentile):
    discharging_bound, charging_bound = get_discharging_charging_bounds(load, charging_percentile, discharging_percentile)
    expected_week_load  = load.copy()
    expected_week_load.load = np.minimum(expected_week_load, discharging_bound)
    expected_week_load.load = np.maximum(expected_week_load, charging_bound)
    return expected_week_load

def visualize_load(load):
    return alt.Chart(load.reset_index()).encode(
        x='time:T',
        y='load:Q'
    ).mark_line()

def visualize_charging_discharging(load, charging_percentile, discharging_percentile):
    def red_line_chart(y):
        return alt.Chart(pd.DataFrame({'y': [y]})).encode(y='y:Q').mark_rule(color='red')

    # base_load = load.min().values[0]
    discharging_bound, charging_bound = get_discharging_charging_bounds(load, charging_percentile, discharging_percentile)

    return alt.Chart(load.reset_index()).encode(x='time:T',
                                                y='load:Q').mark_line() + \
            red_line_chart(charging_bound) + red_line_chart(discharging_bound)

sweden_load_path = Path('../ipynotebooks/data/sweden_load_2005_2017.csv')
day_load = get_load_n_days(sweden_load_path, 1)
week_load = get_load_n_days(sweden_load_path, 7)

num_of_batteries = 40

nominal_v = 3.3
capacity_wh = 400*num_of_batteries
capacity_ah = capacity_wh/nominal_v

max_charging_wh = 100*num_of_batteries
max_charging_ah = max_charging_wh/nominal_v

c_rate_num = max_charging_ah/capacity_ah

# step = 10
# load_df = df_tmp.resample(f'{step}S').asfreq().interpolate()
load_df = week_load

def get_power_load(load, time: int):
    start_time = pd.to_datetime(f'{start_date} 00:00:00')
    time = start_time + pd.to_timedelta(time, 's')
    return load_df.iloc[load_df.index.get_loc(time, method='nearest')].values[0]

def get_per_mw_price(load, time: int):
    pload = get_power_load(load, time)
    return 5 + 0.5 * pload + 0.05*pload**2

def get_energy_mwh_required(load, time: int, step: int):
    pload = get_power_load(load, time)
    return pload*step/1000/3600


# visualize_charging_discharging(week_load, 25, 75)
