import datetime

import pandas as pd
import numpy as np


def _local_extremum(arr, min_: bool):
    mask = np.empty_like(arr, dtype=bool)
    mask[0] = True
    mask[1:] = arr[:-1] != arr[1:]
    arr_filtered = arr[mask]
    idx_to_orig = mask.nonzero()[0]

    if not min_:
        idx_maximum = np.logical_and(arr_filtered[:-2] < arr_filtered[1:-1], arr_filtered[1:-1] >= arr_filtered[2:])
    else:
        idx_maximum = np.logical_and(arr_filtered[:-2] > arr_filtered[1:-1], arr_filtered[1:-1] <= arr_filtered[2:])

    idx_maximum = idx_to_orig[1:-1][idx_maximum]

    return idx_maximum


def local_maximum(arr):
    return _local_extremum(arr, min_=False)


def local_minimum(arr):
    return _local_extremum(arr, min_=True)


def _per_day_extremum(arr, min_: bool):
    if min_:
        maxs = local_minimum(arr)
    else:
        maxs = local_maximum(arr)
    max_st = 0
    per_day_maxs = []
    for i in range(24, maxs[-1], 24):
        curr_max_st = max_st
        day_maxs = []
        while maxs[curr_max_st] < i:
            day_maxs.append(maxs[curr_max_st])
            curr_max_st += 1

        max_st = curr_max_st

        if len(day_maxs) == 1:
            per_day_maxs.append(day_maxs[0])
            continue
        elif len(day_maxs) == 0:
            continue

        if min_:
            per_day_maxs.append(day_maxs[np.argmin(arr[day_maxs])])
        else:
            per_day_maxs.append(day_maxs[np.argmax(arr[day_maxs])])

    return np.array(per_day_maxs)


def _per_day_minimum(arr):
    return _per_day_extremum(arr, min_=True)


def _per_day_maximum(arr):
    return _per_day_extremum(arr, min_=False)


def _get_start_of_days(df):
    maxs = _per_day_maximum(df.values)
    mins = _per_day_minimum(df.values)
    start_of_day = []
    offset_from_max = []

    mins_st = 0
    maxs_st = 0
    while maxs_st < len(maxs) and mins_st < len(mins):
        if mins[mins_st] < maxs[maxs_st]:
            mins_st += 1
            continue

        start_of_day.append(maxs[maxs_st] + (mins[mins_st] - maxs[maxs_st]) // 2)
        offset_from_max.append((mins[mins_st] - maxs[maxs_st]) // 5 * 2)
        maxs_st += 1

        while maxs_st + 1 < len(maxs) and maxs[maxs_st + 1] < mins[mins_st]:
            maxs_st += 1

    start_of_day = np.array(start_of_day)
    offset_from_max = np.array(offset_from_max)
    return start_of_day


def _get_day_diff(df: pd.DataFrame, day: datetime.date):
    first_day = df.iloc[0].name.to_pydatetime()
    return (datetime.datetime(day.year, day.month, day.day, first_day.hour) - first_day).days


def get_days_load(df: pd.DataFrame, day: datetime.date, n_days=1):
    start_of_day = _get_start_of_days(df)
    day = datetime.datetime.combine(day, datetime.datetime.min.time())

    st_idx = _get_day_diff(df, day)
    while (df.iloc[start_of_day[st_idx-1]].name.to_pydatetime() - day).days > 0:
        st_idx -= 1
    while (day - df.iloc[start_of_day[st_idx+1]].name.to_pydatetime()).days > 0:
        st_idx += 1
    st = start_of_day[st_idx]

    return df.iloc[st:st + 24*n_days]

def get_day_idx(df: pd.DataFrame, day: datetime.date):
    start_of_day = _get_start_of_days(df)
    day = datetime.datetime.combine(day, datetime.datetime.min.time())

    st_idx = _get_day_diff(df, day)
    while (df.iloc[start_of_day[st_idx-1]].name.to_pydatetime() - day).days > 0:
        st_idx -= 1
    while (day - df.iloc[start_of_day[st_idx+1]].name.to_pydatetime()).days > 0:
        st_idx += 1
    st = start_of_day[st_idx]

    return st


def get_interval_load(df: pd.DataFrame, st_day: datetime.date, en_day: datetime.date):
    start_of_day = _get_start_of_days(df)
    st = start_of_day[_get_day_diff(df, st_day)]
    en = start_of_day[_get_day_diff(df, en_day)]
    return df.iloc[st:en + 24]
