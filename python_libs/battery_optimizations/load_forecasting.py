import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
from tqdm import tqdm

from statsmodels.tsa import seasonal
from statsmodels.tsa.arima.model import ARIMA

from sklearn.metrics import mean_absolute_percentage_error, mean_squared_error, mean_absolute_error

def _decompose(ts, period, show_figs=False):
    decomposition = seasonal.seasonal_decompose(ts, period=period)

    trend = decomposition.trend
    trend.dropna(inplace=True)
    seas = decomposition.seasonal
    residual = decomposition.resid
    residual.dropna(inplace=True);

    if show_figs:
        # Original
        plt.subplot(411)
        plt.plot(ts, label='Original')
        plt.legend(loc='upper left')

        # Seasonality
        plt.subplot(412)
        plt.plot(seas, label='Seasonality')
        plt.legend(loc='upper left')

        # Trend
        plt.subplot(413)
        plt.plot(trend, label='Trend')
        plt.legend(loc='upper left')

        # Resudials
        plt.subplot(414)
        plt.plot(residual, label='Residuals')
        plt.legend(loc='upper left')

        plt.tight_layout()
        plt.show()
    return seas, trend, residual


def predict_arima(model_params, ts, train_indeces, val_indeces, diff_order=0):
    train_val = ts.iloc[[train_indeces[0] - i for i in range(1, diff_order+1)][::-1] + list(train_indeces)].values
    offsets = []
    for i in range(diff_order):
        offsets.append(train_val[0])
        train_val = np.diff(train_val, 1)

    predicted = np.array([])
    for val_index in tqdm(val_indeces):
        model = ARIMA(np.append(train_val, predicted), **model_params)
        model_fit = model.fit(method_kwargs={"warn_convergence": False})
        res = model_fit.forecast(1)
        train_indeces = np.append(train_indeces, val_index)
        predicted = np.append(predicted, res[0])

    res = np.r_[train_val, predicted]
    for i in range(1, diff_order+1):
        res = np.r_[offsets[-i], res].cumsum()
    return res[diff_order+len(train_val):]


def plot_predicted(ts, predicted, train_indeces, val_indeces, ax, title):
    ax.plot(ts[np.append(train_indeces, val_indeces)].values)
    ax.plot(np.arange(len(train_indeces), len(train_indeces) + len(val_indeces), 1), predicted)
    ax.set_ylabel('Load')
    ax.set_xlabel('Hours')
    ax.set_title(title)


def decompose_load(load_df: pd.DataFrame, show_figs=False):
    year_seasonal, year_trend, year_res = _decompose(load_df, 24*365, show_figs=show_figs)
    day_seasonal, day_trend, day_res = _decompose(year_res, 24, show_figs=show_figs)

    # remove part of data for easier additivity
    year_seasonal = year_seasonal[366*24//2:-366*24//2]
    year_trend = year_trend[24//2:-24//2]
    day_seasonal = day_seasonal[24//2:-24//2]

    return year_seasonal, year_trend, year_res, day_seasonal, day_trend, day_res


def predict_day_load(load_df: pd.DataFrame, day_idx: int, show_figs=False):
    year_seasonal, year_trend, year_res, day_seasonal, day_trend, day_res = decompose_load(load_df, show_figs=False)

    day_idx -= 366*24//2-1

    # train size 2 weeks, prediction for 1 day
    train_indeces = day_idx + np.arange(0, 2*24*7, 1)
    val_indeces = train_indeces[-1] + np.arange(0, 24, 1)

    # TODO: add prediction by LSTM (see: Forecasting.ipynb)
    predicted_day_r = predict_arima({'order': (9, 1, 24)}, day_res, train_indeces, val_indeces)
    predicted_day_t = predict_arima({'order': (3, 1, 2)}, day_trend, train_indeces, val_indeces, diff_order=1)
    predicted_year_r = predict_arima({'order': (3, 1, 2)}, year_res, train_indeces, val_indeces)
    predicted_year_t = predict_arima({'order': (3, 1, 2)}, year_trend, train_indeces, val_indeces, diff_order=2)

    total = year_seasonal + year_trend + day_seasonal + day_trend + day_res
    predicted_t = year_seasonal.iloc[val_indeces] + predicted_year_t + day_seasonal.iloc[val_indeces] + predicted_day_t + predicted_day_r

    if show_figs:
        fig = plt.figure()
        ax_1 = fig.add_subplot(2, 1, 1)
        ax_2 = fig.add_subplot(2, 4, 5)
        ax_3 = fig.add_subplot(2, 4, 6)
        ax_4 = fig.add_subplot(2, 4, 7)
        ax_5 = fig.add_subplot(2, 4, 8)

        def plot_prediction(ts, predicted_ts, train_idx, val_idx, ax, title):
            rmse = mean_squared_error(ts.iloc[val_idx], predicted_ts, squared=False)
            mae = mean_absolute_error(ts.iloc[val_idx], predicted_ts)
            mape = mean_absolute_percentage_error(ts.iloc[val_idx], predicted_ts)
            plot_predicted(ts, predicted_ts, train_idx, val_idx, ax, f'{title}\nMAPE: {mape:.2f}\nMAE: {mae:.2f}\nRMSE: {rmse:.2f}')

        plot_prediction(total, predicted_t, train_indeces, val_indeces, ax_1, 'Total load')
        plot_prediction(day_res, predicted_day_r, train_indeces, val_indeces, ax_2, 'Day residuals')
        plot_prediction(day_trend, predicted_day_t, train_indeces, val_indeces, ax_3, 'Day trend')
        plot_prediction(year_res, predicted_year_r, train_indeces, val_indeces, ax_4, 'Year residuals')
        plot_prediction(year_trend, predicted_year_t, train_indeces, val_indeces, ax_5, 'Year trend')

        plt.show()
    return predicted_t

if __name__ == "__main__":
    df = pd.read_csv('/Users/roman.milishchuk/bachelor/notebooks/data/sweden_load_2005_2017.csv', parse_dates=['cet_cest_timestamp'])

    df['cet_cest_timestamp'] = df['cet_cest_timestamp'].apply(lambda x: x.replace(tzinfo=None))
    df = df.rename({'cet_cest_timestamp': 'time', 'SE_load_actual_tso': 'load'}, axis=1)
    df = df.set_index('time')
    # df = df.loc[~df.index.duplicated(keep='first')]

    predict_day_load(df, 103784, show_figs=True)
