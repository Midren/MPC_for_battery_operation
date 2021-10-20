import numpy as np
from functools import partial

from scipy.optimize import minimize
from scipy.optimize import NonlinearConstraint

# bounds
def get_bounds(load):
    bounds = ((load.min(), load.max()), (load.min(), load.max()))
    return bounds

# constraints
def power_amount(load):
    return np.trapz(load)

def battery_used(x, load):
    min_val, max_val = x

    total_power = power_amount(load)

    discharging_load = np.minimum(load, max_val)
    charging_load = np.maximum(load, min_val)

    return (power_amount(charging_load) - total_power, total_power - power_amount(discharging_load))

def charge_discharge_power(x, load):
    min_val, max_val = x

    discharging_load = np.minimum(load, max_val)
    charging_load = np.maximum(load, min_val)

    return power_amount(charging_load) + power_amount(discharging_load)

def get_constrainsts(load, battery_capacity: float):
    total_battery_usage = NonlinearConstraint(fun=partial(
        battery_used, load=load), lb=(0, 0), ub=(battery_capacity, battery_capacity))

    charge_discharge_equivalency = NonlinearConstraint(fun=partial(
        charge_discharge_power, load=load), lb=power_amount(load), ub=2*power_amount(load)
    )

    constraints = [total_battery_usage, charge_discharge_equivalency]
    return constraints

# optimization
def peak_diff(x, load):
    return -battery_used(x, load)[0]
#     return (max_val - min_val)**2


def calculate_ref_load(load, battery_capacity: float):
    x0 = np.ones(2)*np.mean(load)
    res = minimize(fun=partial(peak_diff, load=load), x0=x0, bounds=get_bounds(load), constraints=get_constrainsts(load, battery_capacity))
    charging_bound, discharging_bound = res.x
    discharging_load = np.minimum(load, discharging_bound)
    ref_load = np.maximum(discharging_load, charging_bound)
    return ref_load
