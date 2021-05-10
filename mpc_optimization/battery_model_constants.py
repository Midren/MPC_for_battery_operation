import typing as t

Battery_state_vars: t.Final[str] = [
    'time', 'theveninBasedBattery.U_oc.v', 'theveninBasedBattery.R_s.v',
    'theveninBasedBattery.R_ts.v', 'theveninBasedBattery.R_tl.v',
    'theveninBasedBattery.coulombSocCounter.integrator.y',
    'theveninBasedBattery.capacityFadingCalc.SoC_avg.mu',
    'theveninBasedBattery.capacityFadingCalc.SoC_avg.t_0',
    'theveninBasedBattery.capacityFadingCalc.SoC_dev.variance.mu',
    'theveninBasedBattery.capacityFadingCalc.SoC_dev.variance.var',
    'theveninBasedBattery.capacityFadingCalc.SoC_dev.variance.t_0',
    'theveninBasedBattery.capacityFadingCalc.Ah_throughput.y',
    'theveninBasedBattery.capacityFadingCalc.capacityFade',
    'theveninBasedBattery.capacityFadingCalc.stepCapacityFade', 'theveninBasedBattery.SoH_last',
    'theveninBasedBattery.capacityFadingCalc.isNotCharging.y', 'theveninBasedBattery.v'
]

State_vars_aliases_dict: t.Final[t.Dict[str, str]] = {
    'time': 'time',
    'U_ocv': 'theveninBasedBattery.U_oc.v',
    'R_s': 'theveninBasedBattery.R_s.v',
    'R_ts': 'theveninBasedBattery.R_ts.v',
    'R_tl': 'theveninBasedBattery.R_tl.v',
    'I_int': 'theveninBasedBattery.coulombSocCounter.integrator.y',
    'SoC_avg': 'theveninBasedBattery.capacityFadingCalc.SoC_avg.y',
    'SoC_avg_mu': 'theveninBasedBattery.capacityFadingCalc.SoC_avg.mu',
    'SoC_avg_t0': 'theveninBasedBattery.capacityFadingCalc.SoC_avg.t_0',
    'SoC_dev': 'theveninBasedBattery.capacityFadingCalc.SoC_dev_normed.y',
    'SoC_dev_mu': 'theveninBasedBattery.capacityFadingCalc.SoC_dev.variance.mu',
    'SoC_dev_var': 'theveninBasedBattery.capacityFadingCalc.SoC_dev.variance.var',
    'SoC_dev_t0': 'theveninBasedBattery.capacityFadingCalc.SoC_dev.variance.t_0',
    'capacityFade': 'theveninBasedBattery.capacityFadingCalc.capacityFade',
    'stepCapacityFade': 'theveninBasedBattery.capacityFadingCalc.stepCapacityFade',
    'Ah_throughput': 'theveninBasedBattery.capacityFadingCalc.Ah_throughput.y',
    't_last': 'theveninBasedBattery.capacityFadingCalc.cycleStartTime',
    'I_req': 'I_req',
    'SoC': 'SoC',
    'delta_SoH': 'theveninBasedBattery.SoH_diff',
    'SoH_delay': 'theveninBasedBattery.SoH_last',
    'SoH': 'SoH',
    'U': 'theveninBasedBattery.v',
    'is_not_charging': 'theveninBasedBattery.capacityFadingCalc.isNotCharging.y',
}

C_bat: t.Final[float] = 1.1
C_rate_per_second: t.Final[float] = C_bat / 3600
