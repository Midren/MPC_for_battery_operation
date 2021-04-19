from pymodelica import compile_fmu

fmu_path = compile_fmu('BatteryMPC.BatteryWithFullCycle',
                       '/home/developer/modelica/BatteryWithFullCycle.mo',
                       target='cs',
                       version="2.0",
                       compiler_log_level='warning',  # 'info', 'warning',
                       compiler_options={"generate_html_diagnostics": True,
                                          "nle_solver_tol_factor": 1e-2})  # 1e-2 is the default
