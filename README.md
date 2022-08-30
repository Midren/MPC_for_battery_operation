# Model Predictive Control (MPC) for battery operation

This repository contains code for running Li-Ion battery simulation with ageing effects, and its control for optimal charging and load peak shaving.
Battery model is implemented in Modelica, thus it achieves high perfomance. 
For easier prototyping, battery model can be exported into Functional Mockup Unit (FMU) and used inside any Functional Mockup Inteface (FMI)-compliant software. In our case, it is
used in Python for battery operation using MPC via PyFMI.

# Project structure
- **ModelicaSrc/BatteryWithFullCycle.mo** - contains implementation of Thevenin-based battery model with ageing effects ([paper](10.1109/MELECON53508.2022.9842961))  
- **python_libs/mpc_optimization/** - contains implementation of Model Predictive Control, where model is used as FMU ([UML architecture](https://github.com/Midren/MPC_for_battery_operation/blob/master/docs/mpc_optimization_diagram.png))  
- **python_libs/battery_optmizations/** - contains code for optimal battery charging and load peak shaving (WIP)  
- **notebooks/** - jupyter notebooks with experiments

# Usage

All the work is done under the docker (JModelica is supported only under Linux), thus the simplest way is just to use it.
Dockerfile contains necessary environment for running the code, and Dockerfile-dev contains the environment with my neovim/zsh configuration.  
Makefile has commands for build and run the docker:
```sh
make docker-build # or make docker-build-dev
make jupyter # or make vim 
```

# Citing

If you use this repo, please cite:

```
@article{milishchuk_bogodorova_2022, 
    title={Thevenin-based battery model with ageing effects in Modelica}, 
    DOI={10.1109/melecon53508.2022.9842961}, 
    journal={2022 IEEE 21st Mediterranean Electrotechnical Conference (MELECON)}, 
    author={Milishchuk, Roman and Bogodorova, Tetiana}, 
    year={2022}
} 
```
