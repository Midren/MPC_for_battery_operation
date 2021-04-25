#!/bin/bash
docker run -v `pwd`/mpc_optimization:/home/developer/python_libs/mpc_optimization -v `pwd`/ModelicaSrc:/home/developer/modelica -v `pwd`/notebooks:/home/developer/ipynotebooks -p 127.0.0.1:8888:8888 -it jmodelica:latest jupyter notebook --no-browser --ip=0.0.0.0 --port=8888 --notebook-dir=/home/developer/ipynotebooks --matplotlib=inline
