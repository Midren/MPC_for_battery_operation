#!/bin/bash
docker run -v `pwd`/ModelicaSrc:/home/developer/modelica -v `pwd`/notebooks:/home/developer/ipynotebooks -p 127.0.0.1:8888:8888 --rm -it jmodelica jupyter-notebook --no-browser --ip=0.0.0.0 --port=8888 --notebook-dir=/home/developer/ipynotebooks --matplotlib=inline --notebook-dir=/home/developer/ipynotebooks
