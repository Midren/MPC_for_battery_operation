docker-build:
	docker build -t jmodelica:base -f Dockerfile .

docker-build-dev:
	docker build -t jmodelica:dev -f Dockerfile-dev .

vim:
	docker run -v `pwd`/python_libs/:/home/developer/python_libs \
		       -v `pwd`/ModelicaSrc:/home/developer/modelica \
			   -v `pwd`/notebooks:/home/developer/ipynotebooks \
			   -p 127.0.0.1:8888:8888 -p 127.0.0.1:6001:22 --rm -it jmodelica:dev zsh

jupyter:
	docker run -v `pwd`/python_libs/:/home/developer/python_libs \
		       -v `pwd`/ModelicaSrc:/home/developer/modelica \
			   -v `pwd`/notebooks:/home/developer/ipynotebooks \
			   -p 127.0.0.1:8888:8888 -p 127.0.0.1:6001:22 --rm -it jmodelica:base \
			   /bin/bash -c 'pip3 install -e /home/developer/python_libs/ && jupyter notebook --no-browser --ip=0.0.0.0 --port=8888 --notebook-dir=/home/developer/ipynotebooks --matplotlib=inline'

