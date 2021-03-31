FROM michaelwetter/ubuntu-1804_jmodelica_trunk:latest

USER root

ENV JMODELICA_HOME /usr/local/JModelica
ENV IPOPT_HOME /usr/local/Ipopt-3.12.4 
#ENV CPPAD_HOME /home/docker/installed/JModelica/ThirdParty/CppAD/
ENV SUNDIALS_HOME /usr/local/JModelica/ThirdParty/Sundials
ENV PYTHONPATH /usr/local/JModelica/Python/:
ENV LD_LIBRARY_PATH /usr/local/Ipopt-3.12.4/lib/:\
/usr/local/JModelica/ThirdParty/Sundials/lib:\
/usr/local/JModelica/ThirdParty/CasADi/lib
ENV SEPARATE_PROCESS_JVM /usr/lib/jvm/java-8-openjdk-amd64/
ENV MODELICAPATH /usr/local/JModelica/ThirdParty/MSL:/home/developer/modelica

EXPOSE 8888

RUN apt-get -y update && apt-get install -y ipython vim
RUN pip install jupyter

RUN mkdir -p /home/developer/.jupyter && jupyter notebook --generate-config
RUN python -c 'import json; from notebook.auth import passwd; open("/home/developer/.jupyter/jupyter_notebook_config.json", "w").write(json.dumps({"NotebookApp":{"password": passwd("password")}}));'

USER developer
RUN mkdir /home/developer/ipynotebooks && mkdir /home/developer/modelica
ENV USER developer
ENV DISPLAY :0.0
WORKDIR /home/developer/
