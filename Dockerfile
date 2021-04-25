FROM michaelwetter/ubuntu-1804_jmodelica_trunk:latest

USER root

ENV JMODELICA_HOME /usr/local/JModelica
ENV IPOPT_HOME /usr/local/Ipopt-3.12.4 
#ENV CPPAD_HOME /home/docker/installed/JModelica/ThirdParty/CppAD/
ENV SUNDIALS_HOME /usr/local/JModelica/ThirdParty/Sundials
#ENV PYTHONPATH /usr/local/JModelica/Python/:/usr/local/JModelica/Python/pymodelica/:
ENV LD_LIBRARY_PATH /usr/local/Ipopt-3.12.4/lib/:/usr/local/JModelica/ThirdParty/Sundials/lib:/usr/local/JModelica/ThirdParty/CasADi/lib:/usr/local/JModelica/Python/pyfmi
ENV SEPARATE_PROCESS_JVM /usr/lib/jvm/java-8-openjdk-amd64/
ENV MODELICAPATH /usr/local/JModelica/ThirdParty/MSL:/home/developer/modelica
ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64
ENV JCC_JDK /usr/lib/jvm/java-8-openjdk-amd64
ENV PATH="/home/developer/miniconda3/bin:${PATH}"

EXPOSE 8888
EXPOSE 22

RUN apt-get -y update && apt-get install -y ipython vim libgeos-dev git openssh-server sudo iproute2 python3-pip python3
RUN apt-get install -y wget && rm -rf /var/lib/apt/lists/*

RUN usermod -aG sudo developer
RUN echo "developer ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/username
RUN echo 'developer:password' | chpasswd

USER developer
WORKDIR /home/developer/

RUN wget \
    https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
    && mkdir /home/developer/.conda \
    && bash Miniconda3-latest-Linux-x86_64.sh -b \
    && rm -f Miniconda3-latest-Linux-x86_64.sh 
RUN conda update conda -y \
    && conda config --add channels conda-forge \
    && conda install pyfmi -y
RUN conda install matplotlib>=2.0.2 numpy>=1.16.6 pandas>=0.20.3 python-dateutil>=2.6.1 scikit-learn>=0.18.2 sphinx>=1.6.3 numpydoc>=0.7.0 pyDOE>=0.3.8 netCDF4==1.4.2 cftime==1.0.4.2 siphon==0.8.0 -y
RUN conda install jupyter jupyter_contrib_nbextensions jupyterthemes -y

RUN mkdir /home/developer/ipynotebooks && mkdir /home/developer/modelica
#RUN mkdir /home/developer/mpcpy_libs/
#WORKDIR /home/developer/mpcpy_libs
#RUN git clone https://github.com/lbl-srg/EstimationPy
# connect via docker sharing
#RUN git clone https://github.com/Midren/MPCPy


RUN mkdir -p /home/developer/.jupyter && jupyter notebook --generate-config
# Add password, so no need to look for token
RUN python3 -c 'import json; from notebook.auth import passwd; open("/home/developer/.jupyter/jupyter_notebook_config.json", "w").write(json.dumps({"NotebookApp":{"password": passwd("password")}}));'

RUN mkdir -p /home/developer/.local/share/jupyter/nbextensions
WORKDIR /home/developer/.local/share/jupyter/nbextensions
RUN git clone https://github.com/lambdalisue/jupyter-vim-binding vim_binding
RUN jupyter nbextension enable vim_binding/vim_binding
RUN jupyter contrib nbextension install --user
RUN jupyter nbextension enable code_prettify/autopep8
RUN jupyter nbextension enable collapsible_headings/main
RUN jt -tf source -nf ptsans -nfs 10 -ofs 10 -fs 10 -tfs 10 #-t gruvboxd --vim

RUN conda install altair vega autopep8
RUN conda init bash 
#&& conda activate

ENV USER developer
ENV DISPLAY :0.0
##ENTRYPOINT sudo -S service ssh restart && bash
