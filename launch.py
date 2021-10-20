#!/usr/bin/env python3
import subprocess
import argparse
from pathlib import Path

parser = argparse.ArgumentParser(description="Launcher for project")

group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("--vim", action='store_true')
group.add_argument("--jupyter", action='store_true')

args = vars(parser.parse_args())

if args['vim']:
    docker = "jmodelica:dev"
    startup_cmd = ["zsh"]
elif args["jupyter"]:
    docker = "jmodelica:base"
    startup_cmd = [
        "/bin/bash", "-c", "pip3 install -e /home/developer/python_libs/ && "
        "jupyter notebook --no-browser --ip=0.0.0.0 --port=8888 "
        "--notebook-dir=/home/developer/ipynotebooks --matplotlib=inline"
    ]
else:
    raise RuntimeError("Shouldn't happen")

pwd = Path(__file__).parent.resolve()
subprocess.run([
    "docker", "run", "-v", f"{pwd}/python_libs:/home/developer/python_libs", "-v",
    f"{pwd}/ModelicaSrc:/home/developer/modelica", "-v", f"{pwd}/notebooks:/home/developer/ipynotebooks", "-p",
    "127.0.0.1:8888:8888", "-p", "127.0.0.1:6001:22", "--rm", "-it", docker, *startup_cmd
], check=True)
