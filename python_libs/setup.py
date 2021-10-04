from setuptools import setup, find_packages


def read_requirements():
    with open("requirements.txt", "r") as f:
        return [x for x in f.read().splitlines() if x != ""]


setup(name='mpc-optimization',
      version='0.0.0',
      python_requires='>=3.8.0',
      description='MPC algorithm using FMU models',
      url='https://github.com/Midren/MPC_for_battery_operation',
      author='Roman Milishchuk',
      author_email='milishchuk.roman@gmail.com',
      license='Proprietary',
      packages=find_packages(),
      # install_requires=read_requirements(),
      include_package_data=True,
      zip_safe=False)
