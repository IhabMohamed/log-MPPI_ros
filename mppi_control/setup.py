from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args_mppi = generate_distutils_setup(
    packages=['mppi_control'],
    package_dir={'':'scripts'},
    scripts=[])

setup(**setup_args_mppi)