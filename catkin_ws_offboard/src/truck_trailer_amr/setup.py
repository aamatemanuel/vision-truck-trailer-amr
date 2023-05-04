from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['truck_trailer_amr'],
    package_dir={'': 'src'}
)

setup(**d)
