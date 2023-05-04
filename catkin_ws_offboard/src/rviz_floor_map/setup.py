from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['rviz_floor_map'],
    package_dir={'': 'src'}
)

setup(**d)
