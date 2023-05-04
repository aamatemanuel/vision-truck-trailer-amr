from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['angle_encoder_ros'],
    package_dir={'': 'src'}
)

setup(**d)
