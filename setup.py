# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['nodes/mock_camera', 'nodes/video_recorder'],
    packages=['robot_video_recorder'],       # This is the name of the package
    package_dir={'': 'src'})  # This says that the package is in the "python" directory

setup(**setup_args)

