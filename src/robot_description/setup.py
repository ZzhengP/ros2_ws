from setuptools import find_packages, setup
import os 
from glob import glob
package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
  (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
  (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
  (os.path.join('share', package_name, 'meshes','visual'), glob('meshes/visual/*')),
    (os.path.join('share', package_name, 'meshes','collision'), glob('meshes/collision/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
