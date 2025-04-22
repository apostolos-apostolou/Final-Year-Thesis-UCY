from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simulation_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name + '/models', glob('models/*')), #include meshes
        ('share/' + package_name + '/description', glob('description/*')), #include description
        ('share/' + package_name + '/worlds', glob('worlds/*')), #include world
        ('share/' + package_name + '/params', glob('params/*')), #include params
        ('share/' + package_name + '/config', glob('config/*')), #include config
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Apostolos Apostolou',
    maintainer_email='aapost04@ucy.ac.cy',
    description='Package to simulate my final year thesis robot using Gazebo and ROS2 Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
