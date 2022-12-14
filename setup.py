from setuptools import setup
import os
from glob import glob

package_name = 'autow_ros'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name, f'{package_name}.utils'],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
     (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='TODO',
 maintainer_email='TODO',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'autow_node = autow_ros.autow_node:main',
             'controller_node = autow_ros.controller_node:main',
             'driver_node = autow_ros.driver:main'
     ],
   },
)