from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'skyforge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/skyforge']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),  # ✅ Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dwidjaja',
    maintainer_email='dwidjaja@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test = skyforge.my_first_node:main"
        ],
    },
)
