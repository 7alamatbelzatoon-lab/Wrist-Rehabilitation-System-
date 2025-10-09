from setuptools import setup, find_packages
from pathlib import Path

package_name = 'wrist_replay'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install urdf and launch files
        ('share/' + package_name + '/urdf', ['urdf/wrist.urdf']),
        ('share/' + package_name + '/launch', ['launch/replay.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anas',
    maintainer_email='anas@todo.todo',
    description='CSV-to-RViz wrist replay',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'replay_node = wrist_replay.replay_node:main',
        ],
    },
)

