from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'doctor_engine'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index + package.xml (required)
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include launch files later if you add them:
        # ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        # keep this if you plan to run the doctor MQTT bridge here:
        'paho-mqtt',
    ],
    zip_safe=True,
    maintainer='anas',
    maintainer_email='anas@todo.todo',
    description='Doctor-side runtime (targets + MQTT bridge + optional doctor HW)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'doctor_node = doctor_engine.nodes.doctor_node:main',
            'doctor_mqtt_bridge = doctor_engine.nodes.doctor_mqtt_bridge:main',
            'doctor_network_anomaly_handler = doctor_engine.nodes.doctor_network_anomaly_handler:main'
            # add later if/when you create it:
            # 'doctor_actuator_interface = doctor_engine.nodes.doctor_actuator_interface:main',
        ],
    },
)
