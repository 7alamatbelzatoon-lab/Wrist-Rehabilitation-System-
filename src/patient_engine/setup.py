from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'patient_engine'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include launch files later if you add them:
        # ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # include cfg files later if you add them:
        # ('share/' + package_name + '/cfg', glob('cfg/*')),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt',  # patient bridge uses MQTT now
    ],
    zip_safe=True,
    maintainer='anas',
    maintainer_email='anas@todo.todo',
    description='Patient-side runtime (control + MQTT bridge; safety later)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patient_node = patient_engine.nodes.patient_node:main',
            'patient_mqtt_bridge = patient_engine.nodes.patient_mqtt_bridge:main',
            'patient_network_anomaly_handler = patient_engine.nodes.patient_network_anomaly_handler:main',
            'patient_motion_anomaly_handler = patient_engine.nodes.patient_motion_anomaly_handler.py:main'
            # add later:
            # 'safety_monitor = patient_engine.nodes.safety_monitor:main',
            # 'patient_actuator_interface = patient_engine.nodes.patient_actuator_interface:main',
        ],
    },
)
