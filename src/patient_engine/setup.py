from setuptools import find_packages, setup
from glob import glob

package_name = 'patient_engine'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config YAMLs so get_package_share_directory can find them
        ('share/' + package_name + '/config', [
            'config/motion_thresholds_beginner.yaml',
            'config/motion_thresholds_advanced.yaml',
        ]),
        # If you later add launch files:
        # ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt',
        'PyYAML',  # for profile YAML loading in motion_detector
    ],
    zip_safe=True,
    maintainer='anas',
    maintainer_email='anas@todo.todo',
    description='Patient-side runtime (control, detectors, orchestrator, MQTT bridge)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # bridges
            'patient_mqtt_bridge = patient_engine.nodes.bridges.patient_mqtt_bridge:main',
            # detectors
            'motion_detector = patient_engine.nodes.detectors.motion_detector:main',
            'network_detector = patient_engine.nodes.detectors.network_detector:main',
            # orchestrator
            'patient_orchestrator = patient_engine.nodes.orchestrator.patient_orchestrator:main',
            # locomotion
            'patient_node = patient_engine.nodes.patient_node:main',
            # supervisor
            'stack_manager = patient_engine.nodes.stack_manager_node:main',
            #Network Test
            'network_detector_scenario_test = patient_engine.nodes.detectors.network_detector_scenario_test:main',
            #Position control mode driver Driver
            'dxl_position_driver = patient_engine.nodes.drivers.dxl_position_driver:main',
            #Current control mode driver Driver
            'dxl_current_driver = patient_engine.nodes.drivers.dxl_current_driver:main',
            #Current Control
            'patient_current_controller = patient_engine.nodes.drivers.patient_current_controller:main',
        ],
    },
)
