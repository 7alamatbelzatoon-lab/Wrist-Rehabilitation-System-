from setuptools import find_packages, setup

package_name = 'rehab_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='anas',
    maintainer_email='anas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "patient_node = rehab_system.patient_node : main",
            "doctor_node = rehab_system.doctor_node : main",
            "mqtt_bridge = rehab_system.mqtt_bridge : main"


        ],
    },
)
