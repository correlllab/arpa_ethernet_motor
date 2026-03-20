from setuptools import find_packages, setup

package_name = 'arpa_ethernet_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='maxlconway@gmail.com',
    description='Ethernet motor control and motor current ROS nodes',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_node = arpa_ethernet_motor.scripts.motor_node:main',
            'motor_current_node = arpa_ethernet_motor.scripts.motor_current_node:main',
        ],
    },
)
