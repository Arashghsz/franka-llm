from setuptools import find_packages, setup

package_name = 'franka_coordinator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arash',
    maintainer_email='ghasemzadehh.arash@gmail.com',
    description='Coordinator for Franka LLM Pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'coordinator_node=franka_coordinator.coordinator_node:main',
            'web_handler=franka_coordinator.web_handler:main',
            'test_coordinate_transform=franka_coordinator.test_coordinate_transform:main',
        ],
    },
)
