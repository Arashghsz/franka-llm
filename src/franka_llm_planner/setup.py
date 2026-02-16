from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'franka_llm_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='arash',
    maintainer_email='ghasemzadehh.arash@gmail.com',
    description='LLM Task Planner for Franka',
    license='MIT',
    entry_points={
        'console_scripts': [
            'llm_node = franka_llm_planner.llm_node:main',
            'llm_coordinator = franka_llm_planner.llm_coordinator_node:main',
        ],
    },
)
