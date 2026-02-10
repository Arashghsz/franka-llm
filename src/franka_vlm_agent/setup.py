from setuptools import find_packages, setup
from glob import glob

package_name = 'franka_vlm_agent'

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
    description='VLM Agent for Franka',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
