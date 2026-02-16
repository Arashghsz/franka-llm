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
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'requests', 'pillow', 'opencv-python', 'pyyaml'],
    zip_safe=True,
    maintainer='arash',
    maintainer_email='ghasemzadehh.arash@gmail.com',
    description='VLM Agent for Franka - LLaVA 7B on Jetson',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vlm_node=franka_vlm_agent.vlm_node:main',
            'vlm_test=franka_vlm_agent.vlm_test:main',
        ],
    },
)

