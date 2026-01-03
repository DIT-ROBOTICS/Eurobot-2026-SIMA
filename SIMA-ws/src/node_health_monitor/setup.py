from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'node_health_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 包含 launch 檔案
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 包含 config 檔案
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='fujiade0518@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'supervisor = node_health_monitor.supervisor:main',
        ],
    },
)
