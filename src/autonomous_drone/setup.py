from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'autonomous_drone'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drone_developer',
    maintainer_email='developer@example.com',
    description='Autonomous drone software',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_processor = autonomous_drone.camera_processor:main',
            'ai_detector = autonomous_drone.ai_detector:main',

        ],
    },
)
