from setuptools import setup
import os
from glob import glob

package_name = 'system'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='faris',
    maintainer_email='faris071298@gmail.com',
    description='system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'anchor = system.anchor:main',
                'read_data = system.read_data:main',
                'static_tf = system.static_tf:main',
                'visualize = system.visualize:main',
        ],
    },
)
