import os
from glob import glob
from setuptools import setup

package_name = 'turtlebot3_random_bouncer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luiz',
    maintainer_email='luizcarloscosmifilho@gmail.com',
    description='Turtlebot3 random bouncer',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_bouncer = turtlebot3_random_bouncer.main:main'
        ],
    },
)
