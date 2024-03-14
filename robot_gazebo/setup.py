import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*"))),
        (os.path.join("share", package_name, "worlds"), glob(os.path.join("worlds", "*"))),
        (os.path.join("share", package_name, "configs"), glob(os.path.join("configs", "*"))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linzhangUCA',
    maintainer_email='lzhang12@ucae.edu',
    description='Demo robot simulation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
