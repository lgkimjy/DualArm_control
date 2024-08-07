import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'yori_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'meshes', 'yori'), glob('meshes/yori/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='donghun',
    maintainer_email='donghunnoh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_position_publisher_test = yori_description.joint_position_publisher_test:main',
        ],
    },
)
