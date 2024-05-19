import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robotics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.STL'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        ('lib/' + package_name, [package_name+'/tf.py']),
        ('lib/' + package_name, [package_name+'/quaternions.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paolo',
    maintainer_email='paolo.alfonso.reyes@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2goal = robotics.go2goal:main',
            'odom2wheelspeeds = robotics.odom2wheelspeeds:main',
            'path_generator = robotics.path_generator:main',
            'gz_tf_to_rviz_tf = robotics.gz_tf_to_rviz_tf:main',
            'variable_robot_kinematics = robotics.variable_robot_kinematics:main',
        ],
    },
)
