import os
from glob import glob
from setuptools import setup

package_name = 'onrobot_2fg7_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Haruto Nagai',
    author_email='u598104j@ecs.osaka-u.ac.jp',
    maintainer='Haruto Nagai',
    maintainer_email='u598104j@ecs.osaka-u.ac.jp',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='OnRobot 2FG7 tutorials.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'onrobot_2fg7_service = onrobot_2fg7_tutorials.onrobot_2fg7_service:main',
            'onrobot_2fg7_gui_bridge = onrobot_2fg7_tutorials.onrobot_2fg7_gui_bridge:main',
        ],
    },
    include_package_data=True,
)
