from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bluerov2_ctrl_serv_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yihan.xing@uis.no',
    description='This package contains the action servers for the control of the BlueROV2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_disp_z_control_action = bluerov2_ctrl_serv_py.pid_disp_z_control_action:main',
            'pid_disp_z_control_client = bluerov2_ctrl_serv_py.pid_disp_z_control_client:main',
            'pid_abs_yaw_control_action = bluerov2_ctrl_serv_py.pid_abs_yaw_control_action:main',
            'pid_abs_yaw_control_client = bluerov2_ctrl_serv_py.pid_abs_yaw_control_client:main'
        ],
    },
)