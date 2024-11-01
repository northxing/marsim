from setuptools import find_packages, setup

package_name = 'marsim_control_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yihan.xing@uis.no',
    description='This package contains control routines implemented as nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bluerov2_pid_disp_xyyaw_control = marsim_control_py.scripts.bluerov2_pid_disp_xyyaw_control:main',
            'bluerov2_pid_pose_z_control = marsim_control_py.scripts.bluerov2_pid_pose_z_control:main',
            'bluerov2_pid_disp_z = marsim_control_py.scripts.bluerov2_pid_disp_z_control:main',
            'bluerov2_stop_thrusters = marsim_control_py.scripts.bluerov2_stop_thrusters:main',
        ],
    },
)