from setuptools import find_packages, setup

package_name = 'robot_xyz_control'

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
    maintainer='robousr',
    maintainer_email='mar.palomaresq@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          "controller_manager = robot_xyz_control.controller_manager:main",
          "hardware_interface = robot_xyz_control.hardware_interface:main",
          "manipulator_controller = robot_xyz_control.manipulator_controller:main"
        ],
    },
)
