from setuptools import find_packages, setup

package_name = 'robot1_controler'

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
    maintainer_email='alexander.mendez@ingenieria.unam.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          "controller_manager = robot1_controler.controller_manager:main",
          "hardware_interface = robot1_controler.hardware_interface:main",
          "manipulator_controller = robot1_controler.manipulator_controller:main"
        ],
    },
)
