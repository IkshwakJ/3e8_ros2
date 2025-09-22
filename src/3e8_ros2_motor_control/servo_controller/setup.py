from setuptools import find_packages, setup

package_name = 'servo_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'fibre',
        'odrive',
    ],
    zip_safe=True,
    maintainer='Ikshwak',
    maintainer_email='ikshwakjinesh@gmail.com',
    description='Servo controller for ODrive and related drivers',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # syntax: name = package.module:function
            'calibrate_odrive = servo_controller.calibrate_odrive:main'
        ],
    },
)
