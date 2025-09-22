from setuptools import find_packages, setup

package_name = '3e8_ros2_system_tests'

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
    maintainer='Ikshwak',
    maintainer_email='ikshwakjinesh@gmail.com',
    description='TODO: Package description',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
