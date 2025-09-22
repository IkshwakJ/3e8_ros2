from setuptools import find_packages, setup

package_name = 'foxglove_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/foxglove_mapping_layout.json']),
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
