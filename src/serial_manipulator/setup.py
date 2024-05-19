from setuptools import find_packages, setup

package_name = 'serial_manipulator'
submodules = 'serial_manipulator/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricardon',
    maintainer_email='ricardonavarro2003@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_manipulator = serial_manipulator.dynamic_manipulator:main'
        ],
    },
)
