from setuptools import find_packages, setup

package_name = 'z1_control'

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
    maintainer='andrea',
    maintainer_email='andrea.dantona@unife.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'interactive_motion = z1_control.interactive_motion:main',
            'z1_state_debug = z1_control.state_debug_node:main',
            'z1_incremental = z1_control.z1_incremental_controller:main',
        ],
    },
)
