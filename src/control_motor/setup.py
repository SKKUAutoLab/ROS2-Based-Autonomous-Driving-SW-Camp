from setuptools import find_packages, setup

package_name = 'control_motor'

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
    maintainer='jinsunlee',
    maintainer_email='with23skku@g.skku.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gen_control_data = control_motor.generate_control_signal_node:main',
            'convert_protocol = control_motor.convert_protocol_node:main',
            'send_serial = control_motor.control_motor_node:main',

        ],
    },
)
