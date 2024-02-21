from setuptools import find_packages, setup

package_name = 'yolov8_ros'

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
    
    description='ref. https://github.com/mgonzs13/yolov8_ros ',
    license='ref. https://github.com/mgonzs13/yolov8_ros ',
    
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'predict = yolov8_ros.instance_segmentation_node:main',
                
        ],
    },
)
