from setuptools import find_packages, setup

package_name = 'img_process'

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
         
            'img_post = img_process.image_postprocess_node:main',   
            'img_grad = img_process.image_extract_info_node:main',
            
            
        ],
    },
)
