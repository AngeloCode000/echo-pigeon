from setuptools import find_packages, setup

package_name = 'data_logger'

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
    maintainer='Cardi',
    maintainer_email='CARDILLH@protonmail.com',
    description='CSV / ROS bag logging of detections and tracks',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'logger_node = data_logger.logger_node:main'
        ],
    },
)
