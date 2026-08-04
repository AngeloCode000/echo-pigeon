from setuptools import find_packages, setup

package_name = 'ti_radar_driver'

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
    description='Serial/USB driver for the TI IWR6843ISK-ODS mmWave radar',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'radar_driver_node = ti_radar_driver.radar_driver_node:main'
        ],
    },
)
