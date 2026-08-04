from setuptools import find_packages, setup

package_name = 'track_visualizer'

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
    description='RViz marker publishing for target tracks',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'visualizer_node = track_visualizer.visualizer_node:main'
        ],
    },
)
