from setuptools import setup

package_name = 'mixed_pub_sub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Register the package with ROS2
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shashank Goyal',
    maintainer_email='shashank3199@gmail.com',
    description='Mixed Python publisher and C++ subscriber package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_publisher = mixed_pub_sub.twist_publisher:main',
        ],
    },
)