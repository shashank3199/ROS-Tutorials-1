from setuptools import setup

package_name = 'py_pub_sub_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shashank Goyal',
    maintainer_email='shashank3199@gmail.com',
    description='Python publisher and subscriber package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py_twist_publisher = py_pub_sub_pkg.twist_publisher:main',
            'py_twist_subscriber = py_pub_sub_pkg.twist_subscriber:main',
        ],
    },
)