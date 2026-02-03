from setuptools import setup

package_name = 'python_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Python Maintainer',
    maintainer_email='python@example.com',
    description='A Python ROS 2 node example',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = python_node.publisher_node:main',
            'listener = python_node.subscriber_node:main',
        ],
    },
)