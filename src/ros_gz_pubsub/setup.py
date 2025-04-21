from setuptools import find_packages, setup

package_name = 'ros_gz_pubsub'

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
    maintainer='heinrich',
    maintainer_email='boekhoffhenry@gmail.com',
    description='Publisher and subscribers nodes to interact with gazebo',
    license='Do what you want with this, I dont care.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'runner = ros_gz_pubsub.pubsub:main'
        ],
    },
)
