from setuptools import find_packages, setup

package_name = 'my_robot_tools'

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
    maintainer='cchyun',
    maintainer_email='cchyun@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_navigator = my_robot_tools.lidar_navigator:main',
            'pid_arm_control = my_robot_tools.pid_arm_control:main'
        ],
    },
)
