from setuptools import find_packages, setup

package_name = 'joy_buzzer_pkg'

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
    maintainer='h',
    maintainer_email='h@todo.todo',
    description='Triggers TurtleBot3 buzzer on gamepad A/B button press',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'joy_buzzer = joy_buzzer_pkg.joy_buzzer:main',
            'keyboard_buzzer = joy_buzzer_pkg.keyboard_buzzer:main',
        ],
    },
)
