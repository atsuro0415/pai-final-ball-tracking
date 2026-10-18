from setuptools import find_packages, setup

package_name = 'dtw_control'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Heading hold, obstacle handling and ball chase for dtw_robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heading_hold = dtw_control.heading_hold:main',
            'obstacle_stop = dtw_control.obstacle_stop:main',
            'obstacle_avoid = dtw_control.obstacle_avoid:main',
            'ball_chase = dtw_control.ball_chase:main',
        ],
    },
)
