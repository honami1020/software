from setuptools import find_packages, setup

package_name = 'soft_ws'

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
    maintainer='honami',
    maintainer_email='khona475.f@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_to_odom_node=soft_ws.gps_to_odom_node:main',
            'goal_navigation_with_imu=soft_ws.goal_navigation_with_imu:main',
            'goal_reporter_node=soft_ws.goal_reporter_node:main',
            'fake_imu_publisher=soft_ws.fake_imu_publisher:main',
            'gps_controller=soft_ws.gps_controller:main',
            'navigation_imu=soft_ws.navigation_imu:main',
            'image_recognition=soft_ws.image_recognition:main',
        ],
    },
)
