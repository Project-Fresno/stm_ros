from setuptools import setup

package_name = 'stm_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hemanth',
    maintainer_email='hemanthvasireddy2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = stm_ros.odom_pub:main',
            'cmd_vel_sub = stm_ros.subscribe_cmd_send_serial:main',
        ],
    },
)
