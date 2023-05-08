from setuptools import setup

package_name = 'gpt_ros'

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
    maintainer='tong',
    maintainer_email='mutongtongxue@yeah.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpt_ros2_server = gpt_ros.gpt_ros2_server:main',
            'gpt_ros2_client = gpt_ros.gpt_ros2_client:main',
        ],
    },
)
