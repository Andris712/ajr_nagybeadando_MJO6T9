from setuptools import setup
package_name = 'my_beadando_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],            # <= EZ KELL
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/random_pipeline.launch.py',
            'launch/talker_listener.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Semsey András',
    maintainer_email='semse...@gmail.com',
    description='Kis beadandó ROS2 package.',
    license='Apache-2.0',
    tests_require=['pytest'],
  entry_points={
    'console_scripts': [
        'talker = my_beadando_pkg.publisher_node:main',
        'listener = my_beadando_pkg.subscriber_node:main',
        'random_sensor = my_beadando_pkg.random_sensor_node:main',
        'random_processor = my_beadando_pkg.processor_node:main',
    ],
},
)

