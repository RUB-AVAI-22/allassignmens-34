from setuptools import setup

package_name = 'camera_task'

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
    maintainer='ubuntu',
    maintainer_email='yu.zha@rub.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_task.camera_node:main',
            'image_processing_node = camera_task.image_processing_node:main',
            'image_display_node = camera_task.image_display_node:main',
            'sensor_fusion_node = camera_task.sensor_fusion_node:main',
            'lidar_sensor_node = camera_task.lidar_sensor_node:main'
        ],
    },
)
