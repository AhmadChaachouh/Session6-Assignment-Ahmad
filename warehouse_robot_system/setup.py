from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'warehouse_robot_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmad',
    maintainer_email='ahmad.chaachouh.cad@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'item_delivery_action_server = warehouse_robot_system.item_delivery_action_server:main',
            'item_delivery_action_client = warehouse_robot_system.item_delivery_action_client:main',
            'stock_checker_service_server = warehouse_robot_system.stock_checker_service_server:main',
            'stock_checker_service_client = warehouse_robot_system.stock_checker_service_client:main',
        ],
    },
)
