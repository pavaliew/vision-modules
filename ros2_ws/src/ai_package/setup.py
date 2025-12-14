from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ai_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Базовые файлы пакета
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch файлы
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        
        # Конфигурационные файлы (если есть)
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.yml')),
        ('share/' + package_name + '/config', glob('config/*.json')),
        
        # Другие ресурсы
        ('share/' + package_name + '/resource', glob('resource/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        #models
        (os.path.join('share', package_name, 'models'), 
         glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV Developer',
    maintainer_email='dev@example.com',
    description='AI modules for UAV operations',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'camera_publisher = ai_package.camera.camera_publisher:main',
            'road_field_detector = ai_package.BorderControl.scv_cam:main',
            'stitcher = ai_package.BorderControl.camstitcher:main',
            'sift_detector = ai_package.BorderControl.ssift_cam:main',
            'field_object_detector = ai_package.FlyingObjectsDetector.inference_node:main',
            'landing_takeoff_node = ai_package.LandingTakeoffSafety.landing_takeoff_node:main',
            # Other nodes
        ],
    },
)
