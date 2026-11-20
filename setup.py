from setuptools import setup
from glob import glob

package_name = 'selection_e_manager'
setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='selection-e',
    maintainer_email='noreply@example.com',
    description='Waypoint-triggered passage/Florence OCR/speech orchestrator',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'manager_node = selection_e_manager.manager_node:main',
        'camera_node = selection_e_manager.camera_node:main',
        'ocr_node = selection_e_manager.ocr_node:main',
        'speaker_node = selection_e_manager.speaker_node:main',
    ]},
)
