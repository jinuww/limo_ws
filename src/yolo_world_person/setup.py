from setuptools import setup

package_name = 'yolo_world_person'

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
    maintainer='jinwoo',
    maintainer_email='jinwoo@todo.todo',
    description='YOLO-World person detector for Limo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_node = yolo_world_person.yolo_world_person_node:main',
            'dabai_cam_pub = yolo_world_person.dabai_cam_pub:main',
            'evacuation_commander = yolo_world_person.evacuation_commander:main',
            'threat_detector = yolo_world_person.threat_detector:main',
            'remote_audio_player = yolo_world_person.remote_audio_player:main',
        ],
    },
)
