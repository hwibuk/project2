import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'project2'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 일반 폴더들 설치
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
        (os.path.join('share', package_name, 'textures'), glob(os.path.join('textures', '*.png'))),
    ]

    # models 폴더의 계층 구조를 유지하며 추가하는 로직
    for root, dirs, files in os.walk('models'):
        for file in files:
            file_path = os.path.join(root, file)
            dest_path = os.path.join('share', package_name, root)
            data_files.append((dest_path, [file_path]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwibuk',
    maintainer_email='gnl9658@gmail.com',
    description='ROS 2 Differential Drive Robot Package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
