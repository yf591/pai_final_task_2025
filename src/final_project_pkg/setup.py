# src/final_project_pkg/setup.py

import os
from glob import glob
from setuptools import setup

package_name = 'final_project_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yf591',
    maintainer_email='student@example.com', # ダミーのメールアドレス
    description='A package for the final assignment of the Physical AI course.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # AI強化タスク実行ノードのエントリポイント
            'task_executor = final_project_pkg.task_executor:main',
        ],
    },
)