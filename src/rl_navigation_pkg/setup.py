from glob import glob

from setuptools import find_packages, setup

package_name = 'rl_navigation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'gymnasium', 'numpy'],
    zip_safe=True,
    maintainer='taku_ros',
    maintainer_email='taku_ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'env_smoke_test = rl_navigation_pkg.nodes.env_smoke_test:main',
            'ekf_input_gate = rl_navigation_pkg.nodes.ekf_input_gate:main',
            'release_driver = rl_navigation_pkg.nodes.release_driver:main',
        ],
    },
)
