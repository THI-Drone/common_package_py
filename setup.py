from setuptools import find_packages, setup

package_name = 'common_package_py'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='philipp',
    maintainer_email='phg63865@thi.de',
    description='Common package for all Python nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'common_node = common_package_py.common_node:main',
        ],
    },
)
