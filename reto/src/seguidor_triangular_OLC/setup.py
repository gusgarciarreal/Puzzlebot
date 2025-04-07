from setuptools import find_packages, setup

package_name = 'seguidor_triangular_OLC'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gusgm',
    maintainer_email='gusgarciarrealm@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seguidor_triangular = seguidor_triangular_OLC.seguidor_triangular_OLC:main',
        ],
    },
)
