from setuptools import find_packages, setup

package_name = 'osf_receiver_pkg'

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
    maintainer='infopcgood',
    maintainer_email='infopcgood@protonmail.com',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'osf_receiver = osf_receiver_pkg.osf_receiver:main'
        ],
    },
)
