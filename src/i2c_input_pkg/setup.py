from setuptools import find_packages, setup

package_name = 'i2c_input_pkg'

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
    description='Package to process and broadcast I2C input data',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'i2c_input = i2c_input_pkg.i2c_input:main'
        ],
    },
)
