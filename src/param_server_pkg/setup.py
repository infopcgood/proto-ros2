from setuptools import find_packages, setup

package_name = 'param_server_pkg'

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
    maintainer='info',
    maintainer_email='infopcgood@protonmail.com',
    description='Package that loads and provides all the parameters to other packages',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'param_server = param_server_pkg.param_server:main'
        ],
    },
)
