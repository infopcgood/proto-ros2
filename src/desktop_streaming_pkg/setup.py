from setuptools import find_packages, setup

package_name = 'desktop_streaming_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'mss>=10.2,<11', 'Pillow>=9.1'],
    zip_safe=True,
    maintainer='infopcgood',
    maintainer_email='infopcgood@protonmail.com',
    description='Stream desktop captures to the proto-ros2 screen dispatcher.',
    license='GPL-3.0-only',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'desktop_streaming = desktop_streaming_pkg.desktop_streaming:main',
        ],
    },
)
