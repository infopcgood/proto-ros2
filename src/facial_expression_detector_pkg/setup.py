from setuptools import find_packages, setup

package_name = 'facial_expression_detector_pkg'

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
    maintainer_email='112328126+infopcgood@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'facial_expression_detector = facial_expression_detector_pkg.facial_expression_detector:main'
        ],
    },
)
