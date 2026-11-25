from glob import glob
from setuptools import find_packages, setup

setup(name='prootface_pkg',version='0.1.0',packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages',['resource/prootface_pkg']),
                  ('share/prootface_pkg',['package.xml','README.md']),
                  ('share/prootface_pkg/launch',glob('launch/*.launch.py')),
                  ('share/prootface_pkg/config',glob('config/*.yaml'))],
      install_requires=['setuptools'],zip_safe=True,
      maintainer='infopcgood',maintainer_email='infopcgood@protonmail.com',
      description='Continuous two-side face renderer using the external prootface core',
      license='GPL-3.0-only',
      entry_points={'console_scripts':['prootface = prootface_pkg.node:main']})
