from setuptools import setup

package_name = 'rc_navigation_pkg'

setup(
 name=package_name,
 version='0.0.1',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name), ['launch/rc_navigation_pkg_launch.py'])
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Martin Paradesi',
 maintainer_email='martin.paradesi@gmail.com',
 description='This package contains logic to move the DeepRacer forward and backward',
 license='Apache 2.0',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'rc_navigation_node = rc_navigation_pkg.rc_navigation_node:main'
     ],
   },
)
