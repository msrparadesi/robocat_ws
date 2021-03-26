from setuptools import setup

package_name = 'mouse_detection_pkg'

setup(
 name=package_name,
 version='0.0.1',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name), ['launch/mouse_detection_pkg_launch.py'])
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Martin Paradesi',
 maintainer_email='martin.paradesi@gmail.com',
 description='This package contains logic to detect mouse from infrared camera images',
 license='Apache 2.0',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'mouse_detection_node = mouse_detection_pkg.mouse_detection_node:main'
     ],
   },
)
