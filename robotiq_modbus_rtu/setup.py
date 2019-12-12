from setuptools import find_packages
from setuptools import setup

package_name = 'robotiq_modbus_rtu'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages('src', exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Kelsey Hawkins',
    author_email='kphawkins@gatech.edu',
    maintainer='Jean-Philippe Roberge',
    maintainer_email='ros@robotiq.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD 3-Clause License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'A stack to communicate with Robotiq grippers using the Modbus RTU protocol.'
    ),
    license='BSD 3-Clause License',
    tests_require=['pytest'],
    package_dir={'':'src'}, 
)
