import os
from glob import glob
from setuptools import setup

package_name = 'miradar_node'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    py_modules=["ppi_visualizer"],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='Kensei Demura',
    author_email='k.demura@qibitech.com',
    maintainer='Kensei Demura',
    maintainer_email='k.demura@qibitech.com',
    keywords=['foo', 'bar'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Radar node',
    license='Apache License 2.0',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'ppi_visualizer = miradar_node.ppi_visualizer:main'
        ],
    },
)
