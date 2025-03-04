from setuptools import setup

package_name = 'auto_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='cgrund@uvm.edu',
    description='First shot at autonomy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Model_Free_Node = auto_drive.Model_Free_Node:main'
        ],
    },
)
