from setuptools import find_packages, setup

package_name = 'csv_decoder'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    py_modules=['csv_decoder'],
    data_files=[
    	('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/SpaFrancorchamps.csv']),
        ('share/' + package_name + '/config', ['config/figure8.csv']),
        ('share/' + package_name + '/config', ['config/figure0.csv']),
    ],
    install_requires=['setuptools', 'std_msgs'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='CSV decoder node for Crazyflie goal publishing',
    license='Apache-2.0',
    tests_require=['pytest'],
    
    
    entry_points={
        'console_scripts': [
            'csv_decoder = csv_decoder:main'
        ],
    },
)

