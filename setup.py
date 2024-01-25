from setuptools import setup

package_name = 'uwb_localization'

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
    maintainer='sss0301',
    maintainer_email='atom.9031@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_receiver = uwb_localization.uwb_init:main',
            'uwb_localization = uwb_localization.localization:main'
        ],
    },
)
