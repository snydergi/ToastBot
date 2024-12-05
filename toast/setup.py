"""Setup."""
from setuptools import find_packages, setup

package_name = 'toast'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/makeToast.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gis',
    maintainer_email='grasnyder2001@gmail.com',
    description='TODO: Package description',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'toast_bot = toast.toastBot:main'
        ],
    },
)
