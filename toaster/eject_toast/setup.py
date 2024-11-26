from setuptools import find_packages, setup

package_name = 'eject_toast'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rviz.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/toaster.urdf.xacro']),
        ('share/' + package_name + '/config', ['config/toast_config.rviz']),
        ('share/' + package_name + '/meshes', ['meshes/toaster.stl']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asaace00',
    maintainer_email='cyberasasoftware@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eject_toast_node = eject_toast.toast_ejector:main'
        ],
    },
)
