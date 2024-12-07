"""The setup file called during the build."""
from setuptools import setup

package_name = 'realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # The directory containing your Python code
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'config/tags.yaml',
            'launch/camera.launch.py',
            'launch/visualize_tags.launch.py',
        ]),
    ],
    install_requires=['setuptools'],  # Dependencies for Python
    zip_safe=True,
    maintainer='Matthew Elwin',
    maintainer_email='elwin@northwestern.edu',
    description='Demonstrate some point cloud processing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_localizer = realsense.camera_localizer:main',
            'tag_visualizer = realsense.tag_visualizer:main',
        ],
    },
)
