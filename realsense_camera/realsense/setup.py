from setuptools import setup

package_name = 'realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # The directory containing your Python code
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/pcl.rviz', 'config/tags.yaml', 'config/tags_tf.rviz']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py', 'launch/rs_launch.py', 'launch/visualFeed.launch.py']),
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
            'table = realsense.tablefinder:table_entry',
            'simpleDetection = realsense.simpleDetection:main',
            'visualizeFeed = realsense.visualizeFeed:main',
            'apriltagsTracker = realsense.apriltagsTracker:main'
        ],
    },
)
