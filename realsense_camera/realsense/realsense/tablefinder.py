import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2
from sensor_msgs.msg import PointCloud2
import numpy as np

class TableFind(Node):
    def __init__(self):
        super().__init__("table_find")
        self._sub = self.create_subscription(PointCloud2, "pcl_handler", self.pcl_handler, 10)
        self._cropped = self.create_publisher(PointCloud2, "pcl_cropped", 10)
        self._voxel = self.create_publisher(PointCloud2, "pcl_voxel", 10)
        self._inplane = self.create_publisher(PointCloud2, "pcl_inplane", 10)

    def pcl_handler(self, pcl_msg: PointCloud2):
        # Convert ROS2 message to numpy array
        points = sensor_msgs_py.point_cloud2.read_points_numpy(pcl_msg, ['x', 'y', 'z'])
        
        # Crop points
        cropped_points = self.crop_points(points)
        cropped_msg = sensor_msgs_py.point_cloud2.create_cloud_xyz32(pcl_msg.header, cropped_points)
        self._cropped.publish(cropped_msg)
        
        # Voxel downsampling
        voxel_points = self.voxel_downsample(cropped_points)
        voxel_msg = sensor_msgs_py.point_cloud2.create_cloud_xyz32(pcl_msg.header, voxel_points)
        self._voxel.publish(voxel_msg)
        
        # Plane segmentation
        plane_points = self.segment_plane(voxel_points)
        if plane_points is not None:
            inplane_msg = sensor_msgs_py.point_cloud2.create_cloud_xyz32(pcl_msg.header, plane_points)
            self._inplane.publish(inplane_msg)

    def crop_points(self, points):
        # Crop points to a specific bounding box
        mask = (
            (points[:, 0] >= -0.75) & (points[:, 0] <= 1.0) &
            (points[:, 1] >= -0.6) & (points[:, 1] <= 0.5) &
            (points[:, 2] >= 0.1) & (points[:, 2] <= 2.0)
        )
        return points[mask]

    def voxel_downsample(self, points, leaf_size=0.01):
        # Simple voxel downsampling using NumPy
        min_coords = np.floor(points / leaf_size) * leaf_size
        unique_coords, indices = np.unique(min_coords, axis=0, return_index=True)
        return points[indices]

    def segment_plane(self, points, distance_threshold=0.02):
        # RANSAC plane segmentation using NumPy
        if len(points) < 3:
            return None

        best_plane = None
        max_inliers = 0

        for _ in range(100):  # RANSAC iterations
            # Randomly select 3 points
            sample_indices = np.random.choice(len(points), 3, replace=False)
            sample_points = points[sample_indices]
            
            # Compute plane equation
            v1 = sample_points[1] - sample_points[0]
            v2 = sample_points[2] - sample_points[0]
            normal = np.cross(v1, v2)
            normal /= np.linalg.norm(normal)
            
            # Distance of points to plane
            distances = np.abs(np.dot(points - sample_points[0], normal))
            
            # Count inliers
            inliers = points[distances < distance_threshold]
            
            if len(inliers) > max_inliers:
                max_inliers = len(inliers)
                best_plane = inliers

        return best_plane

def table_entry(args=None):
    rclpy.init(args=args)
    node = TableFind()
    rclpy.spin(node)
    rclpy.shutdown()