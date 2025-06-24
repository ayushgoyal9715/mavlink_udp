import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA

class FlatSurfaceDetector(Node):
    def __init__(self):
        super().__init__('flat_surface_detector')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/point_reliable',
            self.listener_callback,
            10)

        self.flat_pub = self.create_publisher(PointCloud2, '/flat_surface', 10)

        self.get_logger().info("Flat Surface Detector Node Started")

    def listener_callback(self, msg):
        cloud_points = np.array([
            [p[0], p[1], p[2]] for p in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)
        ], dtype=np.float32)

        if cloud_points.shape[0] < 100:
            self.get_logger().warn("Insufficient points for processing")
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud_points)

        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                 ransac_n=3,
                                                 num_iterations=1000)
        [a, b, c, d] = plane_model
        self.get_logger().info(f"Plane model: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        flat_surface = cloud_points[inliers]

        pca = PCA(n_components=3)
        pca.fit(flat_surface)
        eigenvalues = pca.explained_variance_
        flatness_index = eigenvalues[2] / eigenvalues[0]
        self.get_logger().info(f"Flatness index (small is flatter): {flatness_index:.6f}")

        # 🚨 NEW: Check area of the flat region
        inlier_pcd = o3d.geometry.PointCloud()
        inlier_pcd.points = o3d.utility.Vector3dVector(flat_surface)
        bbox = inlier_pcd.get_axis_aligned_bounding_box()
        extent = bbox.get_extent()  # [length_x, length_y, length_z]
        area = extent[0] * extent[1]  # projected area

        self.get_logger().info(f"Flat surface area: {area:.2f} m²")

        if area >= 1.49:  # 4x4 feet ≈ 1.22 x 1.22 m = 1.49 m²
            self.get_logger().info("🟢 SAFE SPOT DETECTED — Sufficient flat area found!")

        # Publish flat surface to /flat_surface
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id
        flat_msg = point_cloud2.create_cloud_xyz32(header, flat_surface.tolist())
        self.flat_pub.publish(flat_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FlatSurfaceDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
