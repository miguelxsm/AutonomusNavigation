"""
Station Detector node - Detects the charging station (4-pillar pattern).
FIB UPC - Robotics Project

Processes LiDAR data to identify 4 small cylindrical pillars (~5cm diameter)
arranged in a 40cm x 40cm square pattern. Publishes the station center
pose when detected.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from rob_project.utils import (
    euler_from_quaternion,
    scan_to_cartesian,
    transform_point_to_map,
    distance,
    quaternion_from_yaw,
)


class StationDetector(Node):
    """Node that detects the 4-pillar charging station from LiDAR data."""

    def __init__(self):
        super().__init__('station_detector')

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=10
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, depth=10
        )

        # Parameters
        self.declare_parameter('station_side', 0.40)
        self.declare_parameter('pillar_diameter', 0.05)
        self.declare_parameter('side_tolerance', 0.15)
        self.declare_parameter('cluster_distance', 0.08)
        self.declare_parameter('min_cluster_points', 2)
        self.declare_parameter('max_cluster_points', 15)
        self.declare_parameter('enabled', False)

        self.station_side = self.get_parameter('station_side').value
        self.pillar_diam = self.get_parameter('pillar_diameter').value
        self.side_tol = self.get_parameter('side_tolerance').value
        self.cluster_dist = self.get_parameter('cluster_distance').value
        self.min_cluster_pts = self.get_parameter('min_cluster_points').value
        self.max_cluster_pts = self.get_parameter('max_cluster_points').value
        self.enabled = self.get_parameter('enabled').value

        # State
        self.scan_data = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.station_found = False
        self.station_center = None  # (x, y) in map frame
        self.station_yaw = 0.0
        self.detection_count = 0
        self.required_detections = 3  # Confirm with multiple scans

        # Subscribers
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_best_effort
        )
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_best_effort
        )

        # Publishers
        self.station_pose_pub = self.create_publisher(
            PoseStamped, '/station_pose', qos_reliable
        )
        self.station_found_pub = self.create_publisher(
            Bool, '/station_found', qos_reliable
        )

        # Detection timer at 2 Hz (heavy computation)
        self.create_timer(0.5, self.detect_station)

        self.get_logger().info('Station detector initialized (disabled until Phase II).')

    def scan_callback(self, msg):
        """Store latest scan data."""
        self.scan_data = msg

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    def cluster_points(self, points):
        """Simple distance-based clustering (similar to DBSCAN with minPts=1).

        Groups nearby points into clusters. Each cluster potentially
        represents a pillar.

        Args:
            points: List of (x, y) tuples in robot frame.

        Returns:
            list[list[tuple]]: List of clusters, each a list of points.
        """
        if not points:
            return []

        visited = [False] * len(points)
        clusters = []

        for i in range(len(points)):
            if visited[i]:
                continue
            cluster = [points[i]]
            visited[i] = True
            queue = [i]

            while queue:
                idx = queue.pop(0)
                px, py = points[idx]
                for j in range(len(points)):
                    if visited[j]:
                        continue
                    qx, qy = points[j]
                    if distance(px, py, qx, qy) < self.cluster_dist:
                        visited[j] = True
                        cluster.append(points[j])
                        queue.append(j)

            clusters.append(cluster)

        return clusters

    def cluster_center(self, cluster):
        """Compute centroid of a cluster.

        Args:
            cluster: List of (x, y) tuples.

        Returns:
            tuple[float, float]: Centroid (x, y).
        """
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        return (cx, cy)

    def cluster_diameter(self, cluster):
        """Compute maximum span of a cluster.

        Args:
            cluster: List of (x, y) tuples.

        Returns:
            float: Maximum distance between any two points.
        """
        max_d = 0.0
        for i in range(len(cluster)):
            for j in range(i + 1, len(cluster)):
                d = distance(
                    cluster[i][0], cluster[i][1],
                    cluster[j][0], cluster[j][1]
                )
                if d > max_d:
                    max_d = d
        return max_d

    def is_pillar_cluster(self, cluster):
        """Check if a cluster could be a pillar (~5cm diameter cylinder).

        Args:
            cluster: List of (x, y) tuples.

        Returns:
            bool: True if cluster matches pillar profile.
        """
        n = len(cluster)
        if n < self.min_cluster_pts or n > self.max_cluster_pts:
            return False

        diam = self.cluster_diameter(cluster)
        # Pillar is ~5cm diameter; allow some tolerance
        return diam < self.pillar_diam * 4.0  # < 20cm span

    def find_square_pattern(self, centroids):
        """Find 4 centroids forming a square of side ~40cm.

        Tries all combinations of 4 centroids and checks if they form
        a square with the expected side length.

        Args:
            centroids: List of (x, y) tuples (cluster centers).

        Returns:
            tuple or None: (center_x, center_y, yaw, corners) if found.
        """
        n = len(centroids)
        if n < 4:
            return None

        best = None
        best_error = float('inf')
        expected = self.station_side

        # Try all combinations of 4
        for i in range(n):
            for j in range(i + 1, n):
                for k in range(j + 1, n):
                    for m in range(k + 1, n):
                        pts = [centroids[i], centroids[j],
                               centroids[k], centroids[m]]
                        result = self._check_square(pts, expected)
                        if result is not None and result[0] < best_error:
                            best_error = result[0]
                            best = (pts, result)

        if best is None:
            return None

        pts, (error, ordering) = best
        # Compute center
        cx = sum(p[0] for p in pts) / 4.0
        cy = sum(p[1] for p in pts) / 4.0
        # Compute orientation from first edge
        p0, p1 = ordering[0], ordering[1]
        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])

        return (cx, cy, yaw, pts)

    def _check_square(self, pts, expected_side):
        """Check if 4 points form a square with given side length.

        Args:
            pts: List of 4 (x, y) tuples.
            expected_side: Expected side length.

        Returns:
            tuple or None: (total_error, ordered_points) if valid square.
        """
        # Compute all 6 pairwise distances
        dists = []
        for i in range(4):
            for j in range(i + 1, 4):
                d = distance(pts[i][0], pts[i][1], pts[j][0], pts[j][1])
                dists.append((d, i, j))

        dists.sort(key=lambda x: x[0])

        # A square has 4 equal sides and 2 equal diagonals
        # sorted: [s, s, s, s, d, d] where d = s * sqrt(2)
        sides = [dists[i][0] for i in range(4)]
        diags = [dists[i][0] for i in range(4, 6)]

        # Check sides are close to expected
        side_errors = [abs(s - expected_side) for s in sides]
        diag_expected = expected_side * math.sqrt(2.0)
        diag_errors = [abs(d - diag_expected) for d in diags]

        total_error = sum(side_errors) + sum(diag_errors)

        if all(e < self.side_tol for e in side_errors) and \
           all(e < self.side_tol * 1.5 for e in diag_errors):
            # Build ordering (walk around the square)
            # Use the adjacency from side edges
            adj = {i: [] for i in range(4)}
            for d, i, j in dists[:4]:
                adj[i].append(j)
                adj[j].append(i)

            # Try to build cycle
            try:
                order = [0]
                visited = {0}
                for _ in range(3):
                    current = order[-1]
                    for nxt in adj[current]:
                        if nxt not in visited:
                            order.append(nxt)
                            visited.add(nxt)
                            break
                ordered = [pts[i] for i in order]
                return (total_error, ordered)
            except (IndexError, KeyError):
                return (total_error, pts)

        return None

    def detect_station(self):
        """Main detection loop - called by timer."""
        if not self.enabled or self.scan_data is None:
            return

        if self.station_found:
            # Keep publishing the known station pose
            self._publish_station()
            return

        # Convert scan to cartesian points
        points = scan_to_cartesian(self.scan_data)
        if len(points) < 8:
            return

        # Cluster the points
        clusters = self.cluster_points(points)

        # Filter for pillar-like clusters
        pillar_clusters = [c for c in clusters if self.is_pillar_cluster(c)]

        if len(pillar_clusters) < 4:
            self.detection_count = 0
            return

        # Get centroids
        centroids = [self.cluster_center(c) for c in pillar_clusters]

        # Look for square pattern
        result = self.find_square_pattern(centroids)
        if result is None:
            self.detection_count = 0
            return

        cx_robot, cy_robot, yaw_robot, corners = result

        # Transform center to map frame
        cx_map, cy_map = transform_point_to_map(
            cx_robot, cy_robot,
            self.robot_x, self.robot_y, self.robot_yaw
        )
        yaw_map = self.robot_yaw + yaw_robot

        self.detection_count += 1
        self.get_logger().info(
            f'Station candidate detected ({self.detection_count}/'
            f'{self.required_detections}) at map ({cx_map:.2f}, {cy_map:.2f})'
        )

        if self.detection_count >= self.required_detections:
            self.station_found = True
            self.station_center = (cx_map, cy_map)
            self.station_yaw = yaw_map
            self.get_logger().info(
                f'STATION CONFIRMED at ({cx_map:.2f}, {cy_map:.2f}), '
                f'yaw={math.degrees(yaw_map):.1f} deg'
            )
            self._publish_station()

            found_msg = Bool()
            found_msg.data = True
            self.station_found_pub.publish(found_msg)

    def _publish_station(self):
        """Publish the station pose."""
        if self.station_center is None:
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.station_center[0]
        pose.pose.position.y = self.station_center[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation = quaternion_from_yaw(self.station_yaw)
        self.station_pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = StationDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Station detector interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
