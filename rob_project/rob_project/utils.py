"""
Utility functions for the rob_project autonomous navigation system.
FIB UPC - Robotics Project
"""

import math
from geometry_msgs.msg import Quaternion, PoseStamped
from builtin_interfaces.msg import Time


def euler_from_quaternion(q):
    """Extract yaw angle from a quaternion.

    Args:
        q: Quaternion (geometry_msgs/Quaternion or object with x, y, z, w).

    Returns:
        float: Yaw angle in radians [-pi, pi].
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    """Create a Quaternion message from a yaw angle.

    Args:
        yaw: Yaw angle in radians.

    Returns:
        Quaternion: geometry_msgs/Quaternion message.
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def normalize_angle(angle):
    """Normalize angle to [-pi, pi].

    Args:
        angle: Angle in radians.

    Returns:
        float: Normalized angle.
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance(x1, y1, x2, y2):
    """Euclidean distance between two 2D points."""
    return math.hypot(x2 - x1, y2 - y1)


def angle_to_target(current_x, current_y, target_x, target_y):
    """Angle from current position to target position.

    Returns:
        float: Angle in radians [-pi, pi].
    """
    return math.atan2(target_y - current_y, target_x - current_x)


def angle_difference(target_yaw, current_yaw):
    """Shortest signed angle difference (target - current), normalized.

    Returns:
        float: Angle difference in radians [-pi, pi].
               Positive = turn left, Negative = turn right.
    """
    diff = target_yaw - current_yaw
    return normalize_angle(diff)


def create_pose_stamped(x, y, yaw, frame_id='map', stamp=None):
    """Create a PoseStamped message.

    Args:
        x, y: Position coordinates.
        yaw: Orientation in radians.
        frame_id: Reference frame (default 'map').
        stamp: Optional Time message.

    Returns:
        PoseStamped: The constructed pose message.
    """
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp is not None:
        pose.header.stamp = stamp
    else:
        pose.header.stamp = Time()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation = quaternion_from_yaw(yaw)
    return pose


def get_min_range_in_sector(scan_msg, angle_min_rad, angle_max_rad):
    """Return minimum range in a LiDAR sector.

    Args:
        scan_msg: LaserScan message.
        angle_min_rad: Minimum angle of sector (rad, robot frame).
        angle_max_rad: Maximum angle of sector (rad, robot frame).

    Returns:
        float: Minimum range in the sector, or inf if no valid reading.
    """
    if scan_msg is None:
        return float('inf')

    ranges = scan_msg.ranges
    angle_min_scan = scan_msg.angle_min
    angle_increment = scan_msg.angle_increment
    min_range = float('inf')

    for i, r in enumerate(ranges):
        if math.isnan(r) or math.isinf(r) or r <= 0.0:
            continue
        angle = angle_min_scan + i * angle_increment
        angle = normalize_angle(angle)
        if angle_min_rad <= angle <= angle_max_rad:
            if r < min_range:
                min_range = r

    return min_range


def scan_to_cartesian(scan_msg, min_range=0.12, max_range=3.5):
    """Convert LaserScan ranges to list of (x, y) points in robot frame.

    Args:
        scan_msg: LaserScan message.
        min_range: Minimum valid range.
        max_range: Maximum valid range.

    Returns:
        list[tuple[float, float]]: List of (x, y) points.
    """
    points = []
    if scan_msg is None:
        return points

    angle = scan_msg.angle_min
    for r in scan_msg.ranges:
        if min_range < r < max_range and not math.isnan(r):
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y))
        angle += scan_msg.angle_increment

    return points


def transform_point_to_map(px, py, robot_x, robot_y, robot_yaw):
    """Transform a point from robot frame to map frame.

    Args:
        px, py: Point in robot frame.
        robot_x, robot_y, robot_yaw: Robot pose in map frame.

    Returns:
        tuple[float, float]: Point in map frame.
    """
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    mx = robot_x + px * cos_yaw - py * sin_yaw
    my = robot_y + px * sin_yaw + py * cos_yaw
    return (mx, my)
