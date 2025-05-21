#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from carla_global_planner.srv import PlanGlobalPath
from utilities.planner import compute_route_waypoints

import carla
import random
from tf_transformations import quaternion_from_euler


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('carla_global_planner_node')

        # Carla client setup
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()

        self.marker_pub = self.create_publisher(
            Marker, 'visualization_marker', 10)

        # ROS 2 service
        self.srv = self.create_service(
            PlanGlobalPath, 'plan_to_random_goal', self.plan_path_cb)

        print("CARLA global planner service has started")

    def plan_path_cb(self, request, response):
        start_odom = request.start
        start_location = carla.Location(
            x=start_odom.pose.pose.position.x,
            y=-start_odom.pose.pose.position.y,
            z=start_odom.pose.pose.position.z
        )
        start_wp = self.map.get_waypoint(start_location)
        # Plan Route
        num_waypoints = 0
        while num_waypoints < 50:
            # Choose random navigable goal location
            spawn_points = self.map.get_spawn_points()
            goal_transform = random.choice(spawn_points)
            goal_location = goal_transform.location
            goal_wp = self.map.get_waypoint(goal_location)

            self.get_logger().info(
                f'Planning from {start_location} to {goal_location}')

            route = compute_route_waypoints(
                self.map, start_wp, goal_wp, resolution=1.0)
            num_waypoints = len(route)

        # Build Path msg
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for waypoint, _ in route:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = waypoint.transform.location.x
            pose.pose.position.y = -waypoint.transform.location.y
            pose.pose.position.z = waypoint.transform.location.z

            yaw = waypoint.transform.rotation.yaw * (3.1415 / 180.0)
            q = quaternion_from_euler(0, 0, -yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path_msg.poses.append(pose)

        # Delete old marker
        delete_marker = self.delete_previous_marker(path_msg.header.stamp)
        self.marker_pub.publish(delete_marker)

        # Publish new marker
        new_marker = self.create_path_marker(path_msg)
        self.marker_pub.publish(new_marker)

        response.path = path_msg

        return response

    def create_path_marker(self, path_msg):
        marker = Marker()
        marker.header.frame_id = path_msg.header.frame_id
        marker.header.stamp = path_msg.header.stamp
        marker.ns = "carla_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Visual appearance
        marker.scale.x = 0.6  # line width
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Add all poses to the marker
        for pose_stamped in path_msg.poses:
            pt = Point()
            pt.x = pose_stamped.pose.position.x
            pt.y = pose_stamped.pose.position.y
            pt.z = pose_stamped.pose.position.z
            marker.points.append(pt)

        return marker

    def delete_previous_marker(self, time):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = time
        marker.ns = "carla_path"
        marker.id = 0
        marker.action = Marker.DELETE

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
