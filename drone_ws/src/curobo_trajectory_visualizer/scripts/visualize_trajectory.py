#!/usr/bin/env python3

import colorsys

import h5py
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__("trajectory_visualizer")

        dataset_path = self.declare_parameter("dataset", "").value
        trajectory_index = self.declare_parameter("trajectory_index", 0).value
        num_trajectories = self.declare_parameter("num_trajectories", 0).value

        if num_trajectories < 0:
            raise ValueError("num_trajectories must be nonnegative (0 displays all)")

        markers = []
        with h5py.File(dataset_path) as dataset:
            if not 0 <= trajectory_index < len(dataset["goals"]):
                raise IndexError(f"Dataset contains {len(dataset['goals'])} trajectories")
            frame_id = str(dataset.attrs.get("frame_id", "world"))
            stop = (len(dataset["goals"]) if num_trajectories == 0 else
                    min(trajectory_index + num_trajectories, len(dataset["goals"])))

            for index in range(trajectory_index, stop):
                begin, end = dataset["trajectory_offsets"][index : index + 2]
                trajectory = dataset["trajectory_points"][begin:end]
                points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in trajectory]
                color = colorsys.hsv_to_rgb((index - trajectory_index) / (stop - trajectory_index), 0.8, 1.0)

                line = Marker()
                line.header.frame_id = frame_id
                line.ns = "trajectory"
                line.id = index * 2
                line.type = Marker.LINE_STRIP
                line.pose.orientation.w = 1.0
                line.scale.x = 0.08
                line.color.r, line.color.g, line.color.b = color
                line.color.a = 1.0
                line.points = points

                waypoints = Marker()
                waypoints.header.frame_id = frame_id
                waypoints.ns = "trajectory"
                waypoints.id = index * 2 + 1
                waypoints.type = Marker.SPHERE_LIST
                waypoints.pose.orientation.w = 1.0
                waypoints.scale.x = waypoints.scale.y = waypoints.scale.z = 0.15
                waypoints.color.r, waypoints.color.g, waypoints.color.b = color
                waypoints.color.a = 1.0
                waypoints.points = points
                markers.extend([line, waypoints])

        self.markers = MarkerArray(markers=markers)
        self.publisher = self.create_publisher(MarkerArray, "/curobo_trajectory", 1)
        self.timer = self.create_timer(1.0, self.publish_trajectory)
        self.get_logger().info(f"Displaying trajectories {trajectory_index} through {stop - 1}")

    def publish_trajectory(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.markers)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(TrajectoryVisualizer())
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
