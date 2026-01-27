#!/usr/bin/env python3
"""
Publishes collision objects from Gazebo to the MoveIt planning scene.
Uses subprocess to read Gazebo poses directly (workaround for ros_gz_bridge issue #172).
"""

import re
import subprocess
import threading
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene


class ScenePublisher(Node):
    """Node that publishes Gazebo objects to the MoveIt planning scene."""

    def __init__(self):
        super().__init__('scene_publisher')

        self.declare_parameter('frame_id', 'world')
        self.frame_id = self.get_parameter('frame_id').value

        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        self.dynamic_objects = {
            'red_box': ('box', [0.15, 0.15, 0.15]),
            'blue_cylinder': ('cylinder', [0.21, 0.075]),  # height, radius
        }

        self.object_poses = {}
        self._lock = threading.Lock()

        # Timer to publish updates
        self.timer = self.create_timer(0.1, self.publish_dynamic_objects)

        # Add static objects after delay
        self.static_timer = self.create_timer(3.0, self.add_static_objects)
        self.static_added = False

        # Start Gazebo pose listener thread
        self.gz_thread = threading.Thread(target=self.gz_pose_listener, daemon=True)
        self.gz_thread.start()

        self.get_logger().info('Scene publisher initialized - listening to Gazebo directly')

    def gz_pose_listener(self):
        """Listen to Gazebo pose topic using subprocess."""
        try:
            proc = subprocess.Popen(
                ['gz', 'topic', '-e', '-t', '/world/tabletop_world/dynamic_pose/info'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )

            current_pose = {}
            current_name = None

            for line in proc.stdout:
                line = line.strip()

                # Parse pose messages
                if 'name:' in line:
                    match = re.search(r'name:\s*"([^"]+)"', line)
                    if match:
                        current_name = match.group(1)
                        current_pose = {'name': current_name}

                elif current_name and 'x:' in line and 'position' not in line.lower():
                    match = re.search(r'x:\s*([-\d.e]+)', line)
                    if match and 'pos_x' not in current_pose:
                        current_pose['pos_x'] = float(match.group(1))
                    elif match and 'pos_x' in current_pose:
                        current_pose['ori_x'] = float(match.group(1))

                elif current_name and 'y:' in line:
                    match = re.search(r'y:\s*([-\d.e]+)', line)
                    if match and 'pos_y' not in current_pose:
                        current_pose['pos_y'] = float(match.group(1))
                    elif match and 'pos_y' in current_pose:
                        current_pose['ori_y'] = float(match.group(1))

                elif current_name and 'z:' in line:
                    match = re.search(r'z:\s*([-\d.e]+)', line)
                    if match and 'pos_z' not in current_pose:
                        current_pose['pos_z'] = float(match.group(1))
                    elif match and 'pos_z' in current_pose:
                        current_pose['ori_z'] = float(match.group(1))

                elif current_name and 'w:' in line:
                    match = re.search(r'w:\s*([-\d.e]+)', line)
                    if match:
                        current_pose['ori_w'] = float(match.group(1))

                        # Complete pose received, update if it's one of our objects
                        name = current_pose.get('name', '')
                        for obj_name in self.dynamic_objects.keys():
                            if obj_name == name:
                                pose = Pose()
                                pose.position.x = current_pose.get('pos_x', 0.0)
                                pose.position.y = current_pose.get('pos_y', 0.0)
                                pose.position.z = current_pose.get('pos_z', 0.0)
                                pose.orientation.x = current_pose.get('ori_x', 0.0)
                                pose.orientation.y = current_pose.get('ori_y', 0.0)
                                pose.orientation.z = current_pose.get('ori_z', 0.0)
                                pose.orientation.w = current_pose.get('ori_w', 1.0)

                                with self._lock:
                                    old = self.object_poses.get(obj_name)
                                    if old:
                                        dx = abs(pose.position.x - old.position.x)
                                        dy = abs(pose.position.y - old.position.y)
                                        dz = abs(pose.position.z - old.position.z)
                                        if dx > 0.01 or dy > 0.01 or dz > 0.01:
                                            self.get_logger().info(
                                                f'{obj_name}: ({old.position.x:.3f}, {old.position.y:.3f}, {old.position.z:.3f}) -> '
                                                f'({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
                                            )
                                    self.object_poses[obj_name] = pose
                                break

                        current_name = None
                        current_pose = {}

        except Exception as e:
            self.get_logger().error(f'Gazebo listener error: {e}')

    def add_static_objects(self):
        """Add static collision objects (table with legs)."""
        if self.static_added:
            return

        scene = PlanningScene()
        scene.is_diff = True

        table = CollisionObject()
        table.header.frame_id = self.frame_id
        table.header.stamp = self.get_clock().now().to_msg()
        table.id = 'table'
        table.operation = CollisionObject.ADD

        # Table top
        table_top = SolidPrimitive()
        table_top.type = SolidPrimitive.BOX
        table_top.dimensions = [0.8, 1.0, 0.04]
        table_top_pose = Pose()
        table_top_pose.position = Point(x=0.5, y=0.0, z=0.4)
        table_top_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        table.primitives.append(table_top)
        table.primitive_poses.append(table_top_pose)

        # Table legs
        for x, y, z in [(0.85, 0.45, 0.19), (0.85, -0.45, 0.19),
                        (0.15, 0.45, 0.19), (0.15, -0.45, 0.19)]:
            leg = SolidPrimitive()
            leg.type = SolidPrimitive.BOX
            leg.dimensions = [0.04, 0.04, 0.38]
            leg_pose = Pose()
            leg_pose.position = Point(x=x, y=y, z=z)
            leg_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            table.primitives.append(leg)
            table.primitive_poses.append(leg_pose)

        scene.world.collision_objects.append(table)
        self.scene_pub.publish(scene)

        self.static_added = True
        self.get_logger().info('Added table to planning scene')

        # Add initial poses for dynamic objects
        self.add_initial_dynamic_objects()

    def add_initial_dynamic_objects(self):
        """Add dynamic objects at their initial positions."""
        initial_poses = {
            'red_box': (0.4, 0.2, 0.495),       # box center height = table + half height
            'blue_cylinder': (0.6, -0.2, 0.525),  # cylinder center height
        }

        with self._lock:
            for name, (x, y, z) in initial_poses.items():
                if name not in self.object_poses:
                    pose = Pose()
                    pose.position = Point(x=x, y=y, z=z)
                    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    self.object_poses[name] = pose

        self.get_logger().info('Added dynamic objects to planning scene')

    def publish_dynamic_objects(self):
        """Publish current dynamic object poses to the planning scene."""
        with self._lock:
            if not self.object_poses:
                return

            scene = PlanningScene()
            scene.is_diff = True

            for name, (obj_type, dims) in self.dynamic_objects.items():
                if name not in self.object_poses:
                    continue

                obj = CollisionObject()
                obj.id = name
                obj.header.frame_id = self.frame_id
                obj.header.stamp = self.get_clock().now().to_msg()
                obj.operation = CollisionObject.ADD

                primitive = SolidPrimitive()
                if obj_type == 'box':
                    primitive.type = SolidPrimitive.BOX
                elif obj_type == 'cylinder':
                    primitive.type = SolidPrimitive.CYLINDER
                elif obj_type == 'sphere':
                    primitive.type = SolidPrimitive.SPHERE
                primitive.dimensions = list(dims)

                obj.primitives.append(primitive)
                obj.primitive_poses.append(self.object_poses[name])
                scene.world.collision_objects.append(obj)

            self.scene_pub.publish(scene)


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
