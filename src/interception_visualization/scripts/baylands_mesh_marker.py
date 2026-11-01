#!/usr/bin/env python3

import copy
import math
import os
import re
import shutil

import rospy
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker


class BaylandsMeshMarkerNode:
    def __init__(self):
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/rviz/baylands_mesh"
        )
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.gazebo_model_states_topic = rospy.get_param(
            "~gazebo_model_states_topic", ""
        )
        self.gazebo_model_name = rospy.get_param(
            "~gazebo_model_name", "baylands"
        )
        self.gazebo_pose = None
        self.model_dir = os.path.expanduser(
            rospy.get_param(
                "~model_dir", "~/.gazebo/models/holydrone_baylands"
            )
        )
        self.cache_dir = os.path.expanduser(
            rospy.get_param("~cache_dir", "/tmp/bvr_baylands_rviz")
        )
        self.origin_x = float(rospy.get_param("~origin_x", 0.0))
        self.origin_y = float(rospy.get_param("~origin_y", 0.0))
        self.origin_z = float(
            rospy.get_param("~origin_z", rospy.get_param("~z_offset", -1.3))
        )
        self.origin_roll = float(rospy.get_param("~origin_roll", 0.0))
        self.origin_pitch = float(rospy.get_param("~origin_pitch", 0.0))
        self.origin_yaw = float(rospy.get_param("~origin_yaw", 0.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 0.2))
        self.meshes = [
            "media/baylands_01.DAE",
            "media/baylands_02.DAE",
            "media/baylands_03.DAE",
        ]

        self.publisher = rospy.Publisher(
            self.marker_topic, Marker, queue_size=3, latch=True
        )
        if self.gazebo_model_states_topic:
            rospy.Subscriber(
                self.gazebo_model_states_topic,
                ModelStates,
                self.model_states_callback,
                queue_size=1,
            )
        self.warned_missing = False
        self.prepare_rviz_meshes()
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_rate, 1e-3)),
            self.publish_markers,
        )
        self.publish_markers(None)

    def model_states_callback(self, msg):
        try:
            index = msg.name.index(self.gazebo_model_name)
        except ValueError:
            rospy.logwarn_throttle(
                2.0,
                "Baylands mesh marker: Gazebo model '%s' not found"
                % self.gazebo_model_name,
            )
            return
        self.gazebo_pose = copy.deepcopy(msg.pose[index])
        self.publish_markers(None)

    @staticmethod
    def sanitize_collada_text(text):
        # RViz/Assimp in Noetic rejects a few self-closing metadata tags found
        # in the Gazebo Baylands DAE files, although Gazebo itself accepts them.
        for tag in ("author", "comments", "keywords", "revision", "subject", "title"):
            text = re.sub(r"<{}\s*/>".format(tag), "<{0}></{0}>".format(tag), text)
        return text

    def prepare_rviz_meshes(self):
        os.makedirs(os.path.join(self.cache_dir, "media"), exist_ok=True)
        texture_src = os.path.join(self.model_dir, "media", "Textures")
        texture_dst = os.path.join(self.cache_dir, "media", "Textures")
        # os.path.exists() is false for a dangling symlink, even though the
        # directory entry still exists and would make os.symlink() fail. This
        # can happen when the bundled model directory is moved or renamed.
        if (
            os.path.islink(texture_dst)
            and os.path.realpath(texture_dst) != os.path.realpath(texture_src)
        ):
            os.unlink(texture_dst)
        if os.path.exists(texture_src) and not os.path.lexists(texture_dst):
            os.symlink(texture_src, texture_dst)

        for relative_path in self.meshes:
            src = os.path.join(self.model_dir, relative_path)
            dst = os.path.join(self.cache_dir, relative_path)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            if not os.path.exists(src):
                continue
            try:
                with open(src, "r", encoding="utf-8", errors="replace") as src_file:
                    text = src_file.read()
                with open(dst, "w", encoding="utf-8") as dst_file:
                    dst_file.write(self.sanitize_collada_text(text))
            except Exception as exc:
                rospy.logwarn(
                    "Baylands mesh marker: failed to sanitize %s: %s. "
                    "Falling back to original mesh.",
                    src,
                    exc,
                )
                shutil.copyfile(src, dst)

    def mesh_uri(self, relative_path):
        cached_path = os.path.join(self.cache_dir, relative_path)
        if os.path.exists(cached_path):
            return "file://" + cached_path
        return "file://" + os.path.join(self.model_dir, relative_path)

    @staticmethod
    def quaternion_from_rpy(roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def publish_markers(self, _event):
        if self.gazebo_model_states_topic and self.gazebo_pose is None:
            return
        stamp = rospy.Time.now()
        missing = [
            path for path in self.meshes
            if not os.path.exists(os.path.join(self.model_dir, path))
        ]
        if missing:
            if not self.warned_missing:
                rospy.logwarn(
                    "Baylands mesh marker: missing mesh files under %s: %s",
                    self.model_dir,
                    ", ".join(missing),
                )
                self.warned_missing = True
            return

        qx, qy, qz, qw = self.quaternion_from_rpy(
            self.origin_roll, self.origin_pitch, self.origin_yaw
        )
        for marker_id, mesh_path in enumerate(self.meshes):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.frame_id
            marker.ns = "baylands"
            marker.id = marker_id
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            if self.gazebo_pose is not None:
                # Use the authoritative model pose from the running world so
                # RViz follows any translation/rotation declared by its SDF.
                marker.pose = copy.deepcopy(self.gazebo_pose)
            else:
                marker.pose.position.x = self.origin_x
                marker.pose.position.y = self.origin_y
                marker.pose.position.z = self.origin_z
                marker.pose.orientation.x = qx
                marker.pose.orientation.y = qy
                marker.pose.orientation.z = qz
                marker.pose.orientation.w = qw
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.mesh_resource = self.mesh_uri(mesh_path)
            marker.mesh_use_embedded_materials = True
            marker.lifetime = rospy.Duration(0.0)
            self.publisher.publish(marker)


if __name__ == "__main__":
    rospy.init_node("baylands_mesh_marker")
    BaylandsMeshMarkerNode()
    rospy.spin()
