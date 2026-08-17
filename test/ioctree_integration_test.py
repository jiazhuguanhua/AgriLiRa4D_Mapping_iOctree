#!/usr/bin/env python3

import struct
import threading
import unittest

import rospy
import rostest
from octomap_msgs.msg import Octomap
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def make_lidar_cloud():
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1),
        PointField("ring", 16, PointField.UINT16, 1),
        PointField("timestamp", 18, PointField.FLOAT64, 1),
    ]
    points = [
        (1.0, 0.0, 0.0, 10.0, 0, 10.00),
        (2.0, 0.0, 0.0, 20.0, 0, 10.04),
        (3.0, 0.0, 0.0, 30.0, 0, 10.08),
        (4.0, 0.0, 0.0, 40.0, 0, 10.12),
    ]
    data = b"".join(struct.pack("<ffffHd", *point) for point in points)
    return PointCloud2(
        header=Header(stamp=rospy.Time.from_sec(1.0), frame_id="lidar"),
        height=1,
        width=len(points),
        fields=fields,
        is_bigendian=False,
        point_step=26,
        row_step=26 * len(points),
        data=data,
        is_dense=True,
    )


def make_radar_cloud():
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("v_doppler_mps", 12, PointField.FLOAT32, 1),
        PointField("snr_db", 16, PointField.FLOAT32, 1),
        PointField("rcs", 20, PointField.FLOAT32, 1),
    ]
    point = (1.0, 1.0, 0.0, 0.0, 20.0, 1.0)
    return PointCloud2(
        header=Header(stamp=rospy.Time.from_sec(1.1), frame_id="radar"),
        height=1,
        width=1,
        fields=fields,
        is_bigendian=False,
        point_step=24,
        row_step=24,
        data=struct.pack("<ffffff", *point),
        is_dense=True,
    )


class IOctreePipelineIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.map_message = None
        self.map_event = threading.Event()
        self.map_sub = rospy.Subscriber(
            "/occupied_voxels", Octomap, self._map_callback, queue_size=1
        )
        self.lidar_pub = rospy.Publisher(
            "/test/rslidar_points", PointCloud2, queue_size=1, latch=True
        )
        self.radar_pub = rospy.Publisher(
            "/test/radar_points", PointCloud2, queue_size=1, latch=True
        )

    def _map_callback(self, message):
        self.map_message = message
        self.map_event.set()

    def test_pipeline_publishes_occupied_voxels(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.lidar_pub.get_num_connections() and self.radar_pub.get_num_connections():
                break
            rospy.sleep(0.05)

        self.assertGreater(self.lidar_pub.get_num_connections(), 0)
        self.assertGreater(self.radar_pub.get_num_connections(), 0)
        self.lidar_pub.publish(make_lidar_cloud())
        rospy.sleep(0.1)
        self.radar_pub.publish(make_radar_cloud())

        self.assertTrue(self.map_event.wait(5.0), "timed out waiting for i-Octree map")
        self.assertIsNotNone(self.map_message)
        self.assertEqual(self.map_message.header.frame_id, "world")
        self.assertFalse(self.map_message.binary)
        self.assertEqual(self.map_message.id, "OcTree")
        self.assertGreater(len(self.map_message.data), 0)


if __name__ == "__main__":
    rospy.init_node("ioctree_pipeline_integration_test")
    rostest.rosrun(
        "mapping", "ioctree_pipeline_integration", IOctreePipelineIntegrationTest
    )
