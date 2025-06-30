import datetime
import rospy
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import message_filters
import tf2_ros
from tf.transformations import quaternion_matrix
import threading

from Config import Config
# from utils import get_transform_matrix
class GocatorInterface:
    def __init__(self, cfg: Config):

        self.cfg = cfg.sensor

        rospy.init_node(self.cfg.ros_node_name, anonymous=True)
        self.lock = threading.Lock()
        self.pcd = []
        self.is_running = False

        # Subscribe to point cloud
        pcd_sub = message_filters.Subscriber(self.cfg.pcd_topic, PointCloud2)
        self.sync = message_filters.TimeSynchronizer([pcd_sub], queue_size=10)
        self.sync.registerCallback(self.callback)


        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("Listening to /gocator_profile_pcd...")

        self.is_save = True

    def _on_shutdown(self):
        rospy.loginfo("Shutting down...")

    def callback(self, msg: PointCloud2):
        try:
            if self.is_running:
                cloud = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)
                self.pcd.append(cloud)

        except Exception as e:
            rospy.logerr(f"[callback] Error: {e}")

    def start(self):
        """
        Enable point cloud collection. Does not reset collected data.
        """
        with self.lock:
            self.is_running = True

    def stop(self):
        """
        Disable point cloud collection and return collected data. Resets self.pcd.
        """
        with self.lock:
            self.is_running = False
            result = self.pcd.copy()
            self.pcd = []
        # Save as npy (list of arrays)
        # now_str = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
        # np.save(f"gocator_collected_pcd_list_{now_str}.npy", result, allow_pickle=True)
        return result

    def shutdown(self):
        """
        Cleanly shutdown the collector and ROS node.
        """
        with self.lock:
            self.is_running = False
        rospy.signal_shutdown("GocatorPCDCollector shutdown called.")

    def get_current_data(self):
        """
        Get the latest collected point cloud data.
        """
        with self.lock:
            return self.cloud, self.robot_frame