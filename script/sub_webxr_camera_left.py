#!/usr/bin/env python3

import os
import threading

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage

try:
    import cv2
    import numpy as np
    HAS_CV = True
except ImportError:
    cv2 = None
    np = None
    HAS_CV = False


class CameraLeftSubscriber:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_left = None
        self.latest_left_recv_time = None
        self.left_update_count = 0
        self.camera_count = 0

        self.left_topic = rospy.get_param(
            '~left_position_topic',
            '/webxr/controller_state/left_con/position'
        )
        self.camera_topic = rospy.get_param(
            '~camera_topic',
            '/hmd/camera/compressed'
        )
        self.stale_warn_sec = float(rospy.get_param('~stale_warn_sec', 1.0))
        self.show_image = bool(rospy.get_param('~show_image', True))
        self.window_name = rospy.get_param('~window_name', 'webxr camera')
        self.display_env = os.environ.get('DISPLAY', '')
        self.debug_dump_path = rospy.get_param('~debug_dump_path', '')
        self.debug_dump_interval = int(rospy.get_param('~debug_dump_interval', 30))

        if self.show_image and not HAS_CV:
            rospy.logwarn('~show_image is true but cv2/numpy are unavailable. image display is disabled.')
            self.show_image = False
        if self.show_image and not self.display_env:
            rospy.logwarn('DISPLAY is not set. image display is disabled in this environment.')
            self.show_image = False
        if self.show_image:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        rospy.Subscriber(self.left_topic, Point, self.on_left_position, queue_size=20)
        rospy.Subscriber(self.camera_topic, CompressedImage, self.on_camera_image, queue_size=2)

        rospy.loginfo('subscribed left position: %s', self.left_topic)
        rospy.loginfo('subscribed camera image: %s', self.camera_topic)
        rospy.loginfo('show_image=%s window_name=%s', self.show_image, self.window_name)
        if self.debug_dump_path:
            rospy.loginfo('debug dump enabled: path=%s interval=%d', self.debug_dump_path, self.debug_dump_interval)

    def on_left_position(self, msg):
        recv_time = rospy.Time.now()
        with self.lock:
            self.latest_left = (msg.x, msg.y, msg.z)
            self.latest_left_recv_time = recv_time
            self.left_update_count += 1

        rospy.loginfo_throttle(
            2.0,
            'left updates=%d latest=(%.3f, %.3f, %.3f)',
            self.left_update_count,
            msg.x,
            msg.y,
            msg.z,
        )

    def on_camera_image(self, msg):
        recv_time = rospy.Time.now()
        with self.lock:
            left = self.latest_left
            left_time = self.latest_left_recv_time
            left_count = self.left_update_count

        self.camera_count += 1

        if self.show_image:
            self.render_camera_image(msg, left)
        if self.debug_dump_path and self.debug_dump_interval > 0 and self.camera_count % self.debug_dump_interval == 0:
            self.dump_camera_image(msg)

        if left is None or left_time is None:
            rospy.logwarn_throttle(2.0, 'camera received but left position is not available yet')
            return

        age_ms = (recv_time - left_time).to_sec() * 1000.0
        if age_ms > self.stale_warn_sec * 1000.0:
            rospy.logwarn_throttle(
                1.0,
                'left position is stale: age=%.1fms updates=%d (camera keeps coming)',
                age_ms,
                left_count,
            )

        rospy.loginfo(
            'pair #%d camera_bytes=%d format=%s left=(%.3f, %.3f, %.3f) left_age=%.1fms left_updates=%d',
            self.camera_count,
            len(msg.data),
            msg.format,
            left[0],
            left[1],
            left[2],
            age_ms,
            left_count,
        )

    def render_camera_image(self, msg, left):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn_throttle(2.0, 'failed to decode compressed image')
            return

        if left is None:
            overlay = 'left=(n/a) frame=%d' % (self.camera_count,)
        else:
            overlay = 'left=(%.3f, %.3f, %.3f) frame=%d' % (
                left[0],
                left[1],
                left[2],
                self.camera_count,
            )
        cv2.putText(frame, overlay, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def shutdown(self):
        if self.show_image and HAS_CV:
            cv2.destroyAllWindows()

    def dump_camera_image(self, msg):
        try:
            with open(self.debug_dump_path, 'wb') as f:
                f.write(msg.data)
            rospy.loginfo('dumped jpeg: %s (bytes=%d frame=%d)', self.debug_dump_path, len(msg.data), self.camera_count)
        except Exception as e:
            rospy.logwarn_throttle(2.0, 'failed to dump jpeg: %s', str(e))


if __name__ == '__main__':
    rospy.init_node('subscribe_webxr_camera_left')
    node = CameraLeftSubscriber()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()