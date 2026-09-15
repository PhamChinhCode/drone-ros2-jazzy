"""landing_target_bridge_node - node DUY NHAT duoc phep khang dinh "dang bam dung marker".

Tach trach nhiem xac thuc ID khoi marker_detector_node (chi bao moi ID thay duoc) va khoi
mission_manager_node (khong nen tu parse pose tho).

Nguyen tac bat buoc: mat marker thi NGUNG PUBLISH ngay - khong noi suy, khong giu gia tri cu.
FC bam vao mot vi tri cu da khong con dung con nguy hiem hon la khong co du lieu.

Pose tag lay qua TF (apriltag_ros phat khung anh -> ten trong tag.frames), tra TF MOI NHAT va dung
stamp cua chinh TF do: khi CPU day tai /tf toi sau ban tin detections >100 ms (do 09-14, xem
marker_pose_republisher_node). Ten khung theo ID lay tu config/tags.yaml.
"""

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from rclpy.exceptions import ParameterUninitializedException
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import Bool, Int32
from tf2_ros import Buffer, TransformException, TransformListener

from drone_control.qos import EVENT_QOS, SENSOR_QOS
from drone_control.target_tracker import TargetTracker


class LandingTargetBridgeNode(Node):

    def __init__(self):
        super().__init__('landing_target_bridge_node')

        # timeout_s: qua dai -> FC bam vi tri cu da sai; qua ngan -> bao mat bam gia khi rung
        # mot khung hinh. 0.5-1.0 s tuy toc do ha canh.
        self.declare_parameter('timeout_s', 0.7)
        self.declare_parameter('publish_rate_hz', 25.0)     # 20-30 Hz trong pha bam marker
        self.declare_parameter('target_frame', 'base_link')
        # Ban do tag dung chung (config/tags.yaml): chi can ID -> ten khung TF.
        self.declare_parameter('known_tags', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('tag_frames', Parameter.Type.STRING_ARRAY)

        try:
            flat = self.get_parameter('known_tags').value
            frames = self.get_parameter('tag_frames').value
            ids = [int(flat[i]) for i in range(0, len(flat), 4)]
            if len(ids) != len(frames):
                raise ValueError(f'tag_frames co {len(frames)} ten, known_tags co {len(ids)} tag')
            self.tag_frames = dict(zip(ids, frames))
        except ParameterUninitializedException:
            self.tag_frames = {}
            self.get_logger().error('chua nap config/tags.yaml - khong bam duoc tag nao')

        self.expected_id = -1
        self.tracker = TargetTracker(self.get_parameter('timeout_s').value)
        self.last_tf_stamp = None

        self.tf_buffer = Buffer()
        # spin_thread=True: buffer TF van cap nhat khi executor chinh ban callback (CPU day tai).
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections', self.on_detections, SENSOR_QOS)
        self.create_subscription(
            Int32, '/mission/expected_marker_id', self.on_expected_id, EVENT_QOS)

        # Topic NOI BO cho position_controller_node. KHONG gui LANDING_TARGET len FC: ban tin 149
        # phe bo tu giao uoc 1.4 - Pi tu dong vong van toc theo marker (11.1 #12, #13c).
        self.pub_target = self.create_publisher(PoseStamped, '/landing_target/pose', SENSOR_QOS)
        self.pub_lost = self.create_publisher(Bool, '/landing_target/lost', EVENT_QOS)

        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self.check_timeout)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_expected_id(self, msg):
        """Doi bai dap thi reset dong ho mat bam, tranh bao mat bam gia ngay sau khi doi ID."""
        if msg.data == self.expected_id:
            return
        self.publish_event(self.tracker.reset())     # bao mat bam theo ID CU truoc khi doi
        self.expected_id = msg.data
        self.last_tf_stamp = None
        if msg.data >= 0 and msg.data not in self.tag_frames:
            self.get_logger().warning(f'tag {msg.data} khong co trong config/tags.yaml')

    def on_detections(self, msg):
        """Chi xu ly detection co ID == expected_id; chuyen pose tag sang target_frame qua TF."""
        frame = self.tag_frames.get(self.expected_id)
        if frame is None or not any(d.id == self.expected_id for d in msg.detections):
            return
        target = self.get_parameter('target_frame').value
        try:
            t = self.tf_buffer.lookup_transform(target, frame, Time())
        except TransformException:
            return
        stamp = Time.from_msg(t.header.stamp)
        age_s = self.now_s() - stamp.nanoseconds / 1e9
        if stamp == self.last_tf_stamp or age_s > self.tracker.timeout_s:
            return      # TF cu hoac da dung - khong phat lai gia tri cu (nguyen tac o dau file)
        self.last_tf_stamp = stamp

        out = PoseStamped()
        out.header.stamp = t.header.stamp
        out.header.frame_id = target
        out.pose.position.x = t.transform.translation.x
        out.pose.position.y = t.transform.translation.y
        out.pose.position.z = t.transform.translation.z
        out.pose.orientation = t.transform.rotation
        self.pub_target.publish(out)
        self.publish_event(self.tracker.seen(self.now_s()))

    def check_timeout(self):
        """Qua timeout_s khong thay -> /landing_target/lost = true dung mot lan."""
        self.publish_event(self.tracker.check(self.now_s()))

    def publish_event(self, event):
        if event is None:
            return
        self.pub_lost.publish(Bool(data=(event == 'lost')))
        text = f'tag {self.expected_id}: {"MAT BAM" if event == "lost" else "bat duoc"}'
        if event == 'lost':
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = LandingTargetBridgeNode()
    # EventsExecutor: tren Pi 4 executor mac dinh cua rclpy ton phan lon CPU de dung lai wait-set
    # moi lan thuc day (do 09-14: mission_manager_node 45-50 % -> 13,5 %).
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
