import time

from geometry_msgs.msg import PoseArray
import omni.kit.app
import omni.usd
from pxr import Gf, UsdGeom
import rclpy
from rclpy.node import Node


TOPIC_NAME = "/retarget_keypoint_points"
ROOT_PRIM = "/World/Retarget_Keypoints"
SPHERE_RADIUS = 0.045
LINE_WIDTH = 0.025
PRINT_STATS = True

KEYPOINT_NAMES = (
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
)

SKELETON_12 = (
    (0, 1),
    (0, 2),
    (2, 4),
    (1, 3),
    (3, 5),
    (0, 6),
    (1, 7),
    (6, 7),
    (6, 8),
    (8, 10),
    (7, 9),
    (9, 11),
)


def pose_to_isaac(pose):
    position = pose.position
    return Gf.Vec3d(position.x, position.y, position.z)


class RetargetKeypointVisualizer(Node):
    def __init__(self):
        self.instance_id = int(time.time() * 1000)
        super().__init__(f"retarget_keypoint_visualizer_{self.instance_id}")

        self.stage = omni.usd.get_context().get_stage()
        self.joint_xforms = []
        self.bone_curves = []
        self.latest_msg = None
        self.has_new_msg = False
        self.last_stats_time = 0.0
        self.received_count = 0
        self.stopped = False

        self._create_spheres()
        self._create_bones()

        self.subscription = self.create_subscription(
            PoseArray,
            TOPIC_NAME,
            self.pose_callback,
            10,
        )

        self.update_subscription = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(
                self.on_update,
                name=f"retarget_keypoint_visualizer_{self.instance_id}",
            )
        )

        print(
            f"Listening to {TOPIC_NAME}; updating "
            f"{len(KEYPOINT_NAMES)} retarget keypoints."
        )

    def _create_spheres(self):
        UsdGeom.Xform.Define(self.stage, ROOT_PRIM)

        for index, name in enumerate(KEYPOINT_NAMES):
            path = f"{ROOT_PRIM}/joint_{index:02d}_{name}"
            sphere = UsdGeom.Sphere.Define(self.stage, path)
            sphere.CreateRadiusAttr(SPHERE_RADIUS)
            sphere.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.55, 0.1)])
            xform = UsdGeom.XformCommonAPI(sphere)
            xform.SetTranslate(
                Gf.Vec3d(index * SPHERE_RADIUS * 2.5, 0.0, 1.0)
            )
            self.joint_xforms.append(xform)

    def _create_bones(self):
        initial_point = Gf.Vec3f(0.0, 0.0, 1.0)

        for index, (start_joint, end_joint) in enumerate(SKELETON_12):
            start_name = KEYPOINT_NAMES[start_joint]
            end_name = KEYPOINT_NAMES[end_joint]
            path = f"{ROOT_PRIM}/bone_{index:02d}_{start_name}_{end_name}"
            curve = UsdGeom.BasisCurves.Define(self.stage, path)
            curve.CreateTypeAttr("linear")
            curve.CreateCurveVertexCountsAttr([2])
            curve.CreatePointsAttr([initial_point, initial_point])
            curve.CreateWidthsAttr([LINE_WIDTH])
            curve.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.75, 0.1)])
            self.bone_curves.append(curve)

    def pose_callback(self, msg):
        self.latest_msg = msg
        self.has_new_msg = True
        self.received_count += 1

    def on_update(self, event):
        if self.stopped:
            return

        rclpy.spin_once(self, timeout_sec=0.0)

        if self.latest_msg is None or not self.has_new_msg:
            self._print_waiting_stats()
            return

        self.has_new_msg = False
        poses = self.latest_msg.poses
        count = min(len(poses), len(self.joint_xforms))

        for index in range(count):
            isaac_position = pose_to_isaac(poses[index])
            self.joint_xforms[index].SetTranslate(isaac_position)

        self._update_bones(poses, count)
        self._print_pose_stats(poses)

    def _update_bones(self, poses, count):
        for curve, (start_joint, end_joint) in zip(
                self.bone_curves,
                SKELETON_12,
        ):
            if start_joint >= count or end_joint >= count:
                continue

            start = pose_to_isaac(poses[start_joint])
            end = pose_to_isaac(poses[end_joint])
            curve.GetPointsAttr().Set([
                Gf.Vec3f(start[0], start[1], start[2]),
                Gf.Vec3f(end[0], end[1], end[2]),
            ])

    def _print_waiting_stats(self):
        if not PRINT_STATS:
            return

        now = time.time()
        if now - self.last_stats_time < 2.0:
            return

        self.last_stats_time = now
        print(
            f"Waiting for {TOPIC_NAME}; "
            f"received_count={self.received_count}"
        )

    def _print_pose_stats(self, poses):
        if not PRINT_STATS:
            return

        now = time.time()
        if now - self.last_stats_time < 1.0:
            return

        self.last_stats_time = now
        if not poses:
            print(f"{TOPIC_NAME} received, but poses is empty.")
            return

        xs = [pose.position.x for pose in poses]
        ys = [pose.position.y for pose in poses]
        zs = [pose.position.z for pose in poses]
        print(
            f"{TOPIC_NAME}: poses={len(poses)} "
            f"count={self.received_count} "
            f"x=[{min(xs):.3f}, {max(xs):.3f}] "
            f"y=[{min(ys):.3f}, {max(ys):.3f}] "
            f"z=[{min(zs):.3f}, {max(zs):.3f}]"
        )

    def stop(self):
        if self.stopped:
            return

        self.stopped = True
        self.update_subscription = None

        try:
            self.destroy_node()
        except Exception as exc:
            print(f"Retarget keypoint visualizer destroy skipped: {exc}")


old_visualizer = globals().get("_retarget_keypoint_visualizer")
if old_visualizer is not None:
    try:
        old_visualizer.stop()
        print("Stopped previous retarget keypoint visualizer.")
    except Exception as exc:
        print(f"Previous retarget visualizer stop failed, continuing: {exc}")

if not rclpy.ok():
    rclpy.init()

_retarget_keypoint_visualizer = RetargetKeypointVisualizer()
