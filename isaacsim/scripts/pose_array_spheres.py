import time

from geometry_msgs.msg import PoseArray
import omni.kit.app
import omni.usd
from pxr import Gf, UsdGeom
import rclpy
from rclpy.node import Node


TOPIC_NAME = "/hybrik_pose_points"
ROOT_PRIM = "/World/ROS_Points"
JOINT_COUNT = 29
SPHERE_RADIUS = 0.035
SCALE = 2.2
ANCHOR = Gf.Vec3d(0.0, 0.0, 1.2)
PRINT_STATS = False
# To stop this script from Isaac Script Editor, run:
# isaacsim/scripts/stop_pose_array_spheres.py


def hybrik_to_isaac(position):
    return Gf.Vec3d(
        ANCHOR[0] + SCALE * position.x,
        ANCHOR[1] - SCALE * position.z,
        ANCHOR[2] - SCALE * position.y,
    )


class PoseArraySphereVisualizer(Node):
    def __init__(self):
        self.instance_id = int(time.time() * 1000)
        super().__init__(f"isaac_pose_array_sphere_visualizer_{self.instance_id}")

        self.stage = omni.usd.get_context().get_stage()
        self.joint_xforms = []
        self.latest_msg = None
        self.has_new_msg = False
        self.last_stats_time = 0.0
        self.stopped = False

        self._create_spheres()

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
                name=f"hybrik_pose_array_sphere_visualizer_{self.instance_id}",
            )
        )

        print(f"Listening to {TOPIC_NAME}; updating {JOINT_COUNT} spheres.")

    def _create_spheres(self):
        UsdGeom.Xform.Define(self.stage, ROOT_PRIM)

        for index in range(JOINT_COUNT):
            path = f"{ROOT_PRIM}/joint_{index:02d}"
            sphere = UsdGeom.Sphere.Define(self.stage, path)
            sphere.CreateRadiusAttr(SPHERE_RADIUS)
            xform = UsdGeom.XformCommonAPI(sphere)
            xform.SetTranslate(
                Gf.Vec3d(index * SPHERE_RADIUS * 2.5, 0.0, 1.0)
            )
            self.joint_xforms.append(xform)

    def pose_callback(self, msg):
        self.latest_msg = msg
        self.has_new_msg = True

    def on_update(self, event):
        if self.stopped:
            return

        rclpy.spin_once(self, timeout_sec=0.0)

        if self.latest_msg is None or not self.has_new_msg:
            return

        self.has_new_msg = False
        poses = self.latest_msg.poses
        count = min(len(poses), len(self.joint_xforms))

        for index in range(count):
            isaac_position = hybrik_to_isaac(poses[index].position)
            self.joint_xforms[index].SetTranslate(isaac_position)

        now = time.time()
        if PRINT_STATS and now - self.last_stats_time > 1.0:
            self.last_stats_time = now
            self._print_stats(poses)

    def _print_stats(self, poses):
        if not poses:
            print("PoseArray received, but poses is empty.")
            return

        xs = [pose.position.x for pose in poses]
        ys = [pose.position.y for pose in poses]
        zs = [pose.position.z for pose in poses]
        print(
            f"poses={len(poses)} "
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
            print(f"Visualizer node destroy skipped: {exc}")


old_visualizer = globals().get("_hybrik_pose_visualizer")
if old_visualizer is not None:
    try:
        old_visualizer.stop()
        print("Stopped previous HybrIK pose visualizer.")
    except Exception as exc:
        print(f"Previous visualizer stop failed, continuing: {exc}")

if not rclpy.ok():
    rclpy.init()

_hybrik_pose_visualizer = PoseArraySphereVisualizer()
