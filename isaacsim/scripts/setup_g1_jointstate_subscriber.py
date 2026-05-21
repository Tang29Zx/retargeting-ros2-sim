"""
Run this file inside Isaac Sim Script Editor.

It creates an OmniGraph that subscribes to /g1_joint_targets
sensor_msgs/msg/JointState and drives an existing G1 articulation with
position commands. It does not create or reference a new robot copy.
The ROS2 bridge extension must be available in Isaac Sim.
"""

import omni.graph.core as og
import omni.kit.app
import omni.usd
from pxr import Usd, UsdPhysics


GRAPH_PATH = "/World/G1_ROS2_JointState_Graph"
COMMAND_TOPIC = "/g1_joint_targets"
STATE_TOPIC = "/isaac_g1_joint_states"
PREFERRED_ROBOT_ROOTS = (
    "/World/G1",
    "/World/g1",
    "/World/g1_23dof",
    "/World/G1_23DOF",
)


def enable_extension(extension_name):
    manager = omni.kit.app.get_app().get_extension_manager()
    if manager.is_extension_enabled(extension_name):
        return
    manager.set_extension_enabled_immediate(extension_name, True)


def get_stage():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
    return stage


def find_articulation_root_under(stage, root_path):
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None

    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim.GetPath().pathString

    return None


def find_existing_articulation(stage):
    for root_path in PREFERRED_ROBOT_ROOTS:
        articulation_path = find_articulation_root_under(stage, root_path)
        if articulation_path is not None:
            return articulation_path

    world = stage.GetPrimAtPath("/World")
    search_root = world if world.IsValid() else stage.GetPseudoRoot()
    for prim in Usd.PrimRange(search_root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim.GetPath().pathString

    raise RuntimeError(
        "No existing articulation root found in the current stage. "
        "Load your G1 USD in Isaac Sim first, then run this script again. "
        f"Preferred roots checked: {PREFERRED_ROBOT_ROOTS}"
    )


def remove_existing_graph(stage):
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)


def create_ros2_joint_graph(articulation_path):
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeJointState",
                 "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("PublishJointState",
                 "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("ArticulationController",
                 "isaacsim.core.nodes.IsaacArticulationController"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick",
                 "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick",
                 "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick",
                 "ArticulationController.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime",
                 "PublishJointState.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames",
                 "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand",
                 "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand",
                 "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand",
                 "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubscribeJointState.inputs:topicName", COMMAND_TOPIC),
                ("SubscribeJointState.inputs:queueSize", 10),
                ("PublishJointState.inputs:topicName", STATE_TOPIC),
                ("PublishJointState.inputs:targetPrim", articulation_path),
                ("ArticulationController.inputs:robotPath",
                 articulation_path),
            ],
        },
    )


def main():
    enable_extension("isaacsim.ros2.bridge")

    stage = get_stage()
    articulation_path = find_existing_articulation(stage)
    remove_existing_graph(stage)
    create_ros2_joint_graph(articulation_path)

    print("G1 ROS2 JointState graph is ready.")
    print(f"  articulation:    {articulation_path}")
    print(f"  subscribe:       {COMMAND_TOPIC}")
    print(f"  publish state:   {STATE_TOPIC}")
    print("Press Play in Isaac Sim, then publish /g1_joint_targets from ROS2.")


main()
