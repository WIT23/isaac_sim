# This is a python test scripts for IsaacSim action graph
# Fit for carter-v1, sensor part and controller part are finished


import omni
import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.prims import XFormPrim, GeometryPrim
import omni.graph.core as og
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np


def add_action_graph(self, prefix):
    stage = omni.usd.get_context().get_stage()
    keys = og.Controller.Keys
    robot = "robot" + str(prefix)
    # Laser IMU
    og.Controller.edit(
        {"graph_path": "/World/" + robot + "/sensor_graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("LidarReader", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                ("LidarPub", "omni.isaac.ros_bridge.ROS1PublishLaserScan"),
                ("ImuReader", "omni.isaac.sensor.IsaacReadIMU"),
                ("ImuPub", "omni.isaac.ros_bridge.ROS1PublishImu"),
            ],
            keys.SET_VALUES: [
                ("LidarPub.inputs:topicName", "scan"),
                ("ImuPub.inputs:topicName", "imu"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "LidarReader.inputs:execIn"),
                ("OnTick.outputs:tick", "LidarPub.inputs:execIn"),
                ("OnTick.outputs:tick", "ImuReader.inputs:execIn"),
                ("OnTick.outputs:tick", "ImuPub.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "ImuPub.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "LidarPub.inputs:timeStamp"),
                ("ImuReader.outputs:angVel", "ImuPub.inputs:angularVelocity"),
                ("ImuReader.outputs:linAcc", "ImuPub.inputs:linearAcceleration"),
                ("ImuReader.outputs:orientation", "ImuPub.inputs:orientation"),
                ("LidarReader.outputs:azimuthRange", "LidarPub.inputs:azimuthRange"),
                ("LidarReader.outputs:depthRange", "LidarPub.inputs:depthRange"),
                ("LidarReader.outputs:horizontalFov", "LidarPub.inputs:horizontalFov"),
                ("LidarReader.outputs:horizontalResolution", "LidarPub.inputs:horizontalResolution"),
                ("LidarReader.outputs:intensitiesData", "LidarPub.inputs:intensitiesData"),
                ("LidarReader.outputs:linearDepthData", "LidarPub.inputs:linearDepthData"),
                ("LidarReader.outputs:numCols", "LidarPub.inputs:numCols"),
                ("LidarReader.outputs:numRows", "LidarPub.inputs:numRows"),
                ("LidarReader.outputs:rotationRate", "LidarPub.inputs:rotationRate"),
            ]
        }
    )

    # Differential Controller
    og.Controller.edit(
        {"graph_path": "/World/" + robot + "/controller_graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("VelocitySub", "omni.isaac.ros_bridge.ROS1SubscribeTwist"),
                ("Scale", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                ("LinearBreaker", "omni.graph.nodes.BreakVector3"),
                ("AngularBreaker", "omni.graph.nodes.BreakVector3"),
                ("DiffController", "omni.isaac.wheeled_robots.DifferentialController"),
                ("ArtiController", "omni.isaac.core_nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("ArtiController.inputs:usePath", False),
                ("DiffController.inputs:wheelDistance", 0.53342),
                ("DiffController.inputs:wheelRadius", 0.24),
                ("ArtiController.inputs:jointNames", ["right_wheel", "left_wheel"])
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "VelocitySub.inputs:execIn"),
                ("OnTick.outputs:tick", "ArtiController.inputs:execIn"),
                ("VelocitySub.outputs:execOut", "DiffController.inputs:execIn"),
                ("VelocitySub.outputs:angularVelocity", "AngularBreaker.inputs:tuple"),
                ("VelocitySub.outputs:linearVelocity", "Scale.inputs:value"),
                ("AngularBreaker.outputs:z", "DiffController.inputs:angularVelocity"),
                ("Scale.outputs:result", "LinearBreaker.inputs:tuple"),
                ("LinearBreaker.outputs:x", "DiffController.inputs:linearVelocity"),
                ("DiffController.outputs:velocityCommand", "ArtiController.inputs:velocityCommand"),
            ]
        }
    )

    set_targets(prim=stage.GetPrimAtPath("/World/" + robot + "/controller_graph/ArtiController"),
                attribute="inputs:targetPrim",
                target_prim_paths=["/World/robot0"])

    # tf clock Odom
    og.Controller.edit(
        {"graph_path": "/World/" + robot + "/utils_graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("ClockPub", "omni.isaac.ros_bridge.ROS1PublishClock"),
                ("OdomReader", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                ("OdomPub", "omni.isaac.ros_bridge.ROS1PublishOdometry"),
                ("RawTFPub", "omni.isaac.ros_bridge.ROS1PublishRawTransformTree"),
                ("TFPub1", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                ("TFPub2", "omni.isaac.ros_bridge.ROS1PublishTransformTree")
            ],
            keys.SET_VALUES: [
                ("ClockPub.inputs:topicName", "clock"),
            ],
            keys.CONNECT: [
                ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
                ("OnTick.outputs:tick", "ClockPub.inputs:execIn"),
                ("OnTick.outputs:tick", "RawTFPub.inputs:execIn"),
            ]
        }
    )


def _add_obstacle(world):
    xform = world.scene.add(
        XFormPrim(
            prim_path="/World/Env",
            name="EnvXForm"
        )
    )
    bound_position = np.array(
        [(5.5, 0.0, 0.5),
         (0.0, 5.5, 0.5),
         (-5.5, 0.0, 0.5),
         (0.0, -5.5, 0.5)]
    )
    bound_scale = np.array(
        [(1.0, 12.0, 1.0),
         (10.0, 1.0, 1.0)]
    )
    for i in range(4):
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Env/bound{i + 1}",
                name=f"bound{i + 1}",
                mass=1000.0,
                position=bound_position[i],
                color=np.array([1.0, 1.0, 1.0]),
                scale=bound_scale[i % 2]
            )
        )

    for i in range(4):
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Env/obstacle{i + 1}",
                name=f"obstacle{i + 1}",
                mass=10000.0,
                position=np.array([-1.5, -2.5 + i * 2, 0.5]),
                color=np.array([1.0, 1.0, 1.0]),
                scale=np.array([3.0, 1.0, 1.0])
            )
        )

    for i in range(4):
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Env/obstacle{i + 1 + 4}",
                name=f"obstacle{i + 1 + 4}",
                mass=10000.0,
                position=np.array([2.5, -2.5 + i * 2, 0.5]),
                color=np.array([1.0, 1.0, 1.0]),
                scale=np.array([3.0, 1.0, 1.0])
            )
        )
