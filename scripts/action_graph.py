# This is a python test scripts for IsaacSim action graph
# Fit for carter-v1, sensor part and controller part are finished


import omni
import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets

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
