# This is an Isaac Sim Connection Scripts for single Carter-v1 robot
# Multi Robots simulation can inherit Class IsaacConnection

from omni.isaac.kit import SimulationApp

CARTER_USD_PATH = "/home/gr-agv-lx91/isaac_sim_ws/src/isaac_sim/isaac/carter.usd"
ENV_USD_PATH = "/home/gr-agv-lx91/isaac_sim_ws/src/isaac_sim/isaac/env.usd"
config = {
    "headless": False,
}
simulation_app = SimulationApp(config)

# utils
import sys
import numpy as np
from enum import Enum
import carb
import time

# isaac
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.prims import XFormPrim, GeometryPrim
import omni.graph.core as og
from omni.isaac.core.utils.stage import add_reference_to_stage

# ros
import rospy
from std_srvs.srv import Empty, EmptyResponse
import rosgraph


class SimulationState(Enum):
    RESET = 0
    PAUSE = 1
    UNPAUSE = 2
    NORMAL = 4
    CLOSE = 8


class IsaacSimConnection:
    def __init__(self):
        self.setup_ros()
        self.setup_scene()
        self.state = SimulationState.NORMAL

    def setup_ros(self):
        enable_extension("omni.isaac.ros_bridge")
        while not rosgraph.is_master_online():
            carb.log_error("Please run roscore before executing this script")
            time.sleep(2.0)
        self.pause_sub = rospy.Service("/pause", Empty, self._pause_callback)
        self.unpause_sub = rospy.Service("/unpause", Empty, self._unpause_callback)
        self.reset_sub = rospy.Service("/reset", Empty, self._reset_callback)
        self.close_sub = rospy.Service("/close", Empty, self._close_callback)

    def setup_scene(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            print("Could not find Isaac Sim assets folder")
            sys.exit(-1)
        self.robots = []
        self.obstacles = []
        self._add_robot()
        self._add_env()
        self._add_action_graph()
        # self.world.reset()
        # self.set_namespace()

    def cycle(self):
        self.world.reset()
        self.world.initialize_physics()
        simulation_app.update()
        self.world.play()
        simulation_app.update()
        while simulation_app.is_running:
            if self.state == SimulationState.NORMAL:
                self.world.step()
            elif self.state == SimulationState.PAUSE:
                self.world.pause()
            elif self.state == SimulationState.UNPAUSE:
                self.world.play()
                self.world.step()
                self.state = SimulationState.NORMAL
            elif self.state == SimulationState.RESET:
                self.robot.set_world_pose(
                    position=np.array([1.5, 1.5, 0]),
                    orientation=np.array([1.0, 0.0, 0.0, 0.0])
                )
                self.world.play()
                self.world.step()
                self.state = SimulationState.NORMAL
            elif self.state == SimulationState.CLOSE:
                break
        print("simulation app is out of running")
        self.world.stop()
        simulation_app.close()

    def _add_robot(self):
        wheel_dof_names = ["left_wheel", "right_wheel"]
        self.robot = self.world.scene.add(
            WheeledRobot(
                prim_path="/World/Carter",
                name="Carter",
                wheel_dof_names=wheel_dof_names,
                create_robot=True,
                usd_path=CARTER_USD_PATH,
                position=np.array([1.5, 1.5, 0]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0])
            )
        )
        self.robots.append(self.robot)
        self.robot_controller = DifferentialController(name="simple_control", wheel_radius=0.0325, wheel_base=0.1125)

    def _add_env(self):
        add_reference_to_stage(usd_path=ENV_USD_PATH, prim_path="/World/Env")
        self.world.scene.add(
            GeometryPrim(
                prim_path="/World/Env",
                name="Env",
                collision=True
            )
        )

    def _add_action_graph(self):
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/clock_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("SimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ClockPub", "omni.isaac.ros_bridge.ROS1PublishClock"),
                ],
                keys.SET_VALUES: [
                    ("ClockPub.inputs:topicName", "clock"),
                ],
                keys.CONNECT: [
                    ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
                    ("OnTick.outputs:tick", "ClockPub.inputs:execIn"),
                ]
            }
        )

    @staticmethod
    def _set_namespace():
        graph = og.Controller.graph("/World/Carter/Carter_Control_Graph")
        og.GraphController.set_variable_default_value(variable_id=(graph, "namespace"), value="Carter")

    def _pause_callback(self, msg):
        self.state = SimulationState.PAUSE
        return EmptyResponse()

    def _unpause_callback(self, msg):
        self.state = SimulationState.UNPAUSE
        return EmptyResponse()

    def _reset_callback(self, msg):
        self.state = SimulationState.RESET
        return EmptyResponse()

    def _close_callback(self, msg):
        self.state = SimulationState.CLOSE
        return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("IsaacSimConnection")
    connection = IsaacSimConnection()
    connection.cycle()
