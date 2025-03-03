import copy
import logging
import sys
import time

from pytrade.wrapper import TRADEWrapper
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from ai.thinkingrobots.trade import TRADE
from bound_mpc.ConvexSetPlanner import ConvexSetPlanner
from bound_mpc.Logger import Logger
from bound_mpc.Plotter import Plotter
from bound_mpc.RobotListener import RobotListener
from bound_mpc.RobotModel import RobotModel
from bound_mpc.utils import (
    append_grasp_set,
    create_traj_msg,
    get_default_path,
    get_default_weights,
)
from bound_mpc.utils.util_functions import normalize_set_size
from bound_mpc_msg.msg import Vector
from bound_mpc_msg.srv import MPCParams, Trajectory, UpdateConstraints
from control_msgs.action import GripperCommand
from edu.tufts.hrilab.interfaces import PlannerInterface
from jpype import JImplements, JOverride, JString
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

# Initialize Logging
formatter = logging.Formatter(
    fmt="%(asctime)s.%(msecs)03d - %(name)s - %(levelname)s - %(message)s",
    datefmt="%H:%M:%S",
)
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(formatter)
logger = logging.getLogger("Logger")
logger.addHandler(handler)
logger.info("Starting Planner Service...")


@JImplements(PlannerInterface)
class DiarcWrapper:
    def __init__(self):
        self.positions = []
        self.orientations = []
        self.received_pose = False
        self.grasp = True
        self.grasps = []

        p_cup_side = np.array([0.4, 0.45, 0.07])
        r_cup_side = R.from_euler("XYZ", [0, 0, np.pi / 2]).as_matrix()
        # p_cup_side = np.array([0.4, -0.4, 0.27])
        # r_cup_side = R.from_euler("XYZ", [0, 0, -np.pi / 2]).as_matrix()
        self.positions.append(p_cup_side)
        self.orientations.append(r_cup_side)
        self.grasps.append(True)

        p_cup_ho = np.array([0.0, -0.6, 0.6])
        r_cup_ho = R.from_euler("XYZ", [0, 0, -np.pi / 2.1]).as_matrix()
        self.positions.append(p_cup_ho)
        self.orientations.append(r_cup_ho)
        self.grasps.append(False)

        self.e_r_bound = None
        self.add_obs = None
        self.obstacles = [
            [
                0.2,
                -0.3,
                0.0,
                1.5,
                -0.1,
                1.2,
            ],
        ]

    def go_to_pose(self, pose_id):
        self.received_pose = True
        self.p_des = self.positions[pose_id]
        self.r_des = self.orientations[pose_id]
        self.grasp = self.grasps[pose_id]

    def set_constraint(self):
        e_r_bound = np.array([3, 90, 3, -3, -90, -3]) * np.pi / 180
        self.e_r_bound = e_r_bound

    def add_obstacles(self, obs_id):
        self.add_obs = [self.obstacles[obs_id]]

    @JOverride
    def send_to_planner(self, instruction):
        logger.info(f"(Planner) Instruction received {instruction}")
        split_instr = str(instruction).split("(")
        logger.info(f"(Planner) Split instruction{split_instr}")
        if split_instr[0] == "GRASP" or split_instr[0] == "PASS":
            logger.info(f"(Planner) Passing instruction {split_instr[1]}")
            if split_instr[1] == "CUP_DEFAULT)" or split_instr[1] == "MUG_DEFAULT)":
                print("Going to default cup pose")
                self.go_to_pose(0)
            elif split_instr[1] == "CUP,MANNER" or split_instr[1] == "MUG,MANNER":
                print("Setting constraint")
                self.set_constraint()
            else:
                print("Adding laptop avoidance")
                self.add_obstacles(0)
        return "s"


class ExperimentRunner(Node):
    def __init__(self):
        super().__init__("experiment_runner")
        self.gripper_client = ActionClient(
            self, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )
        self.traj_srv = self.create_client(Trajectory, "/mpc/set_trajectory")
        self.params_srv = self.create_client(MPCParams, "/mpc/set_params")
        self.con_srv = self.create_client(UpdateConstraints, "/mpc/update_constraints")

        self.params = MPCParams.Request()
        self.params.n = 15
        self.params.dt = 0.1
        weights = get_default_weights()
        weights[0] = 0.0
        self.params.weights = weights.tolist()
        self.params.build = False
        self.params.simulate = True
        self.params.experiment = False
        self.params.real_time = False
        self.params.nr_segs = 4
        self.update = False
        self.p_horizon = []
        self.robot_model = RobotModel()
        self.updated_bounds = False

        self.q0 = np.zeros(7)
        # q0[0] = np.pi / 1.2
        self.q0[3] = np.pi / 2
        self.q0[5] = np.pi / 3
        self.q0[6] = np.pi / 2

        # Define obstacles as boxes by limits (-x, -y, -z, x, y, z)
        p1 = np.array([0.4, 0.45, 0.07])
        self.obstacles = [
            [
                p1[0] - 0.045,
                p1[1] - 0.025,
                p1[2] - 0.04,
                p1[0] + 0.045,
                p1[1] + 0.045,
                p1[2] + 0.06,
            ],
            [0.2, -1.0, -0.1, 1.0, 1.0, 0.01],
            [-0.07, -0.07, 0.0, 0.07, 0.07, 0.35],
            [0.2, -0.35, 0.0, 0.5, -0.1, 0.10],
            # [0.2, -0.35, 0.0, 0.5, -0.33, 0.25],
        ]

        # Plan a path to the final pose
        self.planner = ConvexSetPlanner(
            e_p_max=0.5,
            obstacles=copy.deepcopy(self.obstacles),
        )

        # Open the gripper in the real experiment
        if self.params.simulate:
            self.command_gripper(position=0.0)

        # Send the parameters to the MPC
        self.send_params()

        # Send initial trajectory
        p0fk, _, _ = self.robot_model.forward_kinematics(self.q0, 0 * self.q0)
        r0 = R.from_rotvec(p0fk[3:]).as_matrix()
        (
            p_via,
            r_via,
            bp1_list,
            br1_list,
            e_r_bound,
        ) = get_default_path(p0fk[:3], r0, 2)
        sets_via = []
        a, b = self.planner.set_finder.init_halfspaces()
        a = np.array(a)
        b = np.array(b)
        for _ in range(2):
            sets_via.append([a, b])
        sets_via = normalize_set_size(sets_via)
        msg = create_traj_msg(
            p_via,
            r_via,
            bp1_list,
            br1_list,
            e_r_bound,
            p0fk,
            self.q0,
            self.obstacles[1:],
            sets_via,
        )
        self.traj_srv.wait_for_service()
        future = self.traj_srv.call_async(msg)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

    def command_gripper(self, position):
        print("Opening gripper ...")
        msg = GripperCommand.Goal()
        msg.command.position = position
        msg.command.max_effort = 100.0
        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, future)

    def send_params(self):
        while not self.params_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Parameter service not available, waiting again...")
        future = self.params_srv.call_async(self.params)
        rclpy.spin_until_future_complete(self, future)

    def send_constraints(self, e_r_bound):
        msg = UpdateConstraints.Request()
        msg.e_r_bound.x = e_r_bound.tolist()
        while not self.con_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Constraint service not available, waiting again...")
        future = self.con_srv.call_async(msg)
        rclpy.spin_until_future_complete(self, future)

    def plan_path(
        self, p0, p1, r0, r1, q0, p_horizon=[], new_obs=False, grasp_set=True
    ):
        if grasp_set:
            grasp_dir = r1[:, 0]
            p1 = p1 - 0.1 * grasp_dir
        update = True if p_horizon != [] else False
        start = time.time()
        (
            p_via,
            r_via,
            bp1_list,
            sets_via,
        ) = self.planner.plan_convex_set_path(
            p0,
            p1,
            r0,
            r1,
            replanning=update,
            p_horizon=p_horizon,
            new_obs=new_obs,
        )
        (
            _,
            _,
            _,
            br1_list,
            e_r_bound,
        ) = get_default_path(p0, r0, len(p_via))
        if grasp_set:
            (
                p_via,
                r_via,
                bp1_list,
                sets_via,
            ) = append_grasp_set(p_via, r_via, bp1_list, sets_via, grasp_dir, size=0.02)
            self.planner.publish_sets(sets_via)
            br1_list.append(br1_list[-1])
            e_r_bound.append(np.array([10, 10, 10, -10, -10, -10]) * np.pi / 180)
        br1_list = [np.array([1.0, 0.0, 0.0])] * (len(p_via) - 1)
        if self.updated_bounds or new_obs:
            e_r_bound = [np.array([3, 90, 3, -3, -90, -3]) * np.pi / 180] * (
                len(p_via) - 1
            )
        stop = time.time()
        print(f"Path planning took {stop - start:.2f}s")

        # Publish Trajectory
        p0fk, _, _ = self.robot_model.forward_kinematics(q0, 0 * q0)
        msg = create_traj_msg(
            p_via,
            r_via,
            bp1_list,
            br1_list,
            e_r_bound,
            p0fk,
            q0,
            self.obstacles[1:],
            sets_via,
            update=update,
        )
        self.traj_srv.wait_for_service()
        future = self.traj_srv.call_async(msg)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()


def main(diarc_wrapper):
    replan = True
    rclpy.init()
    node = ExperimentRunner()
    # Listener to actual robot data
    listener = RobotListener()
    # Create Logger
    logger_mpc = Logger(0.0, node.params.n)
    # Start the logging
    logger_mpc.start()
    while logger_mpc.i < 1:
        rclpy.spin_once(logger_mpc)

    # Initial Cartesian pose
    q0 = node.q0
    node.robot_model = RobotModel()
    p0fk, _, _ = node.robot_model.forward_kinematics(q0, 0 * q0)
    p0 = p0fk[:3]
    r0 = R.from_rotvec(p0fk[3:]).as_matrix()

    try:
        replan = True
        new_bounds = False
        new_obs = False
        new_obs_bounds = new_obs
        init = True
        at_end = False
        gripper_open = True
        if new_bounds:
            tail = "scen1_upright"
        elif new_obs:
            tail = "scen1_avoid"
        else:
            tail = "scen1_no_avoid"
        while (
            logger_mpc.phi_max - logger_mpc.phi > 0.01
            or np.linalg.norm(logger_mpc.dq_traj[-1]) > 0.01
            or logger_mpc.phi_max == 0.0
            or np.linalg.norm(logger_mpc.p_traj[-1][:3] - diarc_wrapper.positions[1])
            > 0.01
        ):
            if (
                logger_mpc.phi_max - logger_mpc.phi < 0.01
                and logger_mpc.dq_norm < 0.12
                and logger_mpc.phi_max > 0.001
            ):
                at_end = True
                # Close the gripper in the real experiment
                if gripper_open:
                    if node.params.simulate:
                        node.command_gripper(position=1.0)
                    gripper_open = False
            if at_end and replan:
                diarc_wrapper.go_to_pose(1)
                replan = False

            # if init:
            #     diarc_wrapper.go_to_pose(0)
            #     init = False
            # if logger_mpc.i_seg == 14 and new_obs and not replan:
            #     diarc_wrapper.add_obstacles(0)
            #     new_obs = False
            # if logger_mpc.i_seg == 14 and new_bounds and not replan:
            #     diarc_wrapper.set_constraint()
            #     new_bounds = False
            # if new_obs_bounds and not replan:
            #     diarc_wrapper.set_constraint()
            #     new_obs_bounds = False

            if diarc_wrapper.received_pose:
                diarc_wrapper.received_pose = False
                if logger_mpc.i > 0:
                    node.p_horizon = [x[-1][:3] for x in logger_mpc.p_traj_horizon[1:]]
                    p0 = node.p_horizon[0]
                    r0 = R.from_rotvec(logger_mpc.p_traj[-1][3:]).as_matrix()
                    node.update = True
                p1 = diarc_wrapper.p_des
                r1 = diarc_wrapper.r_des

                if at_end:
                    node.planner.adapted_obs = node.planner.adapted_obs[1:]
                    node.planner.obs = node.planner.obs[1:]
                    node.planner.obs_points = node.planner.obs_points[1:]
                    node.planner.obs_points_sets = node.planner.obs_points_sets[1:]
                    node.planner.obs_sets = node.planner.obs_sets[1:]
                    node.planner.set_finder.obs_sets = node.planner.obs_sets.copy()
                    node.planner.set_finder.obs_points_sets = (
                        node.planner.obs_points_sets.copy()
                    )

                print(
                    f"(Replanning) t = {logger_mpc.i* node.params.dt}s at phi = {logger_mpc.phi:.3f}m"
                )
                print(f"(Replanning) p0 = {p0}")
                print(f"(Replanning) p1 = {p1}")
                node.plan_path(
                    p0,
                    p1,
                    r0,
                    r1,
                    q0,
                    p_horizon=node.p_horizon,
                    grasp_set=diarc_wrapper.grasp,
                )
                logger_mpc.i_seg = 0
            if diarc_wrapper.add_obs is not None:
                print("(Replanning) New obstacles triggering replanning")
                if logger_mpc.i > 0:
                    node.p_horizon = [x[-1][:3] for x in logger_mpc.p_traj_horizon[1:]]
                    p0 = node.p_horizon[0]
                    r0 = R.from_rotvec(logger_mpc.p_ref[-1][3:]).as_matrix()
                    node.update = True
                p1 = diarc_wrapper.p_des
                r1 = diarc_wrapper.r_des
                node.planner.add_obstacle_reps(diarc_wrapper.add_obs, True)
                node.plan_path(
                    p0, p1, r0, r1, q0, p_horizon=node.p_horizon, new_obs=True, grasp_set=False
                )
                diarc_wrapper.add_obs = None
                logger_mpc.i_seg = 0

            if diarc_wrapper.e_r_bound is not None:
                print("(BoundsUpdate) Updating MPC bounds")
                if not replan:
                    node.send_constraints(diarc_wrapper.e_r_bound)
                node.updated_bounds = True
                diarc_wrapper.e_r_bound = None

            rclpy.spin_once(logger_mpc, timeout_sec=0.001)
            if node.params.simulate:
                rclpy.spin_once(listener, timeout_sec=0.001)

    except KeyboardInterrupt:
        logger.info("Caught KeyboardInterrupt")
        pass

    # Stop the logging thread
    logger_mpc.stop()
    listener.stop_logging = True

    # Plot stuff
    save_data = False
    show_plots = True
    path = "/home/toeleric/research/boundmpc_speech/src/boundmpc/data/"

    plotter = Plotter(
        logger_mpc,
        node.params,
        listener=listener,
        obstacles=node.obstacles,
        path=path,
        tail=tail,
        save_data_flag=save_data,
        x_values="time",
    )

    if show_plots:
        plt.show()


if __name__ == "__main__":
    wrapper = TRADEWrapper()
    planner_component = DiarcWrapper()
    TRADE.registerAllServices(planner_component, "")

    logger.info("Planner Service registered with TRADE.")
    main(planner_component)
