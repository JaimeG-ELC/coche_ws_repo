#!/usr/bin/env python3
import math
import numpy as np
from dataclasses import dataclass, field
import cvxpy
from scipy.linalg import block_diag
from scipy.sparse import block_diag, csc_matrix
from scipy.spatial import transform
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class MPCSettings:

    """ Dataclass to store MPC settings and hyperparameters"""
    node: Node

    def __post_init__(self):

        # Dimensions
        self.state_size = 4 # x, y, v, yaw
        self.input_size = 2 # accel, steering
        self.candidate_speed = 2.0

        # R Matrix: Input Cost
        self.input_cost = np.diag([
            self.node.get_parameter("input_cost_accel").value,
            self.node.get_parameter("input_cost_steering").value
        ])
        
        #Rd Matrix: Input Rate Cost
        self.input_rate_cost = np.diag([
            self.node.get_parameter("input_rate_cost_accel").value,
            self.node.get_parameter("input_rate_cost_steering").value
        ])


        # Q Matrix: State Error Cost
        self.state_cost = np.diag([
            self.node.get_parameter("state_cost_x").value,
            self.node.get_parameter("state_cost_y").value,
            self.node.get_parameter("state_cost_v").value,
            self.node.get_parameter("state_cost_yaw").value
        ])
        
        # Qf Matrix: Final State Error Cost
        self.final_state_cost = np.diag([
            self.node.get_parameter("final_state_cost_x").value,
            self.node.get_parameter("final_state_cost_y").value,
            self.node.get_parameter("final_state_cost_v").value,
            self.node.get_parameter("final_state_cost_yaw").value
        ])

        # MPC Parameters
        self.horizon = self.node.get_parameter("horizon").value
        self.search_idx = self.node.get_parameter("search_idx").value
        self.dt = self.node.get_parameter("dt").value
        self.step_length = self.node.get_parameter("step_length").value

        # Vehicle Parameters
        self.veh_length = self.node.get_parameter("veh_length").value
        self.veh_width = self.node.get_parameter("veh_width").value
        self.wheelbase = self.node.get_parameter("wheelbase").value
        self.min_steer = self.node.get_parameter("min_steer").value
        self.max_steer = self.node.get_parameter("max_steer").value
        self.max_steer_rate = self.node.get_parameter("max_steer_rate").value
        self.max_speed = self.node.get_parameter("max_speed").value
        self.min_speed = self.node.get_parameter("min_speed").value
        self.max_accel = self.node.get_parameter("max_accel").value


@dataclass
class VehicleState:
    x: float = 0.0  # x position [m]
    y: float = 0.0  # y position [m]
    delta: float = 0.0 # steering angle [rad]
    v: float = 0.0 # velocity [m/s]
    yaw: float = 0.0  # heading angle [rad]
    yaw_rate: float = 0.0 # yaw rate [rad/s]
    beta: float = 0.0 # slip angle [rad]


class MPCNode(Node):
    """
 A Node implementing a kinematic MPC controller. """

    def __init__(self):
        super().__init__('mpc_node')

        # Initialize MPC settings and variables
        self.setup_parameters()
        self.settings = MPCSettings(self)
        self.prev_steer_rate = None
        self.prev_accel = None
        self.initialized = False
        self.last_scan = None
        self.use_real = self.get_parameter("real_environment").value
        self.map_filename = self.get_parameter("csv_file").value

        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.rot_matrix = np.identity(3)

        # Hyperparameters for reactive obstacle avoidance - Gap analysis
        self.downsample_gap = self.get_parameter("downsampling").value # Downsampling factor for lidar data
        self.max_sight = self.get_parameter("max_sight").value # Maximum range of lidar sensor  
        self.gap_threshold = self.get_parameter("gap_threshold").value # Minimum gap size for obstacle avoidance

        # Topics
        pose_topic = "/pf/viz/inferred_pose" if self.use_real else "/ego_racecar/odom"
        odom_topic = "/pf/viz/inferred_pose" if self.use_real else "/ego_racecar/odom"
        drive_topic = "/drive"
        vis_ref_topic = "/ref_traj_marker"
        vis_wpts_topic = "/waypoints_marker"
        vis_pred_topic = "/pred_path_marker"
        vis_candidate_topic = "/candidate_traj_marker"

        # Initialize variables for reactive obstacle avoidance
        self.processed_ranges = None # Processed lidar data
        self.current_speed = 0.0 # Current vehicle speed

        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped if self.use_real else Odometry,pose_topic,self.pose_update,1)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(PoseStamped if self.use_real else Odometry,odom_topic,self.odometry_update,1)

        # Publishers for drive commands and visualization markers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.drive_msg = AckermannDriveStamped()
        self.ref_traj_pub = self.create_publisher(Marker, vis_ref_topic, 1)
        self.ref_traj_marker = Marker()
        self.wpts_pub = self.create_publisher(Marker, vis_wpts_topic, 1)
        self.wpts_marker = Marker()
        self.pred_path_pub = self.create_publisher(Marker, vis_pred_topic, 1)
        self.pred_path_marker = Marker()
        self.candidate_pub = self.create_publisher(MarkerArray, vis_candidate_topic, 1)

        # Load waypoints from CSV file
        map_dir = os.path.abspath(os.path.join('src', 'csv_data'))
        self.waypoints = np.loadtxt(os.path.join(map_dir, self.map_filename + '.csv'), delimiter=',', skiprows=0)

        self.init_mpc_problem()


    # -----------------------------------------------------------
    # Callback Functions
    # -----------------------------------------------------------

    def lidar_callback(self, scan_msg):
        """ Process incoming LaserScan data. """
        self.last_scan = scan_msg
        ranges = np.array(scan_msg.ranges[180:899])
        self.processed_ranges = self.preprocess_laser_data(ranges)

    def odometry_update(self, msg):
        """ Update the current vehicle speed from odometry. """
        if not self.use_real:
            lin = msg.twist.twist.linear
            self.current_speed = math.sqrt(lin.x**2 + lin.y**2)
        self.get_logger().info(f"Current speed: {self.current_speed:.2f} m/s")

    def pose_update(self, pose_msg):
        """ Process pose message to update state, perform obstacle detection, and run MPC. """
        self.update_rotation_matrix(pose_msg)
        vehicle_state = self.get_vehicle_state(pose_msg)
        

        # Reactive obstacle detection via gap analysis
        avoidance_active = False
        if self.processed_ranges is not None:
            gap_start, gap_end = self.get_max_gap(self.processed_ranges)
            self.get_logger().info(f"Max gap: start={gap_start}, end={gap_end} (length={gap_end - gap_start})")
            if (gap_end - gap_start) < self.gap_threshold:
                avoidance_active = True

        # Determine reference trajectory
        if avoidance_active:
            candidate_trajs = self.create_candidate_trajs(vehicle_state)
            collision_flags = [self.detect_collision(traj, self.last_scan) for traj in candidate_trajs]
            chosen_idx = self.choose_candidate(candidate_trajs, collision_flags)
            self.display_candidate_trajs(candidate_trajs, collision_flags, chosen_idx, vehicle_state)
            if chosen_idx is not None:
                chosen_traj = candidate_trajs[chosen_idx]
                ref_traj = self.convert_to_global(chosen_traj, vehicle_state)
                self.get_logger().info("Obstacle detected: using local avoidance trajectory.")
            else:
                ref_traj = self.compute_reference_traj(
                    vehicle_state,
                    self.waypoints[:, 1],
                    self.waypoints[:, 2],
                    self.waypoints[:, 3],
                    self.waypoints[:, 5]
                )
                self.get_logger().warn("Obstacle detected but no collision-free candidate available!")
        else:
            ref_traj = self.compute_reference_traj(
                vehicle_state,
                self.waypoints[:, 1],
                self.waypoints[:, 2],
                self.waypoints[:, 3],
                self.waypoints[:, 5]
            )

        self.show_reference_traj(ref_traj)

        x0 = [vehicle_state.x, vehicle_state.y, vehicle_state.v, vehicle_state.yaw]
        (self.prev_accel, self.prev_steer_rate,
         ox, oy, oyaw, ov,
         predicted_state) = self.run_linear_mpc(ref_traj, x0, self.prev_accel, self.prev_steer_rate)

        steer_cmd = self.prev_steer_rate[0]
        speed_cmd = vehicle_state.v + self.prev_accel[0] * self.settings.dt

        self.drive_msg.drive.steering_angle = steer_cmd
        self.drive_msg.drive.speed = (-1.0 if self.use_real else 1.0) * speed_cmd
        self.drive_pub.publish(self.drive_msg)
        self.get_logger().info(f"Command: steering={steer_cmd:.3f}, speed={speed_cmd:.3f}")
        self.wpts_pub.publish(self.wpts_marker)

    # -----------------------------------------------------------
    # Vehicle State and MPC Functions
    # -----------------------------------------------------------
    def update_rotation_matrix(self, pose_msg):
        """ Update the rotation matrix from the current pose. """
        orien = pose_msg.pose.orientation if self.use_real else pose_msg.pose.pose.orientation
        quat = [orien.x, orien.y, orien.z, orien.w]
        self.rot_matrix = transform.Rotation.from_quat(quat).as_matrix()

    def get_vehicle_state(self, pose_msg):
        """ Extract vehicle state from pose message. """
        state = VehicleState()
        state.x = pose_msg.pose.position.x if self.use_real else pose_msg.pose.pose.position.x
        state.y = pose_msg.pose.position.y if self.use_real else pose_msg.pose.pose.position.y
        state.v = self.drive_msg.drive.speed
        orient = pose_msg.pose.orientation if self.use_real else pose_msg.pose.pose.orientation
        q = [orient.x, orient.y, orient.z, orient.w]
        state.yaw = math.atan2(2 * (q[3]*q[2] + q[0]*q[1]),
                                1 - 2 * (q[1]**2 + q[2]**2))
        return state

    def init_mpc_problem(self):
        """ Set up the CVXPY MPC optimization problem. """
        self.x_var = cvxpy.Variable((self.settings.state_size, self.settings.horizon + 1))
        self.u_var = cvxpy.Variable((self.settings.input_size, self.settings.horizon))
        cost = 0.0
        cons = []

        self.x0_param = cvxpy.Parameter((self.settings.state_size,))
        self.x0_param.value = np.zeros((self.settings.state_size,))
        self.ref_param = cvxpy.Parameter((self.settings.state_size, self.settings.horizon + 1))
        self.ref_param.value = np.zeros((self.settings.state_size, self.settings.horizon + 1))

        R_blk = block_diag(tuple([self.settings.input_cost] * self.settings.horizon))
        Rd_blk = block_diag(tuple([self.settings.input_rate_cost] * (self.settings.horizon - 1)))
        Q_blk = [self.settings.state_cost] * self.settings.horizon + [self.settings.final_state_cost]
        Q_blk = block_diag(tuple(Q_blk))

        cost += cvxpy.quad_form(cvxpy.vec(self.u_var), R_blk)
        cost += cvxpy.quad_form(cvxpy.vec(self.x_var - self.ref_param), Q_blk)
        cost += cvxpy.quad_form(cvxpy.vec(cvxpy.diff(self.u_var, axis=1)), Rd_blk)
        cost += cvxpy.quad_form(self.x_var[:, -1] - self.ref_param[:, -1], self.settings.final_state_cost)

        A_list = []
        B_list = []
        C_list = []
        state_pred = np.zeros((self.settings.state_size, self.settings.horizon + 1))
        for t in range(self.settings.horizon):
            A_mat, B_mat, C_vec = self.compute_model_matrix(state_pred[2, t], state_pred[3, t], 0.0)
            A_list.append(A_mat)
            B_list.append(B_mat)
            C_list.extend(C_vec)
        A_blk = block_diag(tuple(A_list))
        B_blk = block_diag(tuple(B_list))
        C_blk = np.array(C_list)

        m_A, n_A = A_blk.shape
        self.A_nz = cvxpy.Parameter(A_blk.nnz)
        data_A = np.ones(self.A_nz.size)
        rows_A = A_blk.row * n_A + A_blk.col
        cols_A = np.arange(self.A_nz.size)
        indexer_A = csc_matrix((data_A, (rows_A, cols_A)),
                               shape=(m_A * n_A, self.A_nz.size))
        self.A_nz.value = A_blk.data
        self.A_matrix = cvxpy.reshape(indexer_A @ self.A_nz, (m_A, n_A), order="C")

        m_B, n_B = B_blk.shape
        self.B_nz = cvxpy.Parameter(B_blk.nnz)
        data_B = np.ones(self.B_nz.size)
        rows_B = B_blk.row * n_B + B_blk.col
        cols_B = np.arange(self.B_nz.size)
        indexer_B = csc_matrix((data_B, (rows_B, cols_B)),
                               shape=(m_B * n_B, self.B_nz.size))
        self.B_matrix = cvxpy.reshape(indexer_B @ self.B_nz, (m_B, n_B), order="C")
        self.B_nz.value = B_blk.data

        self.C_param = cvxpy.Parameter(C_blk.shape)
        self.C_param.value = C_blk

        prev_x = cvxpy.vec(self.x_var[:, :-1])
        next_x = cvxpy.vec(self.x_var[:, 1:])
        cons.append(next_x == self.A_matrix @ prev_x + self.B_matrix @ cvxpy.vec(self.u_var) + self.C_param)

        dsteer = cvxpy.diff(self.u_var[1, :])
        cons.append(-self.settings.max_steer_rate * self.settings.dt <= dsteer)
        cons.append(dsteer <= self.settings.max_steer_rate * self.settings.dt)

        cons.append(self.x_var[:, 0] == self.x0_param)
        speed_seq = self.x_var[2, :]
        cons.append(self.settings.min_speed <= speed_seq)
        cons.append(speed_seq <= self.settings.max_speed)
        steer_seq = self.u_var[1, :]
        cons.append(self.settings.min_steer <= steer_seq)
        cons.append(steer_seq <= self.settings.max_steer)
        accel_seq = self.u_var[0, :]
        cons.append(accel_seq <= self.settings.max_accel)

        self.mpc_problem = cvxpy.Problem(cvxpy.Minimize(cost), cons)

    def get_nearest_point(self, pt, traj):
        """ Return the nearest point on a piecewise-linear trajectory. """
        diffs = traj[1:, :] - traj[:-1, :]
        l2s = diffs[:, 0]**2 + diffs[:, 1]**2
        dots = np.empty((traj.shape[0] - 1,))
        for i in range(dots.shape[0]):
            dots[i] = np.dot((pt - traj[i, :]), diffs[i, :])
        t_vals = dots / l2s
        t_vals[t_vals < 0.0] = 0.0
        t_vals[t_vals > 1.0] = 1.0
        projections = traj[:-1, :] + (t_vals * diffs.T).T
        dists = np.empty((projections.shape[0],))
        for i in range(dists.shape[0]):
            diff_vec = pt - projections[i]
            dists[i] = np.sqrt(np.sum(diff_vec * diff_vec))
        min_idx = np.argmin(dists)
        return projections[min_idx], dists[min_idx], t_vals[min_idx], min_idx

    def compute_reference_traj(self, state, cx, cy, cyaw, sp):
        """ Calculate a reference trajectory based on the current vehicle state. """
        ref_traj = np.zeros((self.settings.state_size, self.settings.horizon + 1))
        n_course = len(cx)
        _, _, _, idx = self.get_nearest_point(np.array([state.x, state.y]), np.array([cx, cy]).T)
        ref_traj[0, 0] = cx[idx]
        ref_traj[1, 0] = cy[idx]
        ref_traj[2, 0] = sp[idx]
        ref_traj[3, 0] = cyaw[idx]

        travel_dist = abs(state.v) * self.settings.dt
        d_index = travel_dist / self.settings.step_length
        d_index = 2  # fixed step increment
        idx_list = int(idx) + np.insert(np.cumsum(np.repeat(d_index, self.settings.horizon)), 0, 0).astype(int)
        idx_list[idx_list >= n_course] -= n_course
        ref_traj[0, :] = cx[idx_list]
        ref_traj[1, :] = cy[idx_list]
        ref_traj[2, :] = sp[idx_list]

        angle_thresh = 4.5
        for i in range(len(cyaw)):
            if cyaw[i] - state.yaw > angle_thresh:
                cyaw[i] -= 2 * np.pi
            if state.yaw - cyaw[i] > angle_thresh:
                cyaw[i] += 2 * np.pi
        ref_traj[3, :] = cyaw[idx_list]
        return ref_traj

    def simulate_motion(self, init_state, accel_seq, steer_seq, ref_traj):
        """ Predict the future motion given a sequence of control inputs. """
        sim_traj = ref_traj * 0.0
        for i, val in enumerate(init_state):
            sim_traj[i, 0] = init_state[i]
        sim_state = VehicleState(x=init_state[0], y=init_state[1], yaw=init_state[3], v=init_state[2])
        for (a, d, t) in zip(accel_seq, steer_seq, range(1, self.settings.horizon + 1)):
            sim_state = self.propagate_state(sim_state, a, d)
            sim_traj[0, t] = sim_state.x
            sim_traj[1, t] = sim_state.y
            sim_traj[2, t] = sim_state.v
            sim_traj[3, t] = sim_state.yaw
        return sim_traj

    def propagate_state(self, state, a, delta):
        """ Update the vehicle state using a kinematic bicycle model. """
        if delta >= self.settings.max_steer:
            delta = self.settings.max_steer
        elif delta <= -self.settings.max_steer:
            delta = -self.settings.max_steer
        state.x += state.v * math.cos(state.yaw) * self.settings.dt
        state.y += state.v * math.sin(state.yaw) * self.settings.dt
        state.yaw += (state.v / self.settings.wheelbase) * math.tan(delta) * self.settings.dt
        state.v += a * self.settings.dt
        state.v = min(max(state.v, self.settings.min_speed), self.settings.max_speed)
        return state

    def compute_model_matrix(self, v, phi, delta):
        """ Compute the linearized model matrices A, B, and constant term C. """
        A = np.zeros((self.settings.state_size, self.settings.state_size))
        A[0, 0] = A[1, 1] = A[2, 2] = A[3, 3] = 1.0
        A[0, 2] = self.settings.dt * math.cos(phi)
        A[0, 3] = -self.settings.dt * v * math.sin(phi)
        A[1, 2] = self.settings.dt * math.sin(phi)
        A[1, 3] = self.settings.dt * v * math.cos(phi)
        A[3, 2] = self.settings.dt * math.tan(delta) / self.settings.wheelbase

        B = np.zeros((self.settings.state_size, self.settings.input_size))
        B[2, 0] = self.settings.dt
        B[3, 1] = self.settings.dt * v / (self.settings.wheelbase * math.cos(delta) ** 2)

        C = np.zeros(self.settings.state_size)
        C[0] = self.settings.dt * v * math.sin(phi) * phi
        C[1] = -self.settings.dt * v * math.cos(phi) * phi
        C[3] = -self.settings.dt * v * delta / (self.settings.wheelbase * math.cos(delta) ** 2)
        return A, B, C

    def solve_mpc_problem(self, ref_traj, state_pred, x0):
        """ Solve the MPC quadratic program. """
        self.x0_param.value = x0
        A_list = []
        B_list = []
        C_list = []
        for t in range(self.settings.horizon):
            A_mat, B_mat, C_vec = self.compute_model_matrix(state_pred[2, t], state_pred[3, t], 0.0)
            A_list.append(A_mat)
            B_list.append(B_mat)
            C_list.extend(C_vec)
        A_blk = block_diag(tuple(A_list))
        B_blk = block_diag(tuple(B_list))
        C_blk = np.array(C_list)
        self.A_nz.value = A_blk.data
        self.B_nz.value = B_blk.data
        self.C_param.value = C_blk
        self.ref_param.value = ref_traj

        self.mpc_problem.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)
        if self.mpc_problem.status in [cvxpy.OPTIMAL, cvxpy.OPTIMAL_INACCURATE]:
            ox = np.array(self.x_var.value[0, :]).flatten()
            oy = np.array(self.x_var.value[1, :]).flatten()
            ov = np.array(self.x_var.value[2, :]).flatten()
            oyaw = np.array(self.x_var.value[3, :]).flatten()
            oa = np.array(self.u_var.value[0, :]).flatten()
            odelta = np.array(self.u_var.value[1, :]).flatten()
        else:
            self.get_logger().error("Error: MPC problem not solved optimally.")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None
        return oa, odelta, ox, oy, oyaw, ov

    def run_linear_mpc(self, ref_path, x0, prev_a, prev_delta):
        """ Run the linear MPC controller. """
        if prev_a is None or prev_delta is None:
            prev_a = [0.0] * self.settings.horizon
            prev_delta = [0.0] * self.settings.horizon
        state_prediction = self.simulate_motion(x0, prev_a, prev_delta, ref_path)
        self.show_predicted_path(state_prediction)
        mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v = self.solve_mpc_problem(ref_path, state_prediction, x0)
        return mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v, state_prediction

    # -----------------------------------------------------------
    # Candidate Trajectory Generation & Obstacle-Avoidance Functions
    # -----------------------------------------------------------
    def create_candidate_trajs(self, state):
        """ Generate candidate trajectories via forward simulation. """
        cand_speed = self.settings.candidate_speed
        init_state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=cand_speed)
        cand_trajs = []
        steer_vals = np.linspace(self.settings.min_steer, self.settings.max_steer, 20)
        horz = self.settings.horizon
        for steer in steer_vals:
            traj = np.zeros((self.settings.state_size, horz + 1))
            traj[0, 0] = init_state.x
            traj[1, 0] = init_state.y
            traj[2, 0] = init_state.v
            traj[3, 0] = init_state.yaw
            temp_state = VehicleState(x=init_state.x, y=init_state.y, yaw=init_state.yaw, v=init_state.v)
            for t in range(1, horz + 1):
                temp_state = self.propagate_state(temp_state, a=0.0, delta=steer)
                traj[0, t] = temp_state.x
                traj[1, t] = temp_state.y
                traj[2, t] = temp_state.v
                traj[3, t] = temp_state.yaw
            cand_trajs.append(traj)
        return cand_trajs

    def detect_collision(self, traj, scan_msg):
        """ Check if any point in the trajectory is in collision with obstacles from the scan. """
        if scan_msg is None:
            return False
        angles = np.arange(len(scan_msg.ranges)) * scan_msg.angle_increment + scan_msg.angle_min
        ranges = np.array(scan_msg.ranges)
        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]
        scan_pts = np.vstack((ranges * np.cos(angles), ranges * np.sin(angles))).T
        collision_thresh = 0.3  # [m]
        for t in range(traj.shape[1]):
            pt = traj[:2, t]
            dists = np.linalg.norm(scan_pts - pt.reshape(1, 2), axis=1)
            if np.any(dists < collision_thresh):
                return True
        return False

    def choose_candidate(self, candidates, collisions):
        """ Select the candidate trajectory with the smallest lateral offset that is collision-free. """
        best_idx = None
        best_offset = float('inf')
        for i, traj in enumerate(candidates):
            if not collisions[i]:
                offset = abs(traj[1, -1])
                if offset < best_offset:
                    best_offset = offset
                    best_idx = i
        return best_idx

    def convert_to_global(self, traj, state):
        """ Convert a trajectory from the vehicle frame to the global frame. """
        cos_yaw = math.cos(state.yaw)
        sin_yaw = math.sin(state.yaw)
        R = np.array([[cos_yaw, -sin_yaw],
                      [sin_yaw,  cos_yaw]])
        global_traj = np.copy(traj)
        for t in range(traj.shape[1]):
            local_pt = traj[:2, t]
            global_pt = np.dot(R, local_pt) + np.array([state.x, state.y])
            global_traj[0, t] = global_pt[0]
            global_traj[1, t] = global_pt[1]
            global_traj[3, t] = state.yaw + traj[3, t]
        return global_traj

    def display_candidate_trajs(self, candidate_trajs, collisions, chosen_idx, state):
        """ Visualize candidate trajectories with different colors based on collision status. """
        marker_arr = MarkerArray()
        for i, traj in enumerate(candidate_trajs):
            marker = Marker()
            marker.header.frame_id = '/map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "candidate_trajs"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            if chosen_idx is not None and i == chosen_idx:
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0
            elif collisions[i]:
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0
            else:
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 1.0
            global_traj = self.convert_to_global(traj, state)
            for t in range(global_traj.shape[1]):
                pt = Point()
                pt.x = float(global_traj[0, t])
                pt.y = float(global_traj[1, t])
                pt.z = 0.2
                marker.points.append(pt)
            marker_arr.markers.append(marker)
        self.candidate_pub.publish(marker_arr)

    def setup_parameters(self):
    
        # Declare parameters for the MPC controller
        self.declare_parameter("real_environment")
        self.declare_parameter("csv_file")
        self.declare_parameter("input_cost_accel")
        self.declare_parameter("input_cost_steering")
        self.declare_parameter("input_rate_cost_accel")
        self.declare_parameter("input_rate_cost_steering")
        self.declare_parameter("state_cost_x")
        self.declare_parameter("state_cost_y")
        self.declare_parameter("state_cost_v")
        self.declare_parameter("state_cost_yaw")
        self.declare_parameter("final_state_cost_x")
        self.declare_parameter("final_state_cost_y")
        self.declare_parameter("final_state_cost_v")
        self.declare_parameter("final_state_cost_yaw")
        self.declare_parameter("state_dim")
        self.declare_parameter("input_dim")
        self.declare_parameter("horizon")
        self.declare_parameter("search_idx")
        self.declare_parameter("dt")
        self.declare_parameter("step_length")
        self.declare_parameter("veh_length")
        self.declare_parameter("veh_width")
        self.declare_parameter("wheelbase")
        self.declare_parameter("min_steer")
        self.declare_parameter("max_steer")
        self.declare_parameter("max_steer_rate")
        self.declare_parameter("max_speed")
        self.declare_parameter("min_speed")
        self.declare_parameter("max_accel")
        self.declare_parameter("downsampling")
        self.declare_parameter("max_sight")
        self.declare_parameter("gap_threshold")


    # -----------------------------------------------------------
    # Reactive Obstacle Detection Functions
    # -----------------------------------------------------------
    def preprocess_laser_data(self, ranges):
        num_bins = int(len(ranges) / self.downsample_gap)
        proc = np.zeros(num_bins)
        for i in range(num_bins):
            window = ranges[i * self.downsample_gap : (i + 1) * self.downsample_gap]
            proc[i] = sum(window) / self.downsample_gap
        proc = np.clip(proc, 0.0, self.max_sight)
        return proc

    def get_max_gap(self, free_ranges):
        longest = 0
        current = 0
        end_idx = 0
        start_idx = 0
        safe_dist = 0.7 + 0.4 * self.current_speed
        print(f"Safe distance: {safe_dist:.2f}")
        for i in range(len(free_ranges)):
            if free_ranges[i] > safe_dist:
                current += 1
                if current > longest:
                    longest = current
                    end_idx = i + 1
                    start_idx = end_idx - longest
            else:
                current = 0
        return start_idx, end_idx

    # -----------------------------------------------------------
    # Visualization Functions
    # -----------------------------------------------------------
    def show_waypoints(self):
        self.wpts_marker.points = []
        self.wpts_marker.header.frame_id = '/map'
        self.wpts_marker.type = Marker.POINTS
        self.wpts_marker.color.g = 0.75
        self.wpts_marker.color.a = 1.0
        self.wpts_marker.scale.x = 0.05
        self.wpts_marker.scale.y = 0.05
        self.wpts_marker.id = 0
        for i in range(self.waypoints.shape[0]):
            pt = Point(x=self.waypoints[i, 1], y=self.waypoints[i, 2], z=0.1)
            self.wpts_marker.points.append(pt)
        self.wpts_pub.publish(self.wpts_marker)

    def show_reference_traj(self, ref_traj):
        self.ref_traj_marker.points = []
        self.ref_traj_marker.header.frame_id = '/map'
        self.ref_traj_marker.type = Marker.LINE_STRIP
        self.ref_traj_marker.color.b = 0.75
        self.ref_traj_marker.color.a = 1.0
        self.ref_traj_marker.scale.x = 0.08
        self.ref_traj_marker.scale.y = 0.08
        self.ref_traj_marker.id = 0
        for i in range(ref_traj.shape[1]):
            pt = Point(x=ref_traj[0, i], y=ref_traj[1, i], z=0.2)
            self.ref_traj_marker.points.append(pt)
        self.ref_traj_pub.publish(self.ref_traj_marker)

    def show_predicted_path(self, pred_path):
        self.pred_path_marker.points = []
        self.pred_path_marker.header.frame_id = '/map'
        self.pred_path_marker.type = Marker.LINE_STRIP
        self.pred_path_marker.color.r = 0.75
        self.pred_path_marker.color.a = 1.0
        self.pred_path_marker.scale.x = 0.08
        self.pred_path_marker.scale.y = 0.08
        self.pred_path_marker.id = 0
        for i in range(pred_path.shape[1]):
            pt = Point(x=pred_path[0, i], y=pred_path[1, i], z=0.2)
            self.pred_path_marker.points.append(pt)
        self.pred_path_pub.publish(self.pred_path_marker)


def main(args=None):
    rclpy.init(args=args)
    print("MPC Node Initialized")
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
