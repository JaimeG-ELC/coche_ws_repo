#!/usr/bin/env python3

import math
import numpy as np
from dataclasses import dataclass, field
import cvxpy
from scipy.linalg import block_diag
from scipy.sparse import block_diag, csc_matrix, diags
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

class MPCParameters:
    """
    A class to store parameters for the MPC controller.
    """
    node: Node

    def __post_init__(self):

        # Dimensions
        self.state_dim = 4 # x, y, v, yaw
        self.input_dim = 2 # accel, steering

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
class CarState:
    " A class to store the state of the car. "
    pos_x: float = 0.0 # x position [m]
    pos_y: float = 0.0 # y position [m]
    steer_angle: float = 0.0  # steering angle [rad]
    velocity: float = 0.0   # velocity [m/s]
    heading: float = 0.0   # heading angle [rad]
    yaw_rate: float = 0.0 # yaw rate [rad/s]
    slip_angle: float = 0.0 # slip angle [rad]


class MPCController(Node):
    """
    A Node implementing a kinematic MPC controller.
    """
    def __init__(self):
        super().__init__('mpc_controller')

        # Declare necessary parameters.
        self.setup_parameters()
        self.params = MPCParameters(self)
        self.prev_accel = None
        self.prev_steer = None
        self.prev_steer_rate = None
        self.initialized = False
        self.use_real = self.get_parameter("real_environment").value
        self.map_filename = self.get_parameter("csv_file").value

        # Initialize state variables for callbacks
        self.current_pose = np.array([0.0, 0.0, 0.0])
        self.rot_matrix = np.identity(3)

        # Topics
        pose_topic = "/pf/viz/inferred_pose" if self.use_real else "/ego_racecar/odom"
        drive_topic = "/drive"
        vis_traj_topic = "/ref_traj"
        vis_wpts_topic = "/waypoints"
        vis_pred_topic = "/pred_traj"

        # Subscriber for Pose
        self.pose_sub = self.create_subscription(PoseStamped if self.use_real else Odometry,pose_topic,self.pose_callback,1)

        # Publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.drive_msg = AckermannDriveStamped()
        self.wpts_pub = self.create_publisher(Marker, vis_wpts_topic, 1)
        self.wpts_marker = Marker()
        self.traj_pub = self.create_publisher(Marker, vis_traj_topic, 1)
        self.traj_marker = Marker()
        self.pred_pub = self.create_publisher(Marker, vis_pred_topic, 1)
        self.pred_marker = Marker()

        # Load map waypoints
        map_dir = os.path.abspath(os.path.join('src', 'csv_data'))
        self.waypoints = np.loadtxt(os.path.join(map_dir, self.map_filename + '.csv'), delimiter=',', skiprows=0)
        self.display_waypoints()

        # Initialize MPC problem
        self.initialize_mpc()


    def initialize_mpc(self):

        # Create optimization variables and parameters for the MPC
        self.x_var = cvxpy.Variable((self.params.state_dim, self.params.horizon + 1))
        self.u_var = cvxpy.Variable((self.params.input_dim, self.params.horizon))
        cost_function = 0.0
        constraints = []
        self.x0_param = cvxpy.Parameter((self.params.state_dim,))
        self.x0_param.value = np.zeros((self.params.state_dim,))
        self.ref_traj_param = cvxpy.Parameter((self.params.state_dim, self.params.horizon + 1))
        self.ref_traj_param.value = np.zeros((self.params.state_dim, self.params.horizon + 1))


        # Define cost function from [R, Rd, Q, Qf] matrices
        R_block = block_diag(tuple([self.params.input_cost] * self.params.horizon))
        Rd_block = block_diag(tuple([self.params.input_rate_cost] * (self.params.horizon - 1)))
        Q_blocks = [self.params.state_cost] * self.params.horizon
        Q_blocks.append(self.params.final_state_cost)
        Q_block = block_diag(tuple(Q_blocks))

        cost_function += cvxpy.quad_form(cvxpy.vec(self.u_var), R_block)
        cost_function += cvxpy.quad_form(cvxpy.vec(self.x_var - self.ref_traj_param), Q_block)
        cost_function += cvxpy.quad_form(cvxpy.vec(cvxpy.diff(self.u_var, axis=1)), Rd_block)
        cost_function += cvxpy.quad_form(self.x_var[:, -1] - self.ref_traj_param[:, -1], self.params.final_state_cost)
        
        path_pred = np.zeros((self.params.state_dim, self.params.horizon + 1))

        # Compute model matrices for each time step
        A_mats = []
        B_mats = []
        C_list = []

        for t in range(self.params.horizon):
            A_temp, B_temp, C_temp = self.compute_model_matrices(
                path_pred[2, t], path_pred[3, t], 0.0
            )
            A_mats.append(A_temp)
            B_mats.append(B_temp)
            C_list.extend(C_temp)

        A_sparse = block_diag(tuple(A_mats))
        B_sparse = block_diag(tuple(B_mats))
        C_vector = np.array(C_list)


        # Create sparse matrices for A matrix
        m_A, n_A = A_sparse.shape
        self.A_nz = cvxpy.Parameter(A_sparse.nnz)
        data_A = np.ones(self.A_nz.size)
        rows_A = A_sparse.row * n_A + A_sparse.col
        cols_A = np.arange(self.A_nz.size)
        indexer_A = csc_matrix((data_A, (rows_A, cols_A)), shape=(m_A * n_A, self.A_nz.size))
        self.A_mat = cvxpy.reshape(indexer_A @ self.A_nz, (m_A, n_A), order="C")

        # Create sparse matrices for B matrix
        m_B, n_B = B_sparse.shape
        self.B_nz = cvxpy.Parameter(B_sparse.nnz)
        data_B = np.ones(self.B_nz.size)
        rows_B = B_sparse.row * n_B + B_sparse.col
        cols_B = np.arange(self.B_nz.size)
        indexer_B = csc_matrix((data_B, (rows_B, cols_B)), shape=(m_B * n_B, self.B_nz.size))
        self.B_mat = cvxpy.reshape(indexer_B @ self.B_nz, (m_B, n_B), order="C")
        self.B_nz.value = B_sparse.data

        # Create parameter for C vector
        self.C_param = cvxpy.Parameter(C_vector.shape)
        self.C_param.value = C_vector

        # Define model dynamics constraints
        flat_prev_x = cvxpy.vec(self.x_var[:, :-1])
        flat_next_x = cvxpy.vec(self.x_var[:, 1:])
        dyn_constraint = flat_next_x == self.A_mat @ flat_prev_x + self.B_mat @ cvxpy.vec(self.u_var) + self.C_param
        constraints.append(dyn_constraint)

        # Define constraints on input and state variables
        dsteer = cvxpy.diff(self.u_var[1, :])
        constraints.append(-self.params.max_steer_rate * self.params.dt <= dsteer)
        constraints.append(dsteer <= self.params.max_steer_rate * self.params.dt)

        constraints.append(self.x_var[:, 0] == self.x0_param)

        speed_seq = self.x_var[2, :]
        constraints.append(self.params.min_speed <= speed_seq)
        constraints.append(speed_seq <= self.params.max_speed)

        steer_seq = self.u_var[1, :]
        constraints.append(self.params.min_steer <= steer_seq)
        constraints.append(steer_seq <= self.params.max_steer)

        accel_seq = self.u_var[0, :]
        constraints.append(accel_seq <= self.params.max_accel)


        # Define the MPC problem using CVXPY
        self.mpc_problem = cvxpy.Problem(cvxpy.Minimize(cost_function), constraints)

    def pose_callback(self, msg):

        # Compute rotation matrix from quaternion and get car state
        self.compute_rotation_matrix(msg)
        curr_state = self.get_car_state(msg)

        #Obtain reference trajectory
        ref_traj = self.compute_ref_traj(
            curr_state,
            self.waypoints[:, 1],
            self.waypoints[:, 2],
            self.waypoints[:, 3],
            self.waypoints[:, 5]
        )
        self.display_ref_traj(ref_traj)

        # Initialize state for MPC controller 
        init_state = [curr_state.pos_x, curr_state.pos_y, curr_state.velocity, curr_state.heading]

        # Run MPC controller
        (self.prev_accel,
         self.prev_steer_rate,
         ox, oy, oyaw, ov,
         predicted_path) = self.run_mpc_controller(ref_traj, init_state, self.prev_accel, self.prev_steer_rate)

        # Publish control commands
        steer_out = self.prev_steer_rate[0]
        speed_out = curr_state.velocity + self.prev_accel[0] * self.params.dt
        self.drive_msg.drive.steering_angle = steer_out
        self.drive_msg.drive.speed = speed_out
        self.drive_pub.publish(self.drive_msg)
        #print("Steering Angle ={}, Speed ={}".format(self.drive_msg.drive.steering_angle, self.drive_msg.drive.speed))

        self.wpts_pub.publish(self.wpts_marker)

    def compute_rotation_matrix(self, msg):

        """Compute rotation matrix from quaternion"""

        orientation = msg.pose.orientation if self.use_real else msg.pose.pose.orientation
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.rot_matrix = transform.Rotation.from_quat(quat).as_matrix()

    def get_car_state(self, msg):

        """Get car state from odometry message"""

        state = CarState()
        state.pos_x = msg.pose.position.x if self.use_real else msg.pose.pose.position.x
        state.pos_y = msg.pose.position.y if self.use_real else msg.pose.pose.position.y
        state.velocity = self.drive_msg.drive.speed
        orient = msg.pose.orientation if self.use_real else msg.pose.pose.orientation
        q = [orient.x, orient.y, orient.z, orient.w]
        state.heading = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        return state

    def run_mpc_controller(self, ref_path, init_state, last_a, last_s):

        """Run the MPC controller to compute control commands"""

        if last_a is None or last_s is None:
            last_a = [0.0] * self.params.horizon
            last_s = [0.0] * self.params.horizon

        traj_prediction = self.predict_trajectory(init_state, last_a, last_s, ref_path)
        self.display_pred_traj(traj_prediction)

        mpc_a, mpc_steer, mpc_x, mpc_y, mpc_yaw, mpc_v = self.solve_mpc(ref_path, traj_prediction, init_state)
        return mpc_a, mpc_steer, mpc_x, mpc_y, mpc_yaw, mpc_v, traj_prediction

    def solve_mpc(self, ref_traj, traj_pred, init_state):

        """Solve the MPC problem using CVXPY"""

        self.x0_param.value = init_state

        A_list = []
        B_list = []
        C_list = []
        for t in range(self.params.horizon):
            A_tmp, B_tmp, C_tmp = self.compute_model_matrices(traj_pred[2, t], traj_pred[3, t], 0.0)
            A_list.append(A_tmp)
            B_list.append(B_tmp)
            C_list.extend(C_tmp)

        A_blk = block_diag(tuple(A_list))
        B_blk = block_diag(tuple(B_list))
        C_blk = np.array(C_list)

        self.A_nz.value = A_blk.data
        self.B_nz.value = B_blk.data
        self.C_param.value = C_blk

        self.ref_traj_param.value = ref_traj

        self.mpc_problem.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)

        if (self.mpc_problem.status == cvxpy.OPTIMAL or 
            self.mpc_problem.status == cvxpy.OPTIMAL_INACCURATE):
            ox = np.array(self.x_var.value[0, :]).flatten()
            oy = np.array(self.x_var.value[1, :]).flatten()
            ov = np.array(self.x_var.value[2, :]).flatten()
            oyaw = np.array(self.x_var.value[3, :]).flatten()
            a_profile = np.array(self.u_var.value[0, :]).flatten()
            steer_profile = np.array(self.u_var.value[1, :]).flatten()
        else:
            print("Error: Cannot solve MPC problem.")
            a_profile, steer_profile, ox, oy, oyaw, ov = None, None, None, None, None, None

        return a_profile, steer_profile, ox, oy, oyaw, ov


    def find_nearest_point(self, pt, traj):

        """Find the nearest point on the trajectory to the given point"""

        diffs = traj[1:, :] - traj[:-1, :]
        sq_dists = diffs[:, 0]**2 + diffs[:, 1]**2
        dots = np.empty((traj.shape[0] - 1,))
        for i in range(dots.shape[0]):
            dots[i] = np.dot((pt - traj[i, :]), diffs[i, :])
        t_vals = dots / sq_dists
        t_vals[t_vals < 0.0] = 0.0
        t_vals[t_vals > 1.0] = 1.0
        projections = traj[:-1, :] + (t_vals * diffs.T).T
        dists = np.empty((projections.shape[0],))
        for i in range(dists.shape[0]):
            diff_val = pt - projections[i]
            dists[i] = np.sqrt(np.sum(diff_val * diff_val))
        seg_idx = np.argmin(dists)
        return projections[seg_idx], dists[seg_idx], t_vals[seg_idx], seg_idx

    def compute_ref_traj(self, state, x_coords, y_coords, headings, speeds):

        """Compute the reference trajectory for the MPC controller"""

        ref_traj = np.zeros((self.params.state_dim, self.params.horizon + 1))
        num_pts = len(x_coords)
        _, _, _, nearest_idx = self.find_nearest_point(np.array([state.pos_x, state.pos_y]), np.array([x_coords, y_coords]).T)

        ref_traj[0, 0] = x_coords[nearest_idx]
        ref_traj[1, 0] = y_coords[nearest_idx]
        ref_traj[2, 0] = speeds[nearest_idx]
        ref_traj[3, 0] = headings[nearest_idx]

        travel_dist = abs(state.velocity) * self.params.dt
        delta_idx = travel_dist / self.params.step_length
        delta_idx = 2
        idx_seq = int(nearest_idx) + np.insert(np.cumsum(np.repeat(delta_idx, self.params.horizon)), 0, 0).astype(int)
        idx_seq[idx_seq >= num_pts] -= num_pts
        ref_traj[0, :] = x_coords[idx_seq]
        ref_traj[1, :] = y_coords[idx_seq]
        ref_traj[2, :] = speeds[idx_seq]

        angle_thresh = 4.5
        for i in range(len(headings)):
            if headings[i] - state.heading > angle_thresh:
                headings[i] -= 2 * np.pi
            if state.heading - headings[i] > angle_thresh:
                headings[i] += 2 * np.pi

        ref_traj[3, :] = headings[idx_seq]
        return ref_traj

    def predict_trajectory(self, init_state, accel_seq, steer_seq, ref):

        """Predict the trajectory of the vehicle using the MPC controller"""

        traj = ref * 0.0
        for i, val in enumerate(init_state):
            traj[i, 0] = init_state[i]

        sim_state = CarState(pos_x=init_state[0], pos_y=init_state[1], heading=init_state[3], velocity=init_state[2])
        for (a, s, k) in zip(accel_seq, steer_seq, range(1, self.params.horizon + 1)):
            sim_state = self.simulate_dynamics(sim_state, a, s)
            traj[0, k] = sim_state.pos_x
            traj[1, k] = sim_state.pos_y
            traj[2, k] = sim_state.velocity
            traj[3, k] = sim_state.heading
        return traj

    def simulate_dynamics(self, state, accel, steer_input):

        """Simulate the dynamics of the vehicle"""

        if steer_input >= self.params.max_steer:
            steer_input = self.params.max_steer
        elif steer_input <= -self.params.max_steer:
            steer_input = -self.params.max_steer

        state.pos_x = state.pos_x + state.velocity * math.cos(state.heading) * self.params.dt
        state.pos_y = state.pos_y + state.velocity * math.sin(state.heading) * self.params.dt
        state.heading = state.heading + (state.velocity / self.params.wheelbase) * math.tan(steer_input) * self.params.dt
        state.velocity = state.velocity + accel * self.params.dt

        if state.velocity > self.params.max_speed:
            state.velocity = self.params.max_speed
        elif state.velocity < self.params.min_speed:
            state.velocity = self.params.min_speed

        return state

    def compute_model_matrices(self, vel, ang, steer_in):

        """Compute the model matrices for the MPC controller"""

        A_mat = np.zeros((self.params.state_dim, self.params.state_dim))
        A_mat[0, 0] = 1.0
        A_mat[1, 1] = 1.0
        A_mat[2, 2] = 1.0
        A_mat[3, 3] = 1.0
        A_mat[0, 2] = self.params.dt * math.cos(ang)
        A_mat[0, 3] = -self.params.dt * vel * math.sin(ang)
        A_mat[1, 2] = self.params.dt * math.sin(ang)
        A_mat[1, 3] = self.params.dt * vel * math.cos(ang)
        A_mat[3, 2] = self.params.dt * math.tan(steer_in) / self.params.wheelbase

        B_mat = np.zeros((self.params.state_dim, self.params.input_dim))
        B_mat[2, 0] = self.params.dt
        B_mat[3, 1] = self.params.dt * vel / (self.params.wheelbase * math.cos(steer_in) ** 2)

        C_vec = np.zeros(self.params.state_dim)
        C_vec[0] = self.params.dt * vel * math.sin(ang) * ang
        C_vec[1] = -self.params.dt * vel * math.cos(ang) * ang
        C_vec[3] = -self.params.dt * vel * steer_in / (self.params.wheelbase * math.cos(steer_in) ** 2)

        return A_mat, B_mat, C_vec


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

    """ Visualization methods """

    # Display waypoints
    def display_waypoints(self):
        self.wpts_marker.points = []
        self.wpts_marker.header.frame_id = '/map'
        self.wpts_marker.type = Marker.POINTS
        self.wpts_marker.color.b = 1.0
        self.wpts_marker.color.a = 1.0
        self.wpts_marker.scale.x = 0.05
        self.wpts_marker.scale.y = 0.05
        self.wpts_marker.id = 0
        for i in range(self.waypoints.shape[0]):
            pt = Point(x=self.waypoints[i, 1], y=self.waypoints[i, 2], z=0.1)
            self.wpts_marker.points.append(pt)

    # Display reference trajectory
    def display_ref_traj(self, traj):
        self.traj_marker.points = []
        self.traj_marker.header.frame_id = '/map'
        self.traj_marker.type = Marker.LINE_STRIP
        self.traj_marker.color.g = 1.0
        self.traj_marker.color.a = 1.0
        self.traj_marker.scale.x = 0.08
        self.traj_marker.scale.y = 0.08
        self.traj_marker.id = 0
        for i in range(traj.shape[1]):
            pt = Point(x=traj[0, i], y=traj[1, i], z=0.2)
            self.traj_marker.points.append(pt)
        self.traj_pub.publish(self.traj_marker)

    # Display predicted trajectory
    def display_pred_traj(self, traj):
        self.pred_marker.points = []
        self.pred_marker.header.frame_id = '/map'
        self.pred_marker.type = Marker.LINE_STRIP
        self.pred_marker.color.r = 0.75
        self.pred_marker.color.a = 1.0
        self.pred_marker.scale.x = 0.08
        self.pred_marker.scale.y = 0.08
        self.pred_marker.id = 0
        for i in range(traj.shape[1]):
            pt = Point(x=traj[0, i], y=traj[1, i], z=0.2)
            self.pred_marker.points.append(pt)
        self.pred_pub.publish(self.pred_marker)


def main(args=None):
    rclpy.init(args=args)
    print("MPC Controller Initialized")
    mpc_controller = MPCController()
    rclpy.spin(mpc_controller)
    mpc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
