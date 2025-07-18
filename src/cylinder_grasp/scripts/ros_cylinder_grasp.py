#!/usr/bin/env python3

import rospy
import mujoco
import os
import rospkg
import numpy as np
import math
import threading
import time
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse


class CylinderGraspConfig:
    """Configuration parameters for the cylinder grasping system"""
    rp = rospkg.RosPack()
    pkg_path = rp.get_path("cylinder_grasp")          # 取得包的根目录
    XML_PATH = os.path.join(pkg_path, "models", "scene_right_cylinder.xml")
    
    # Control tolerances
    ERROR_THRESHOLD = 0.001
    FINGER_ERROR_THRESHOLD = 0.00008
    
    # Cylinder-specific grasp parameters
    RADIUS = 0.05
    BETA_ANGLE = 7  # degrees - angled approach for cylinder
    
    # Contact position offsets for better cylinder grip
    CONTACT_OFFSET = np.array([1, 1, 0]) * 0.0002
    
    # Z-heights for finger contact points on cylinder
    FF_HEIGHT = 0.15
    MF_HEIGHT = 0.10
    RF_HEIGHT = 0.05
    TH_HEIGHT = 0.115
    
    # Hand positioning offset relative to cylinder
    HAND_OFFSET = np.array([-0.102, 0.01, 0])
    
    # Lift parameters
    LIFT_HEIGHT = 0.1
    
    # Cylinder-optimized joint position references for null space control
    FF_JOINT_REF = [0, 1.0, 1.5, 1.5]
    MF_JOINT_REF = [0, 1.0, 1.5, 1.5]
    RF_JOINT_REF = [0, 1.0, 1.5, 1.5]
    TH_JOINT_REF = [1.4, 0, 0.8, 0.8]  # Different thumb config for cylinder
    
    # Null space control weights
    ALPHA_FF = 0.3
    ALPHA_MF = 0.3
    ALPHA_RF = 0.3
    ALPHA_TH = 0.2


class PIDController:
    """PID Controller for position control"""
    
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_error = 0
        self.prev_error = 0

    def compute_control_signals(self, error):
        """Compute PID control signals"""
        # Proportional term
        p = self.kp * error

        # Integral term
        self.integral_error += error
        i = self.ki * self.integral_error

        # Derivative term
        derivative_error = error - self.prev_error
        d = self.kd * derivative_error

        # Calculate total control signals
        control_signals = p + i + d

        # Update previous error for next iteration
        self.prev_error = error.copy()

        return control_signals


class CylinderGraspingController:
    """Cylinder-specific grasping control system"""
    
    def __init__(self):
        """Initialize the ROS cylinder grasping controller"""
        rospy.init_node('cylinder_grasping_controller', anonymous=True)
        
        # Initialize MuJoCo
        self.config = CylinderGraspConfig()
        self.model = mujoco.MjModel.from_xml_path(self.config.XML_PATH)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_resetData(self.model, self.data)
        
        # Initialize control state flags
        self.reset_flags()
        
        # Initialize PID controllers for different phases
        self.init_controllers()
        
        # Calculate target contact positions for cylinder grasping
        self.calculate_cylinder_target_positions()
        
        # Get body indices for Jacobian computation
        self.get_body_indices()
        
        # Initialize Jacobian matrices
        self.init_jacobians()
        
        # ROS Publishers
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.cylinder_pose_pub = rospy.Publisher('/cylinder_pose', Pose, queue_size=10)
        self.status_pub = rospy.Publisher('/grasp_status', Bool, queue_size=10)
        self.debug_pub = rospy.Publisher('/debug_info', Float64MultiArray, queue_size=10)
        
        # ROS Services
        self.start_grasp_srv = rospy.Service('/start_grasp', Empty, self.start_grasp_callback)
        self.reset_grasp_srv = rospy.Service('/reset_grasp', Empty, self.reset_grasp_callback)
        self.stop_grasp_srv = rospy.Service('/stop_grasp', Empty, self.stop_grasp_callback)
        self.start_viewer_srv = rospy.Service('/start_viewer', Empty, self.start_viewer_callback)
        self.stop_viewer_srv = rospy.Service('/stop_viewer', Empty, self.stop_viewer_callback)
        
        # Threading
        self.control_running = False
        self.simulation_running = False
        self.viewer_running = False
        
        # Start simulation
        self.start_simulation()
        
        rospy.loginfo("Cylinder Grasping Controller initialized")

    def reset_flags(self):
        """Reset all control phase flags"""
        self.open_flag = False
        self.init_flag = True
        self.grasp_flag = False
        self.lift_flag = False
        self.ff_flag = False
        self.mf_flag = False
        self.rf_flag = False
        self.th_flag = False

    def init_controllers(self):
        """Initialize PID controllers optimized for cylinder grasping"""
        # Hand opening controller
        self.open_pid = PIDController(4, 0.08, 0.01)
        
        # Initial positioning controller
        self.initial_pid = PIDController(1, 0.001, 0.01)
        
        # Finger controllers - same gains as original
        self.ff_pid = PIDController(1.8, 0.18, 0.01)
        self.mf_pid = PIDController(1.8, 0.2, 0.01)
        self.rf_pid = PIDController(1.8, 0.18, 0.01)
        self.th_pid = PIDController(0.5, 0.1, 0.01)  # Lower gains for thumb on cylinder
        
        # Target lifting controller
        self.target_pid = PIDController(0.02, 0.0005, 0.0001)

    def calculate_cylinder_target_positions(self):
        """Calculate target contact positions for cylinder grasping"""
        sin_beta = math.sin(self.config.BETA_ANGLE * math.pi / 180)
        cos_beta = math.cos(self.config.BETA_ANGLE * math.pi / 180)
        
        # Target positions with angled approach for cylinder surface
        base_ff = np.array([
            -self.config.RADIUS * sin_beta,
            -self.config.RADIUS * cos_beta,
            self.config.FF_HEIGHT
        ])
        
        base_mf = np.array([
            -self.config.RADIUS * sin_beta,
            -self.config.RADIUS * cos_beta,
            self.config.MF_HEIGHT
        ])
        
        base_rf = np.array([
            -self.config.RADIUS * sin_beta,
            -self.config.RADIUS * cos_beta,
            self.config.RF_HEIGHT
        ])
        
        base_th = np.array([
            self.config.RADIUS * sin_beta,
            self.config.RADIUS * cos_beta,
            self.config.TH_HEIGHT
        ])
        
        # Apply contact offsets for better cylinder grip
        self.ff_contact_target = base_ff + self.config.CONTACT_OFFSET
        self.mf_contact_target = base_mf + self.config.CONTACT_OFFSET
        self.rf_contact_target = base_rf + self.config.CONTACT_OFFSET
        self.th_contact_target = base_th - self.config.CONTACT_OFFSET  # Opposite offset for thumb

    def get_body_indices(self):
        """Get body indices for Jacobian computation"""
        self.ff_tip_idx = self.model.body('ff_tip').id
        self.mf_tip_idx = self.model.body('mf_tip').id
        self.rf_tip_idx = self.model.body('rf_tip').id
        self.th_tip_idx = self.model.body('th_tip').id
        self.cylinder_object_idx = self.model.body('cylinder_object').id

    def init_jacobians(self):
        """Initialize Jacobian matrices for each finger"""
        self.ff_contact_jacp = np.zeros((3, self.model.nv))
        self.ff_contact_jacr = np.zeros((3, self.model.nv))
        self.mf_contact_jacp = np.zeros((3, self.model.nv))
        self.mf_contact_jacr = np.zeros((3, self.model.nv))
        self.rf_contact_jacp = np.zeros((3, self.model.nv))
        self.rf_contact_jacr = np.zeros((3, self.model.nv))
        self.th_contact_jacp = np.zeros((3, self.model.nv))
        self.th_contact_jacr = np.zeros((3, self.model.nv))

    def start_simulation(self):
        """Start the simulation thread"""
        self.simulation_running = True
        self.sim_thread = threading.Thread(target=self.simulation_loop)
        self.sim_thread.daemon = True
        self.sim_thread.start()
        rospy.loginfo("Simulation started")

    def simulation_loop(self):
        """Main simulation loop"""
        rate = rospy.Rate(500)
        
        while self.simulation_running and not rospy.is_shutdown():
            # Step simulation
            mujoco.mj_step(self.model, self.data)
            
            # Update kinematics
            mujoco.mj_kinematics(self.model, self.data)
            
            # Publish state
            self.publish_state()
            
            rate.sleep()

    def publish_state(self):
        """Publish current state"""
        # Joint states
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [f'joint_{i}' for i in range(len(self.data.qpos))]
        joint_state.position = self.data.qpos.tolist()
        joint_state.velocity = self.data.qvel.tolist()
        joint_state.effort = [0.0] * len(self.data.qpos)
        self.joint_state_pub.publish(joint_state)
        
        # Cylinder pose
        cylinder_pose = Pose()
        cylinder_pos = self.data.body('cylinder_object').xpos
        cylinder_quat = self.data.body('cylinder_object').xquat
        
        cylinder_pose.position.x = cylinder_pos[0]
        cylinder_pose.position.y = cylinder_pos[1]
        cylinder_pose.position.z = cylinder_pos[2]
        cylinder_pose.orientation.w = cylinder_quat[0]
        cylinder_pose.orientation.x = cylinder_quat[1]
        cylinder_pose.orientation.y = cylinder_quat[2]
        cylinder_pose.orientation.z = cylinder_quat[3]
        
        self.cylinder_pose_pub.publish(cylinder_pose)
        
        # Grasp status
        status = Bool()
        status.data = self.control_running
        self.status_pub.publish(status)

    def start_grasp_callback(self, req):
        """Start grasp service callback"""
        if not self.control_running:
            self.reset_flags()
            # self.open_flag = True
            self.control_running = True
            self.control_thread = threading.Thread(target=self.control_loop)
            self.control_thread.daemon = True
            self.control_thread.start()
            rospy.loginfo("Grasp started")
        return EmptyResponse()

    def reset_grasp_callback(self, req):
        """Reset grasp service callback"""
        self.control_running = False
        mujoco.mj_resetData(self.model, self.data)
        self.reset_flags()
        rospy.loginfo("System reset")
        return EmptyResponse()

    def stop_grasp_callback(self, req):
        """Stop grasp service callback"""
        self.control_running = False
        rospy.loginfo("Grasp stopped")
        return EmptyResponse()

    def start_viewer_callback(self, req):
        """Start viewer service callback"""
        if not self.viewer_running:
            self.viewer_running = True
            self.viewer_thread = threading.Thread(target=self.viewer_loop)
            self.viewer_thread.daemon = True
            self.viewer_thread.start()
            rospy.loginfo("Viewer started")
        return EmptyResponse()

    def stop_viewer_callback(self, req):
        """Stop viewer service callback"""
        self.viewer_running = False
        rospy.loginfo("Viewer stopped")
        return EmptyResponse()

    def viewer_loop(self):
        """MuJoCo viewer loop"""
        try:
            import mujoco.viewer as viewer_module
            with viewer_module.launch_passive(self.model, self.data) as viewer:
                while self.viewer_running and not rospy.is_shutdown():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1
                    viewer.sync()
                    time.sleep(0.01)
        except Exception as e:
            rospy.logwarn(f"Viewer error: {e}")
            rospy.loginfo("Viewer not available, continuing without visualization")
        finally:
            self.viewer_running = False

    def control_loop(self):
        """Main control loop"""
        rate = rospy.Rate(100)
        
        while self.control_running and not rospy.is_shutdown():
            if self.open_flag:
                self.control_hand_opening()
            elif self.init_flag:
                self.control_initial_positioning()
            elif self.grasp_flag:
                self.control_cylinder_grasping()
            elif self.lift_flag:
                self.control_lifting()
            
            rate.sleep()

    def control_hand_opening(self):
        """Control phase 1: Open the hand - 完全按照原版逻辑"""
        distance_to_open = np.array([1.2]) - self.data.qpos[15]
        control_signal = self.open_pid.compute_control_signals(distance_to_open)
        
        self.data.ctrl[15] = control_signal[0]
        
        rospy.loginfo_throttle(1, f"Opening hand: {distance_to_open[0]:.4f}")
        
        if distance_to_open < 0.005:
            self.open_flag = False
            self.init_flag = True
            rospy.loginfo("Hand opened, starting positioning")

    def control_initial_positioning(self):
        """Control phase 2: Position hand near cylinder object - 完全按照原版逻辑"""
        hand_pos = self.data.body('palm').xpos
        target_pos = self.data.body('cylinder_object').xpos
        
        # Calculate desired offset from cylinder
        distance_to_target = target_pos - hand_pos + self.config.HAND_OFFSET
        control_signals = self.initial_pid.compute_control_signals(distance_to_target)
        
        self.data.ctrl[0:3] = control_signals
        
        rospy.loginfo_throttle(1, f"Positioning: {np.linalg.norm(distance_to_target):.4f}")
        
        # Check if positioning is complete
        if np.sum(distance_to_target**2) < self.config.ERROR_THRESHOLD:
            self.init_flag = False
            self.grasp_flag = True
            rospy.loginfo("Positioned, starting grasp")

    def control_cylinder_grasping(self):
        """Control phase 3: Execute cylinder grasping motion - 完全按照原版逻辑"""
        # Get current finger contact positions
        ff_pos = self.data.site('ff_contact').xpos
        mf_pos = self.data.site('mf_contact').xpos
        rf_pos = self.data.site('rf_contact').xpos
        th_pos = self.data.site('th_contact').xpos

        # Calculate position errors for each finger
        ff_error = self.ff_contact_target - ff_pos
        mf_error = self.mf_contact_target - mf_pos
        rf_error = self.rf_contact_target - rf_pos
        th_error = self.th_contact_target - th_pos

        # Check if each finger has reached its target
        if np.sum(ff_error**2) < self.config.FINGER_ERROR_THRESHOLD:
            self.ff_flag = True
        if np.sum(mf_error**2) < self.config.FINGER_ERROR_THRESHOLD:
            self.mf_flag = True
        if np.sum(rf_error**2) < self.config.FINGER_ERROR_THRESHOLD:
            self.rf_flag = True
        if np.sum(th_error**2) < self.config.FINGER_ERROR_THRESHOLD:
            self.th_flag = True

        # Check if all fingers are in contact with cylinder
        if all([self.ff_flag, self.mf_flag, self.rf_flag, self.th_flag]):
            self.grasp_flag = False
            self.lift_flag = True
            self.target_final_pos = self.data.body("palm").xpos[2] + self.config.LIFT_HEIGHT
            rospy.loginfo("Grasp completed, starting lift")

        # Compute control signals in task space
        ff_control = self.ff_pid.compute_control_signals(ff_error)
        mf_control = self.mf_pid.compute_control_signals(mf_error)
        rf_control = self.rf_pid.compute_control_signals(rf_error)
        th_control = self.th_pid.compute_control_signals(th_error)

        # Compute Jacobians for each finger
        mujoco.mj_jac(self.model, self.data, self.ff_contact_jacp, self.ff_contact_jacr, 
                     self.data.site('ff_contact').xpos, self.ff_tip_idx)
        mujoco.mj_jac(self.model, self.data, self.mf_contact_jacp, self.mf_contact_jacr, 
                     self.data.site('mf_contact').xpos, self.mf_tip_idx)
        mujoco.mj_jac(self.model, self.data, self.rf_contact_jacp, self.rf_contact_jacr, 
                     self.data.site('rf_contact').xpos, self.rf_tip_idx)
        mujoco.mj_jac(self.model, self.data, self.th_contact_jacp, self.th_contact_jacr, 
                     self.data.site('th_contact').xpos, self.th_tip_idx)

        # Reshape Jacobians
        self.ff_contact_jacp = self.ff_contact_jacp.reshape((3, self.model.nv))
        self.mf_contact_jacp = self.mf_contact_jacp.reshape((3, self.model.nv))
        self.rf_contact_jacp = self.rf_contact_jacp.reshape((3, self.model.nv))
        self.th_contact_jacp = self.th_contact_jacp.reshape((3, self.model.nv))

        # Convert task space control to joint space with cylinder-specific null space control
        joint_controls = self.compute_cylinder_joint_space_control(
            ff_control, mf_control, rf_control, th_control
        )

        # Apply joint space controls
        self.data.ctrl[3:] = joint_controls

        # Publish debug info
        debug_msg = Float64MultiArray()
        debug_msg.data = [np.sum(ff_error**2), np.sum(mf_error**2), np.sum(rf_error**2), np.sum(th_error**2)]
        self.debug_pub.publish(debug_msg)

        rospy.loginfo_throttle(1, f"Grasping - FF:{np.sum(ff_error**2):.6f}, MF:{np.sum(mf_error**2):.6f}, RF:{np.sum(rf_error**2):.6f}, TH:{np.sum(th_error**2):.6f}")

    def compute_cylinder_joint_space_control(self, ff_control, mf_control, rf_control, th_control):
        """Convert task space control signals to joint space - 完全按照原版逻辑"""
        # Compute pseudo-inverse Jacobians
        J_pinv_ff = np.linalg.pinv(self.ff_contact_jacp)
        J_pinv_mf = np.linalg.pinv(self.mf_contact_jacp)
        J_pinv_rf = np.linalg.pinv(self.rf_contact_jacp)
        J_pinv_th = np.linalg.pinv(self.th_contact_jacp)

        # Initialize null space vectors for cylinder grasping
        H_q_ff = np.zeros((self.model.nv, 1))
        H_q_mf = np.zeros((self.model.nv, 1))
        H_q_rf = np.zeros((self.model.nv, 1))
        H_q_th = np.zeros((self.model.nv, 1))

        # Compute null space vectors using cylinder-optimized joint references
        H_q_ff[3:7] = (self.data.qpos[3:7].reshape(4, 1) - 
                       np.array(self.config.FF_JOINT_REF).reshape(4, 1))
        H_q_mf[7:11] = (self.data.qpos[7:11].reshape(4, 1) - 
                        np.array(self.config.MF_JOINT_REF).reshape(4, 1))
        H_q_rf[11:15] = (self.data.qpos[11:15].reshape(4, 1) - 
                         np.array(self.config.RF_JOINT_REF).reshape(4, 1))
        H_q_th[15:19] = (self.data.qpos[15:19].reshape(4, 1) - 
                         np.array(self.config.TH_JOINT_REF).reshape(4, 1))

        # Compute joint space control with null space projection
        ff_joint = (J_pinv_ff @ ff_control - 
                   (self.config.ALPHA_FF * (np.eye(self.model.nv) - 
                    J_pinv_ff @ self.ff_contact_jacp) @ H_q_ff).reshape(self.model.nv,))
        
        mf_joint = (J_pinv_mf @ mf_control - 
                   (self.config.ALPHA_MF * (np.eye(self.model.nv) - 
                    J_pinv_mf @ self.mf_contact_jacp) @ H_q_mf).reshape(self.model.nv,))
        
        rf_joint = (J_pinv_rf @ rf_control - 
                   (self.config.ALPHA_RF * (np.eye(self.model.nv) - 
                    J_pinv_rf @ self.rf_contact_jacp) @ H_q_rf).reshape(self.model.nv,))
        
        th_joint = (J_pinv_th @ th_control - 
                   (self.config.ALPHA_TH * (np.eye(self.model.nv) - 
                    J_pinv_th @ self.th_contact_jacp) @ H_q_th).reshape(self.model.nv,))

        # Extract relevant joint controls
        joint_controls = np.concatenate([
            ff_joint[3:7],
            mf_joint[7:11],
            rf_joint[11:15],
            th_joint[15:19]
        ])

        return joint_controls

    def control_lifting(self):
        """Control phase 4: Lift the grasped cylinder - 完全按照原版逻辑"""
        current_z = self.data.body('palm').xpos[2]
        distance_to_target = self.target_final_pos - current_z
        
        control_signal = self.target_pid.compute_control_signals(distance_to_target)
        self.data.ctrl[2] = control_signal
        
        rospy.loginfo_throttle(1, f"Lifting: {distance_to_target:.4f}")
        
        if distance_to_target < 0.0001:
            self.lift_flag = False
            self.control_running = False
            rospy.loginfo("Lift completed")

    def print_debug_info(self):
        """Print debug information about finger positions"""
        rospy.loginfo("Debug Info:")
        rospy.loginfo(f"  ff_contact: {self.data.site('ff_contact').xpos}")
        rospy.loginfo(f"  mf_contact: {self.data.site('mf_contact').xpos}")
        rospy.loginfo(f"  rf_contact: {self.data.site('rf_contact').xpos}")
        rospy.loginfo(f"  th_contact: {self.data.site('th_contact').xpos}")
        rospy.loginfo("Targets:")
        rospy.loginfo(f"  ff_target: {self.ff_contact_target}")
        rospy.loginfo(f"  mf_target: {self.mf_contact_target}")
        rospy.loginfo(f"  rf_target: {self.rf_contact_target}")
        rospy.loginfo(f"  th_target: {self.th_contact_target}")

    def run(self):
        """Main run function"""
        rospy.loginfo("Controller ready. Available services:")
        rospy.loginfo("  rosservice call /start_grasp")
        rospy.loginfo("  rosservice call /start_viewer")
        rospy.loginfo("  rosservice call /reset_grasp")
        rospy.loginfo("  rosservice call /stop_grasp")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
        finally:
            self.control_running = False
            self.simulation_running = False
            self.viewer_running = False


if __name__ == '__main__':
    try:
        controller = CylinderGraspingController()
        controller.run()
    except Exception as e:
        rospy.logerr(f"Controller error: {e}")