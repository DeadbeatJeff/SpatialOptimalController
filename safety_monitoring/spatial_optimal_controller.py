#!/usr/bin/env python3
import numpy as np
import osqp
from scipy import sparse

# ROS 2 Client Library Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# Pinocchio Rigid-Body Dynamics Engine Imports
import pinocchio as pin

class SpatialOptimalController(Node):
    def __init__(self):
        super().__init__('spatial_optimal_controller')
        
        # 1. Path to your Onshape-exported URDF file
        self.urdf_path = '/home/jeff/ros2_ws/src/ur_6r_description/urdf/robot.urdf'
        
        # 2. Initialize Pinocchio Robot Model
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.num_dofs = self.model.nv
        self.get_logger().info(f"Loaded model successfully. Active DOFs: {self.num_dofs}")
        
        # 3. ROS 2 Communication Pipeline
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.target_sub = self.create_subscription(PoseStamped, '/motion_target', self.target_callback, 10)
        
        # 4. Controller Gains
        self.kp = 35.0
        self.kd = 12.0
        
        # Internal State Vectors dynamically sized to active DOFs
        self.q = np.zeros(self.num_dofs)
        self.dq = np.zeros(self.num_dofs)
        
        # Default 6D Operational Space Target Pose [x, y, z, roll, pitch, yaw]
        self.z_des = np.array([0.4, 0.0, 0.5, 0.0, 1.57, 0.0])
        self.z_dot_des = np.zeros(6)

    def target_callback(self, msg):
        """Updates the operational space target dynamically via external ROS 2 messages."""
        self.z_des[0] = msg.pose.position.x
        self.z_des[1] = msg.pose.position.y
        self.z_des[2] = msg.pose.position.z

    def joint_callback(self, msg):
        """Triggers every time encoder updates arrive from the robot configuration space."""
        if len(msg.position) >= self.num_dofs:
            self.q = np.array(msg.position[:self.num_dofs])
            self.dq = np.array(msg.velocity[:self.num_dofs])
            
            # Execute optimal control loop calculation
            tau = self.compute_spatial_optimal_control()
            
            # Dynamically print torques based on how many joints are actually available
            torque_strings = [f"J{i+1}: {t:.1f}Nm" for i, t in enumerate(tau)]
            self.get_logger().info(f"Computed Torques -> {', '.join(torque_strings)}")

    def compute_spatial_optimal_control(self):
        # Update Pinocchio state references
        pin.forwardKinematics(self.model, self.data, self.q, self.dq)
        pin.computeJointJacobians(self.model, self.data, self.q)
        
        # Fetch Spatial Position + Orientation of end-effector
        end_effector_idx = self.model.getFrameId("tool0") if self.model.existFrame("tool0") else (self.model.nframes - 1)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[end_effector_idx]
        
        # Extract operational space coordinates
        x_curr = oMf.translation
        rpy_curr = pin.rpy.matrixToRpy(oMf.rotation)
        z_curr = np.concatenate([x_curr, rpy_curr])
        
        # Fetch operational space Jacobian, sliced to match active degrees of freedom
        J_spatial = pin.getFrameJacobian(self.model, self.data, end_effector_idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:6, :self.num_dofs]
        
        # Compute Dynamics Tensors (M is nv x nv, C is nv)
        M = pin.crba(self.model, self.data, self.q)            
        C = pin.nonLinearEffects(self.model, self.data, self.q, self.dq) 
        
        # Construct Task Space Control Law Tracking Error
        error = self.z_des - z_curr
        ddz_des = self.kp * error + self.kd * (self.z_dot_des - J_spatial @ self.dq)
        
        # OSQP Quadratic Programming Optimization Loop Setup
        # System Variables: Vector x = [tau (nv), ddq (nv)] -> total size = 2 * self.num_dofs
        A_dyn = sparse.hstack([-sparse.eye(self.num_dofs), sparse.csc_matrix(M)])
        A_task = sparse.hstack([sparse.csc_matrix((6, self.num_dofs)), sparse.csc_matrix(J_spatial)])
        A_full = sparse.vstack([A_dyn, A_task]).tocsc()
        
        # Stack boundaries
        b_full = np.concatenate([-C, ddz_des])
        b_full = np.nan_to_num(b_full, nan=0.0, posinf=1e4, neginf=-1e4)
        
        # Cost functions penalized across dimensions
        H = sparse.diags([1.0]*self.num_dofs + [0.05]*self.num_dofs, format='csc')
        f = np.zeros(2 * self.num_dofs)
        
        # Instantiate and solve the optimization problem
        prob = osqp.OSQP()
        prob.setup(H, f, A_full, b_full, b_full, verbose=False)
        res = prob.solve()
        
        if res.info.status == 'solved':
            return res.x[:self.num_dofs]
        else:
            return np.zeros(self.num_dofs)

def main(args=None):
    rclpy.init(args=args)
    spatial_controller = SpatialOptimalController()
    print("Launching Live OSQP Spatial Optimal Control Loop Node...")
    try:
        rclpy.spin(spatial_controller)
    except KeyboardInterrupt:
        print("\nShutting down control node gracefully...")
    finally:
        spatial_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
