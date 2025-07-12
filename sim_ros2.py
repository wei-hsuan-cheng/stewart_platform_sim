import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import modern_robotics as mr
from scipy.spatial.transform import Rotation 
from collections import deque
import time

class StewartPlatformSim(Node):
    def __init__(self):
        super().__init__('stewart_platform_sim')
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.leg_length_pub = self.create_publisher(Float64MultiArray, '/stewart_platform/leg_length', 10)
        self.leg_velocity_pub = self.create_publisher(Float64MultiArray, '/stewart_platform/leg_velocity', 10)
        self.marker_pubs = []
        for i in range(10):
            pub = self.create_publisher(Marker, f'/stewart_platform/marker_{i}', 10)
            self.marker_pubs.append(pub)
        
        self.fs = 60 # [Hz]
        self.Ts = 1 / self.fs # [s]
        self.timer = self.create_timer(self.Ts, self.timer_callback)
        
        # Start time and time elapsed
        self.start_time = time.time()
        self.t_ = 0
        
        # Buffer for IK target pose and solution
        self.window_size = 2
        self.T_b_e_buffer = deque(maxlen=self.window_size)
        self.s_buffer = deque(maxlen=self.window_size)

        # Mechanical parameters
        self.base_platform_radius = 0.2  # [m] radius of the equilateral triangle
        self.base_platform_side_length = self.base_platform_radius * np.sqrt(3)  # [m] side length of the equilateral triangle
        self.ee_platform_radius = 0.15
        self.ee_platform_side_length = self.ee_platform_radius * np.sqrt(3)  # [m] side length of the equilateral triangle
        
        # Spherical joint positions in the base platform (fixed)
        self.anchors_world_b, self.positions_b_bsj = self.generate_base_spherical_joint_positions(self.base_platform_radius, 
                                                                                                  -np.pi / 6.0, 
                                                                                                  np.zeros(3)) # -30 [deg] offset for the first vertex (anchor)
        
        # Init IK solution
        self.target_pos_b_e = np.zeros(3)  # Target position of the end-effector in the base frame
        self.target_quat_b_e = np.array([1.0, 0.0, 0.0, 0.0])
        self.s = np.zeros(6)  # Leg length
        self.s_dot = np.zeros(6) # Leg velocity
        
        
    '''
    Helpers
    '''
    def NearZero(self, v, eps=1e-6):
        """Check if a value is near zero."""
        return np.abs(v) < eps
    
    def ErrorPercentage(self, v, gt):
        return ( (v - gt) / gt ) * 100.0
    
    def Norm(self, v):
        v = v.flatten()
        norm = np.linalg.norm(v)
        return norm
        
    def Normalized(self, v):
        norm = self.Norm(v)
        if norm == 0: 
            return v
        return v / norm
    
    def Inv(self, M):
        """Inverse of a matrix"""
        return np.linalg.inv(M)
    
    def Rot2Quat(self, R):
        quat = Rotation.from_matrix(R).as_quat()
        return np.asarray([quat[3], quat[0], quat[1], quat[2]]) # qw, qx, qy, qz
    
    def Quat2Rot(self, quat):
        """Convert quaternion to rotation matrix"""
        quat = quat.flatten() # qw, qx, qy, qz
        return Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix() # qx, qy, qz, qw
    
    def PosQuat2TMat(self, pos, quat):
        """Convert position and quaternion to transformation matrix"""
        R = self.Quat2Rot(quat)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = pos
        return T
    
    def publish_tf(self, parent, child, translation, quat, time):
        t = TransformStamped()
        t.header.stamp = time
        t.header.frame_id = parent
        t.child_frame_id = child
        
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        
        t.transform.rotation.w = quat[0]
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]
        
        self.br.sendTransform(t)
        
    def make_marker(self, marker_id, frame_id, type_, color, scale, points=None, pose=None):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = marker_id
        marker.type = type_
        marker.action = Marker.ADD
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime.sec = 0

        if points is not None:
            marker.points = points
        if pose is not None:
            marker.pose = pose
        return marker
    
    def publish_markers(self):
        radius_color = [0.0, 0.6, 1.0, 1.0]
        leg_color = [1.0, 0.0, 0.0, 1.0]
        tri_color = [0.0, 1.0, 0.0, 1.0]
        scale = [0.01, 0.0, 0.0]  # Line thickness

        R = self.Quat2Rot(self.target_quat_b_e)

        # === Marker 0: Base Circle ===
        base_circle_points = []
        for i in range(33):
            angle = 2 * np.pi * i / 32
            x = self.base_platform_radius * np.cos(angle)
            y = self.base_platform_radius * np.sin(angle)
            base_circle_points.append(Point(x=x, y=y, z=0.0))
        m0 = self.make_marker(0, 'base', Marker.LINE_STRIP, radius_color, scale, base_circle_points)
        self.marker_pubs[0].publish(m0)

        # === Marker 1: EE Circle ===
        ee_circle_points = []
        for i in range(33):
            angle = 2 * np.pi * i / 32
            local = np.array([self.ee_platform_radius * np.cos(angle),
                            self.ee_platform_radius * np.sin(angle), 0.0])
            global_pt = self.target_pos_b_e + R @ local
            ee_circle_points.append(Point(x=global_pt[0], y=global_pt[1], z=global_pt[2]))
        m1 = self.make_marker(1, 'world', Marker.LINE_STRIP, radius_color, scale, ee_circle_points)
        self.marker_pubs[1].publish(m1)

        # === Marker 2: Base Triangle ===
        tri_points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.anchors_world_b]
        tri_points.append(tri_points[0])
        m2 = self.make_marker(2, 'base', Marker.LINE_STRIP, tri_color, scale, tri_points)
        self.marker_pubs[2].publish(m2)

        # === Marker 3: EE Triangle ===
        tri_points = []
        for p in self.anchors_e:
            p_b = self.target_pos_b_e + R @ p
            tri_points.append(Point(x=p_b[0], y=p_b[1], z=p_b[2]))
        tri_points.append(tri_points[0])
        m3 = self.make_marker(3, 'world', Marker.LINE_STRIP, tri_color, scale, tri_points)
        self.marker_pubs[3].publish(m3)

        # === Markers 4–9: Legs ===
        for i in range(6):
            leg_line = [
                Point(x=self.positions_b_bsj[i][0], y=self.positions_b_bsj[i][1], z=self.positions_b_bsj[i][2]),
                Point(x=self.positions_b_esj[i][0], y=self.positions_b_esj[i][1], z=self.positions_b_esj[i][2])
            ]
            m_leg = self.make_marker(4 + i, 'base', Marker.LINE_STRIP, leg_color, scale, leg_line)
            self.marker_pubs[4 + i].publish(m_leg)


    '''
    IK target pose
    '''
    def get_target_pose(self):
        # Target pose (pose_b_e)
        self.d_pos_b_e = np.cos(2 * np.pi * 0.05 * self.t_) * np.array([0.025, 0.025, 0.025])
        self.target_pos_b_e = np.array([0.0, 0.0, self.ee_platform_side_length * 0.75]) + self.d_pos_b_e
        
        self.target_axis_ang_b_e = np.array([0.0, 1.0, 0.0, 
                                             (np.pi / 12.0) * np.sin(2 * np.pi * 0.05 * self.t_),
                                             ])
        self.target_quat_b_e = self.Rot2Quat( mr.MatrixExp3( mr.VecToso3(
                                                                         self.target_axis_ang_b_e[:3] * self.target_axis_ang_b_e[3]
                                                                         ) ) )
        
        self.T_b_e = self.PosQuat2TMat(self.target_pos_b_e, self.target_quat_b_e)
        
        # Update T_b_e buffer
        now = self.get_clock().now().nanoseconds * 1e-9  # seconds
        self.T_b_e_buffer.append((now, self.T_b_e))
        
        # Update time elapsed
        self.t_ += self.Ts
    

    '''
    IK solver
    '''
    def generate_base_spherical_joint_positions(self, radius, angle_offset, pos_world_b):
        """Generate three equilateral triangle vertices."""
        
        # Three vertices (anchor points of the base)
        anchors_world_b = np.zeros((3, 3))
        for i in range(3):
            angle = angle_offset + i * 2 * np.pi / 3
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            anchors_world_b[i] = pos_world_b + np.array([x, y, 0.0])
        
        # Each vertex has two spherical joints near by
        # Unit vectors from each anchors
        vec_anchor_12 = self.Normalized(anchors_world_b[1] - anchors_world_b[0])
        vec_anchor_23 = self.Normalized(anchors_world_b[2] - anchors_world_b[1])
        vec_anchor_31 = self.Normalized(anchors_world_b[0] - anchors_world_b[2])
        
        # Spherical joints
        # pos_b_bsj_1: anchor1 + vec_anchor_13 * 0.1 * self.base_platform_side_length
        # pos_b_bsj_2: anchor1 + vec_anchor_12 * 0.1 * self.base_platform_side_length
        # etc.
        
        alpha = 0.1
        positions_b_bsj = np.zeros((6, 3))
        positions_b_bsj[0] = anchors_world_b[0] - vec_anchor_31 * (alpha * self.base_platform_side_length)
        positions_b_bsj[1] = anchors_world_b[0] + vec_anchor_12 * (alpha * self.base_platform_side_length)
        positions_b_bsj[2] = anchors_world_b[1] - vec_anchor_12 * (alpha * self.base_platform_side_length)
        positions_b_bsj[3] = anchors_world_b[1] + vec_anchor_23 * (alpha * self.base_platform_side_length)
        positions_b_bsj[4] = anchors_world_b[2] - vec_anchor_23 * (alpha * self.base_platform_side_length)
        positions_b_bsj[5] = anchors_world_b[2] + vec_anchor_31 * (alpha * self.base_platform_side_length)
        
        return anchors_world_b, positions_b_bsj
    
    def generate_ee_spherical_joint_positions(self, radius, angle_offset, pos_b_e, quat_b_e):
        """Generate three equilateral triangle vertices."""
        
        # Three vertices (anchor points of the end-effector)
        anchors_e = np.zeros((3, 3))
        anchors_b_e = np.zeros((3, 3))
        for i in range(3):
            angle = angle_offset + i * 2 * np.pi / 3
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            anchors_e[i] = np.array([x, y, 0.0])
            anchors_b_e[i] = pos_b_e + self.Quat2Rot(quat_b_e) @ np.array([x, y, 0.0])
        
        # Each vertex has two spherical joints near by
        # Unit vectors from each anchors
        # w.r.t. end-effector
        vec_anchor_12_e = self.Normalized(anchors_e[1] - anchors_e[0])
        vec_anchor_23_e = self.Normalized(anchors_e[2] - anchors_e[1])
        vec_anchor_31_e = self.Normalized(anchors_e[0] - anchors_e[2])
        # w.r.t. base
        vec_anchor_12_b = self.Normalized(anchors_b_e[1] - anchors_b_e[0])
        vec_anchor_23_b = self.Normalized(anchors_b_e[2] - anchors_b_e[1])
        vec_anchor_31_b = self.Normalized(anchors_b_e[0] - anchors_b_e[2])
        
        # Spherical joints
        # ee_shperical_joint1: anchor1 + vec_anchor_12 * 0.1 * self.ee_platform_side_length
        # ee_shperical_joint2: anchor2 + vec_anchor_21 * 0.1 * self.ee_platform_side_length
        # etc.
        alpha = 0.1
        
        # w.r.t. end-effector
        positions_e_esj = np.zeros((6, 3))
        positions_e_esj[0] = anchors_e[0] + vec_anchor_12_e * (alpha * self.ee_platform_side_length)
        positions_e_esj[1] = anchors_e[1] - vec_anchor_12_e * (alpha * self.ee_platform_side_length)
        positions_e_esj[2] = anchors_e[1] + vec_anchor_23_e * (alpha * self.ee_platform_side_length)
        positions_e_esj[3] = anchors_e[2] - vec_anchor_23_e * (alpha * self.ee_platform_side_length)
        positions_e_esj[4] = anchors_e[2] + vec_anchor_31_e * (alpha * self.ee_platform_side_length)
        positions_e_esj[5] = anchors_e[0] - vec_anchor_31_e * (alpha * self.ee_platform_side_length)
        
        # w.r.t. base
        positions_b_esj = np.zeros((6, 3))
        positions_b_esj[0] = anchors_b_e[0] + vec_anchor_12_b * (alpha * self.ee_platform_side_length)
        positions_b_esj[1] = anchors_b_e[1] - vec_anchor_12_b * (alpha * self.ee_platform_side_length)
        positions_b_esj[2] = anchors_b_e[1] + vec_anchor_23_b * (alpha * self.ee_platform_side_length)
        positions_b_esj[3] = anchors_b_e[2] - vec_anchor_23_b * (alpha * self.ee_platform_side_length)
        positions_b_esj[4] = anchors_b_e[2] + vec_anchor_31_b * (alpha * self.ee_platform_side_length)
        positions_b_esj[5] = anchors_b_e[0] - vec_anchor_31_b * (alpha * self.ee_platform_side_length)
        
        return anchors_e, positions_e_esj, positions_b_esj

    def compute_leg_length(self, positions_b_bsj, positions_b_esj):
        """Calculate leg length based on spherical joint positions."""
        s = np.zeros(6)
        for i in range(6):
            s[i] = self.Norm(positions_b_esj[i] - positions_b_bsj[i])
        
        # Update s (leg_length) buffer
        now = self.get_clock().now().nanoseconds * 1e-9  # seconds
        self.s_buffer.append((now, s))
        
        return s
    
    def verify_leg_length(self):
        """Verify the leg length by TFs."""
        # Compute leg length again using LookUpTF
        leg_length = np.zeros(6)
        for i in range(6):
            # Look up the transforms for base spherical joint and end-effector spherical joint
            t_bsj = self.tf_buffer.lookup_transform('base', f'bsj_{i+1}', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            t_esj = self.tf_buffer.lookup_transform('base', f'esj_{i+1}', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Calculate the leg length
            pos_bsj = np.array([t_bsj.transform.translation.x, 
                                t_bsj.transform.translation.y, 
                                t_bsj.transform.translation.z])
            pos_esj = np.array([t_esj.transform.translation.x, 
                                t_esj.transform.translation.y, 
                                t_esj.transform.translation.z])
            leg_length[i] = self.Norm(pos_esj - pos_bsj)
        
        return leg_length
    
    def solve_ik(self):
        # Spherical joint positions in the end-effector platform (moving)
        self.anchors_e, self.positions_e_esj, self.positions_b_esj = self.generate_ee_spherical_joint_positions(self.ee_platform_radius, 
                                                                                                                -np.pi / 2.0, 
                                                                                                                self.target_pos_b_e, 
                                                                                                                self.target_quat_b_e) # -90 [deg] offset for the first vertex (anchor)

        self.s = self.compute_leg_length(self.positions_b_bsj, self.positions_b_esj)
        
    def publish_ik_results(self):
        msg = Float64MultiArray()
        msg.data = self.s.tolist()
        self.leg_length_pub.publish(msg)
        
        msg = Float64MultiArray()
        msg.data = self.s_dot.tolist()
        self.leg_velocity_pub.publish(msg)

    def broadcast_tf(self):
        now = self.get_clock().now().to_msg()

        # Publish base {b}
        self.publish_tf('world', 'base', np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), now)

        # Publish end-effector {ee}
        self.publish_tf('world', 'ee', self.target_pos_b_e, self.target_quat_b_e, now)

        # Publish spherical joints at base {bsj_i}
        for i in range(len(self.positions_b_bsj)):
            # Publish spherical joints at base {b_spherical_joint_i}
            self.publish_tf('base', f'bsj_{i+1}', self.positions_b_bsj[i], np.array([1.0, 0.0, 0.0, 0.0]), now)

        # Publish spherical joints at end-effector {esj_i} (w.r.t. end-effector)
        for i in range(len(self.positions_e_esj)):
            # Publish spherical joints at end-effector {esj_i}
            self.publish_tf('ee', f'esj_{i+1}', self.positions_e_esj[i], np.array([1.0, 0.0, 0.0, 0.0]), now)
        
    def verify_ik(self):
        """Verify the IK solution by checking the leg length."""
        self.s_verify = self.verify_leg_length()
        self.s_error = np.array([self.ErrorPercentage(self.s[i], self.s_verify[i]) for i in range(len(self.s))])
    
    '''
    Differential kinematics (Jacobian)
    '''
    def InvJacob(self):
        inv_jacob = np.zeros((6, 6))
        for i in range(6):
            n_hat_i = self.Normalized(self.positions_b_esj[i] - self.positions_b_bsj[i])
            q_i = self.positions_b_bsj[i]
            inv_jacob[i] = np.concatenate([-np.cross(n_hat_i, q_i), n_hat_i])

        # return inv_jacob.T
        return inv_jacob
    
    def compute_moving_average_TMat_dot(self):
        if len(self.T_b_e_buffer) < 2:
            return np.zeros((4, 4))
        
        # Use first and last elements in the buffer
        t_oldest, T_oldest = self.T_b_e_buffer[0]
        t_latest, T_latest = self.T_b_e_buffer[-1]
        dt = t_latest - t_oldest
        if dt <= 0:
            return np.zeros((4, 4))
        
        T_dot_avg = (T_latest - T_oldest) / dt
        return T_dot_avg
    
    def compute_mavg_base_twist(self):
        """Estimate moving average base twist"""        
        T_dot = self.compute_moving_average_TMat_dot()
        T_inv = np.linalg.inv(self.T_b_e)
        twist_b = mr.se3ToVec(T_dot @ T_inv)
        return twist_b
    
    def compute_mavg_basee_twist_from_adjoint(self):
        """Estimate moving average basee twist from adjoint"""        
        twist_b = mr.Adjoint( self.T_b_e ) @ self.compute_mavg_ee_twist()
        return twist_b
    
    def compute_mavg_ee_twist(self):
        """Estimate moving average ee twist"""        
        T_dot = self.compute_moving_average_TMat_dot()
        T_inv = np.linalg.inv(self.T_b_e)
        twist_e = mr.se3ToVec(T_inv @ T_dot)
        return twist_e
    
    def compute_mavg_ee_twist_from_adjoint(self):
        """Estimate moving average ee twist from adjoint"""        
        twist_e = mr.Adjoint( self.Inv(self.T_b_e) ) @ self.compute_mavg_base_twist()
        return twist_e
    
    def compute_mavg_s_dot(self):
        if len(self.s_buffer) < 2:
            return np.zeros(len(self.s))
        
        # Use first and last elements in the buffer
        t_oldest, s_oldest = self.s_buffer[0]
        t_latest, s_latest = self.s_buffer[-1]
        dt = t_latest - t_oldest
        if dt <= 0:
            return np.zeros(len(self.s))
        
        s_dot_avg = (s_latest - s_oldest) / dt
        return s_dot_avg

    def verify_jacobian(self):
        """Verify the Jacobian by checking the leg velocity."""
        # Get leg velocity from target twist and Jacobian, s_dot = J_b_inv * twist_b
        self.s_dot = self.InvJacob() @ self.compute_mavg_base_twist()
        self.s_dot_verify = self.compute_mavg_s_dot()
        self.s_dot_error = np.array([self.ErrorPercentage(self.s_dot[i], self.s_dot_verify[i]) for i in range(len(self.s))])

    
    def print_results(self):
        """Print the results of the IK solution and Jacobian verification."""
        
        # print(f"\nEnd-effector pose [m, quat] = {self.target_pos_b_e}, {self.target_quat_b_e}")
        # print(f"\nIK solution (leg length) [m] = {self.s}")
        # print(f"\n--------------------")
        # if any of the leg length error compoent is larger than 5%,
        if np.any(np.abs(self.s_error) > 5.0):  # 5%
            print(f"\nIK error (leg length error) [%] = {self.s_error}")
            print(f"\n--------------------")
        
        # print(f"\nEnd-effector twist [rad/s, m/s] = {self.compute_mavg_ee_twist()}")
        # print(f"\nLeg velocity [m/s] = {self.s_dot}")
        # print(f"\n--------------------")
        if np.any(np.abs(self.s_dot_error) > 5.0):  # 5%
            print(f"\nLeg velocity error [%] = {self.s_dot_error}")
            print(f"\n--------------------")
        
    '''
    Main loop
    '''
    def timer_callback(self):
        self.get_target_pose()
        t1 = time.time()
        self.solve_ik()
        self.publish_ik_results()
        self.broadcast_tf()
        self.publish_markers()
        t2 = time.time()
        print(f"\nIK computation time [ms] = {(t2 - t1) * 1000:.4f}")
        
        
        if time.time() - self.start_time > 0.5:
            self.verify_ik()
            self.verify_jacobian()
            self.print_results()
            
        
def main(args=None):
    rclpy.init(args=args)
    node = StewartPlatformSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
