#!/usr/bin/env python

import math
import numpy
import time
from threading import Thread, Lock

import rospy
import tf
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from urdf_parser_py.urdf import URDF


def S_matrix(w):
    S = numpy.zeros((3, 3))
    S[0, 1] = -w[2]
    S[0, 2] = w[1]
    S[1, 0] = w[2]
    S[1, 2] = -w[0]
    S[2, 0] = -w[1]
    S[2, 1] = w[0]
    return S


def scale_np_array(original_np_array, limit):
    norm = numpy.linalg.norm(original_np_array)
    if norm > limit:
        scaled_np_array = original_np_array * (limit / norm)
        #print "norm of ", original_np_array,"is:", norm
    else:
        scaled_np_array = original_np_array

    return scaled_np_array


# This is the function that must be filled in as part of the Project.
def cartesian_control(joint_transforms, b_T_ee_current, b_T_ee_desired,
                      red_control, q_current, q0_desired):
    num_joints = len(joint_transforms)
    #print "num_joints:", num_joints, "\n"
    current_T_ee_b = numpy.linalg.inv(b_T_ee_current)

    current_T_ee_desired = tf.transformations.concatenate_matrices(
        current_T_ee_b,
        b_T_ee_desired
    )

    EE_dTrans = current_T_ee_desired[:3, 3]

    EE_axis_angle = rotation_from_matrix(current_T_ee_desired)
    #print "EE_axis_angle", EE_axis_angle, "\n"
    EE_dRot = EE_axis_angle[0] * numpy.array(EE_axis_angle[1])
    #print "EE_axis_angle[0]", EE_axis_angle[0],"\n"
    #print "EE_axis_angle[1]", EE_axis_angle[1],"\n"
    #print "EE_dRot", EE_dRot,"\n"

    # P controller - converting to translational and angular velocities
    gain = 5.0
    EE_translational_vel = gain * EE_dTrans
    EE_angular_vel = gain * EE_dRot

    # scale translational and angular velocities
    translational_vel_limit = 1  # 0.1 m/s
    angular_vel_limit = 1.0  # 1.0 rad/s
    #print "EE_translational_vel:", EE_translational_vel, "\n"
    #print "EE_angular_vel:", EE_angular_vel, "\n"
    scaled_EE_translational_vel = scale_np_array(EE_translational_vel, translational_vel_limit)
    scaled_EE_angular_vel = scale_np_array(EE_angular_vel, angular_vel_limit)
    #print "scaled_EE_translational_vel:", scaled_EE_translational_vel, "\n"
    #print "scaled_EE_angular_vel:", scaled_EE_angular_vel, "\n"
    # Combined velocity of end effector
    V_ee = numpy.concatenate([scaled_EE_translational_vel, scaled_EE_angular_vel], axis=0)

    # ####################################################################################
    # #####          Numerically compute - Robot Jacobian           ######################
    # ####################################################################################

    J = []
    #print "joint_transforms:", joint_transforms, "\n"
    for index, base_T_joint in enumerate(joint_transforms):
        #print "J_starting:",J,"\n"
        joint_T_base = numpy.linalg.inv(base_T_joint)
        joint_T_ee = tf.transformations.concatenate_matrices(
            joint_T_base,
            b_T_ee_current
        )

        joint_translation_ee = joint_T_ee[:3, 3]
        ee_Rot_joint = numpy.transpose(joint_T_ee[:3, :3])

        # figuring out last column
        top_right_matrix = numpy.dot(-1 * ee_Rot_joint, S_matrix(joint_translation_ee))

        J_col = numpy.concatenate([top_right_matrix[:, -1], ee_Rot_joint[:, -1]], axis=0)

        J.append(J_col)
        #print "J_append:",J,"\n"
    J = numpy.transpose(numpy.asarray(J))
    #print "J_transposed:",J,"\n"
    safe_pseudoInv_J = numpy.linalg.pinv(J, 0.01)

    dq = numpy.dot(safe_pseudoInv_J, V_ee)
    #print "dq", dq
    # threshold
    dq[dq > 5] = 5
    #print "with threshold", dq
    # ####################################################################################
    # #####          Null space control                             ######################
    # ####################################################################################
    """
    if red_control:
        dq_nullSpace = numpy.zeros(num_joints)
        numpy.put(dq_nullSpace, [0], q0_desired - q_current[0])
        print "dq_nullSpace:", dq_nullSpace, "\n"
        q0_vel = gain * dq_nullSpace
        print "q0_vel:", q0_vel, "\n"
        original_pseudoInv_J = numpy.linalg.pinv(J)
        nullSpace = numpy.subtract(numpy.identity(6), numpy.dot(original_pseudoInv_J, J))
        q0_null_vel = numpy.dot(nullSpace, q0_vel)

        dq = numpy.add(dq, q0_null_vel)
    """

    return dq


def convert_from_message(t):
    trans = tf.transformations.translation_matrix((t.translation.x,
                                                   t.translation.y,
                                                   t.translation.z))
    rot = tf.transformations.quaternion_matrix((t.rotation.x,
                                                t.rotation.y,
                                                t.rotation.z,
                                                t.rotation.w))
    T = numpy.dot(trans, rot)
    return T


# Returns the angle-axis representation of the rotation contained in the input matrix
# Use like this:
# angle, axis = rotation_from_matrix(R)
def rotation_from_matrix(matrix):
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    R33 = R[:3, :3]
    # axis: unit eigen vector of R33 corresponding to eigenvalue of 1
    l, W = numpy.linalg.eig(R33.T)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigen vector corresponding to eigenvalue 1")
    axis = numpy.real(W[:, i[-1]]).squeeze()
    # point: unit eigen vector of R33 corresponding to eigenvalue of 1
    l, Q = numpy.linalg.eig(R)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigen vector corresponding to eigenvalue 1")
    # rotation angle depending on axis
    cosa = (numpy.trace(R33) - 1.0) / 2.0
    if abs(axis[2]) > 1e-8:
        sina = (R[1, 0] + (cosa - 1.0) * axis[0] * axis[1]) / axis[2]
    elif abs(axis[1]) > 1e-8:
        sina = (R[0, 2] + (cosa - 1.0) * axis[0] * axis[2]) / axis[1]
    else:
        sina = (R[2, 1] + (cosa - 1.0) * axis[1] * axis[2]) / axis[0]
    angle = math.atan2(sina, cosa)
    return angle, axis


class CartesianControl(object):
    # Initialization
    def __init__(self):
        # Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        # Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        # Subscribes to command for end-effector pose
        rospy.Subscriber("/cartesian_command", Transform, self.command_callback)

        # Subscribes to command for redundant dof
        #rospy.Subscriber("/redundancy_command", Float32, self.redundancy_callback)

        # Publishes desired joint velocities
        self.pub_vel = rospy.Publisher("/joint_velocities", JointState, queue_size=1)

        # This is where we hold the most recent joint transforms
        self.joint_transforms = []
        self.q_current = []
        self.x_current = tf.transformations.identity_matrix()
        self.R_base = tf.transformations.identity_matrix()
        self.x_target = tf.transformations.identity_matrix()
        self.q0_desired = 0
        self.last_command_time = 0
        self.last_red_command_time = 0

        # Initialize timer that will trigger callbacks
        self.mutex = Lock()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def command_callback(self, command):
        self.mutex.acquire()
        self.x_target = convert_from_message(command)
        self.last_command_time = time.time()
        self.mutex.release()

    """
    def redundancy_callback(self, command):
        self.mutex.acquire()
        self.q0_desired = command.data
        self.last_red_command_time = time.time()
        self.mutex.release()
    """

    def timer_callback(self, event):
        msg = JointState()
        self.mutex.acquire()
        if time.time() - self.last_command_time < 0.5:
            dq_unordered = cartesian_control(self.joint_transforms,
                                   self.x_current, self.x_target,
                                   False, self.q_current, self.q0_desired)
            #print "dq_unordered:", dq_unordered, "\n"
            #send the joint velocities in the right order
            dq_ordered = numpy.zeros(6)
            dq_ordered[0] = dq_unordered[2]
            dq_ordered[1] = dq_unordered[1]
            dq_ordered[2] = dq_unordered[0]
            dq_ordered[3] = dq_unordered[3]
            dq_ordered[4] = dq_unordered[4]
            dq_ordered[5] = dq_unordered[5]
            msg.velocity = dq_ordered
            """
            elif time.time() - self.last_red_command_time < 0.5:
                dq = cartesian_control(self.joint_transforms,
                                       self.x_current, self.x_current,
                                       True, self.q_current, self.q0_desired)
                msg.velocity = dq
            """
        else:
            msg.velocity = numpy.zeros(6)
        self.mutex.release()
        self.pub_vel.publish(msg)

    def joint_callback(self, joint_values):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.mutex.acquire()
        self.joint_transforms = []
        self.q_current = joint_values.position
        #print self.robot.child_map
        self.process_link_recursive(root, T, joint_values)
        self.mutex.release()

    def align_with_z(self, axis):
        T = tf.transformations.identity_matrix()
        z = numpy.array([0, 0, 1])
        x = numpy.array([1, 0, 0])
        dot = numpy.dot(z, axis)
        if dot == 1: return T
        if dot == -1: return tf.transformation.rotation_matrix(math.pi, x)
        rot_axis = numpy.cross(z, axis)
        angle = math.acos(dot)
        return tf.transformations.rotation_matrix(angle, rot_axis)

    def process_link_recursive(self, link, T, joint_values):
        if link not in self.robot.child_map:
            #print "cartesian_control link last:", link, "\n"
            #print "joint_values.name:", joint_values.name,"\n"
            #print "joint_values.position:", joint_values.position,"\n"
            self.x_current = T
            return
        for i in range(0, len(self.robot.child_map[link])):
            #print "cartesian_control link:", link, "\n"
            (joint_name, next_link) = self.robot.child_map[link][i]
            if joint_name not in self.robot.joint_map:
                rospy.logerr("Joint not found in map")
                continue
            current_joint = self.robot.joint_map[joint_name]
            #print current_joint, "\n"
            trans_matrix = tf.transformations.translation_matrix((current_joint.origin.xyz[0],
                                                                  current_joint.origin.xyz[1],
                                                                  current_joint.origin.xyz[2]))
            rot_matrix = tf.transformations.euler_matrix(current_joint.origin.rpy[0],
                                                         current_joint.origin.rpy[1],
                                                         current_joint.origin.rpy[2], 'rxyz')
            origin_T = numpy.dot(trans_matrix, rot_matrix)
            current_joint_T = numpy.dot(T, origin_T)
            if current_joint.type != 'fixed':
                if current_joint.name not in joint_values.name:
                    rospy.logerr("Joint not found in list")
                    continue
                # compute transform that aligns rotation axis with z
                aligned_joint_T = numpy.dot(current_joint_T, self.align_with_z(current_joint.axis))
                self.joint_transforms.append(aligned_joint_T)
                index = joint_values.name.index(current_joint.name)
                angle = joint_values.position[index]
                joint_rot_T = tf.transformations.rotation_matrix(angle,
                                                                 numpy.asarray(current_joint.axis))
                next_link_T = numpy.dot(current_joint_T, joint_rot_T)
            else:
                next_link_T = current_joint_T

            self.process_link_recursive(next_link, next_link_T, joint_values)


if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    cc = CartesianControl()
    rospy.spin()
