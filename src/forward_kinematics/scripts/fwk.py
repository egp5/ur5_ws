#!/usr/bin/env python

import numpy

import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf.msg
from urdf_parser_py.urdf import URDF

""" Starting from a computed transform T, creates a message that can be
communicated over the ROS wire. In addition to the transform itself, the message
also specifies who is related by this transform, the parent and the child."""
def convert_to_message(T, child, parent):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child
    translation = tf.transformations.translation_from_matrix(T)
    rotation = tf.transformations.quaternion_from_matrix(T)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    return t

#Our main class for computing Forward Kinematics
class ForwardKinematics(object):


    #Initialization
    def __init__(self):
        """Announces that it will publish forward kinematics results, in the form of tfMessages.
        "tf" stands for "transform library", it's ROS's way of communicating around information
        about where things are in the world"""
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)


    """This function is called every time the robot publishes its joint values. We must use
    the information we get to compute forward kinematics.

    We will iterate through the entire chain, and publish the transform for each link we find.
    """
    def callback(self, joint_values):
        # First, we must extract information about the kinematics of the robot from its URDF.
        # We will start at the root and add links and joints to lists
        link_name = self.robot.get_root()
        link_names = []
        joints = []
        while True:
            # Find the joint connected at the end of this link, or its "child"
            # Make sure this link has a child
            if link_name not in self.robot.child_map:
                break
            # Make sure it has a single child (we don't deal with forks)
            if len(self.robot.child_map[link_name]) != 1:
                rospy.logerror("Forked kinematic chain!");
                break
            # Get the name of the child joint, as well as the link it connects to
            (joint_name, next_link_name) = self.robot.child_map[link_name][0]
            # Get the actual joint based on its name
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                break;
            joints.append(self.robot.joint_map[joint_name])
            link_names.append(next_link_name)

            # Move to the next link
            link_name = next_link_name

        # Compute all the transforms based on the information we have
        all_transforms = self.compute_transforms(link_names, joints, joint_values)

        # Publish all the transforms
        self.pub_tf.publish(all_transforms)



    def compute_transforms(self, link_names, joints, joint_values):
        """ Print all the parameters passed to understand the structure of the urdf file
        i=0
        for link in link_names:
            if(i==0):
                print i,")link:",link
                print i,")joints[i].name:",joints[i].name,":",joints[i].type
                i+=1
            elif (i<=6):
                print i,")link:",link
                print i,")joints[i].name:",joints[i].name,":",joints[i].type
                print i,")joint_values.name[i]:",joint_values.name[i-1]
                i+=1
            else:
                print i,")link:",link
                print i,")joints[i].name:",joints[i].name,":",joints[i].type
        """

        # FORWARD KINEMATICS FOR UR5
        all_transforms = tf.msg.tfMessage()
        # We start with the identity
        T = tf.transformations.identity_matrix() # T is a 4x4 matrix
        i = 0 #index of current joint

        for link in link_names:

            if i == 0:
                T = tf.transformations.translation_matrix(joints[0].origin.xyz)
                print "T0:",T
                all_transforms.transforms.append(convert_to_message(T, link, 'world_link'))
                i += 1
            elif i <= 6:
                print "T",i,":",T
                index_of_q_i = joint_values.name.index(joints[i].name)
                q_i = joint_values.position[index_of_q_i]
                T_w2i_joint_rotation_matrix = tf.transformations.rotation_matrix(q_i, joints[i].axis)
                ri=joints[i].origin.rpy[0]
                pi=joints[i].origin.rpy[1]
                yi=joints[i].origin.rpy[2]
                T_w2i_link_rotation_matrix = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(ri,pi,yi))
                T_w2i_translation_matrix = tf.transformations.translation_matrix(joints[i].origin.xyz)
                T_w2i = tf.transformations.concatenate_matrices(T_w2i_translation_matrix, T_w2i_link_rotation_matrix, T_w2i_joint_rotation_matrix)
                T = tf.transformations.concatenate_matrices(T, T_w2i)
                all_transforms.transforms.append(convert_to_message(T, link, "world_link"))
                i += 1
            else:
                print "T",i,":",T
                T_w2i_translation_matrix = tf.transformations.translation_matrix(joints[7].origin.xyz)
                r7=joints[7].origin.rpy[0]
                p7=joints[7].origin.rpy[1]
                y7=joints[7].origin.rpy[2]
                T_w2i_link_rotation_matrix = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(r7,p7,y7))
                T_w2i = tf.transformations.concatenate_matrices(T_w2i_translation_matrix, T_w2i_link_rotation_matrix)
                T = tf.transformations.concatenate_matrices(T, T_w2i)
                all_transforms.transforms.append(convert_to_message(T, link, "world_link"))

        return all_transforms
        """
        print joints[7].origin.rpy[2]
        """
if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=False)
    fwk = ForwardKinematics()
    rospy.spin()
