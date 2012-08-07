import numpy as np
import tf.transformations as tr
import rospy

def tf_as_matrix(tup):
    return np.matrix(tr.translation_matrix(tup[0])) * np.matrix(tr.quaternion_matrix(tup[1])) 

def transform(to_frame, from_frame, tf_listener, t=0):
    tf_listener.waitForTransform(to_frame, from_frame, rospy.Time(t), rospy.Duration(10))
    return tf_as_matrix(tf_listener.lookupTransform(to_frame, from_frame, rospy.Time(t)))

def tf_as_matrix(tup):
    return np.matrix(tr.translation_matrix(tup[0])) * np.matrix(tr.quaternion_matrix(tup[1])) 

def matrix_as_tf(mat):
    return (tr.translation_from_matrix(mat), tr.quaternion_from_matrix(mat))


