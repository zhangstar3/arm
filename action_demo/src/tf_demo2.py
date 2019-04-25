#!/usr/bin/env python  
import roslib
roslib.load_manifest('action_demo')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.3, 0, 0.2),
                         (0, 1, 0, 0.1),
                         rospy.Time.now(),
                         "carrot2",
                         "base_link")
        rate.sleep()

    """
  target_pose1.orientation.w =-0.06789;//0.726282;
  target_pose1.orientation.x =0.076335;//4.04423e-07;
  target_pose1.orientation.y =-0.70385;//-0.687396;
  target_pose1.orientation.z =0.70385;//4.81813e-07;
//0;
  target_pose1.position.x   = 0.6398;//0.0261186;
  target_pose1.position.y   = -0.006899;//4.50972e-07;
  target_pose1.position.z   = 0.40611;//0.573659;

    """