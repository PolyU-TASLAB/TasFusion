# /**
# * This file is part of tasfusion.
# *
# * Copyright (C) 2025 Trustworthy AI and Autonomous Systems Laboratory, The Hong Kong Polytechnic University (PolyU)
# * Author: Weisong Wen (welson.wen@polyu.edu.hk)
# *
# * tasfusion is free software: you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation, either version 3 of the License, or
# * (at your option) any later version.
# *
# * tasfusion is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with tasfusion. If not, see <http://www.gnu.org/licenses/>.
# */

#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path

if __name__ == '__main__':
    rospy.init_node('CartoTF2Pose')
    poseCartoPub = rospy.Publisher('/poseCarto', Odometry, queue_size=10)  # customerized odometry from ENU
    cartoPathpub = rospy.Publisher('/cartoPath', Path, queue_size=100)
    cartoPath_ = Path()
    CartoPose_ = PoseStamped()
    CartoPose_.header.frame_id = 'velodyne'
    listener = tf.TransformListener()
    rate = rospy.Rate(int(1))
    print 'hshshsh'
    poseCarto = Odometry()
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print 'carto trans',trans
        print 'carto rotation',rot
        print '------'
        poseCarto.header.frame_id = 'map'
        poseCarto.pose.pose.position.x = trans[0]
        poseCarto.pose.pose.position.y = trans[1]
        poseCarto.pose.pose.position.z = trans[2]
        poseCarto.pose.pose.orientation.x = rot[0]
        poseCarto.pose.pose.orientation.y = rot[1]
        poseCarto.pose.pose.orientation.z = rot[2]
        poseCarto.pose.pose.orientation.w = rot[3]
        CartoPose_.pose = poseCarto.pose.pose
        cartoPath_.poses.append(CartoPose_)
        cartoPath_.header.frame_id = 'velodyne'
        print len(cartoPath_.poses)
        cartoPathpub.publish(cartoPath_)
        poseCartoPub.publish(poseCarto)
        rate.sleep()
