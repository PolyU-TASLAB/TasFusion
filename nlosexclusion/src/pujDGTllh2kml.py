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

#! /usr/bin/env python
# -*- coding=utf-8 -*-
# finished by Dr. WEN
"""
    Function: read lat lon height from jingdong standard dataset
    Welson Wen, Ph.D.
    https://sites.google.com/view/weisongwen/about-me

    subcribe : '/ublox_gps_node/fix' latitude longitude

"""
import rospy
import rospy
from pykml.factory import KML_ElementMaker as KML 
import csv # csv reading needed library
from lxml import etree 

from sensor_msgs.msg   import NavSatFix # standard message type for GNSSs


class pullh2kml_eval():
    def __init__(self):
        rospy.Subscriber('/fix2', NavSatFix, self.callublox_llh)
        self.lat_ = [] # used to save latitude
        self.lon_ = [] # used to save longitude
        self.GPS_Week_Second = 0.0
        self.writeToKML = 0.0
        self.standardGT2kml()

    def callublox_llh(self,data):
        self.navsatfix_ = NavSatFix()
        self.navsatfix_ = data
        self.lat_.append(float(self.navsatfix_.latitude))
        print 'len(self.lat_)',len(self.lat_)
        self.lon_.append(float(self.navsatfix_.longitude))
        self.GPS_Week_Second = self.navsatfix_.header.stamp
        print 'GPS_Week_Second',self.GPS_Week_Second

    def standardGT2kml(self):
        print 'begin read standard data'
        self.Fcsv_GNSS = csv.reader(open('/home/wws/software_tim2/librsfllh_trajectory.csv','r'))  # read csv context to csv_reader variable
        for rowCsv in self.Fcsv_GNSS:
            # print 'rowCsv.size () -> ',len(rowCsv)
            self.lat_.append(float(rowCsv[1]))
            self.lon_.append(float(rowCsv[0]))
            
        print 'finish tranversal standard data',len(self.lat_)




if __name__ == '__main__':
    rospy.init_node('pullh2kml_evaluublox', anonymous=True)
    pullh2kml_eval_ =pullh2kml_eval()
    rate = rospy.Rate(0.002)#
    preTim = 0.0
    while not rospy.is_shutdown():
        #rate.sleep()
        # print 'sleep end ',preTim,pullh2kml_eval_.GPS_Week_Second
        #if( (preTim > 0) and (preTim == pullh2kml_eval_.GPS_Week_Second) and (pullh2kml_eval_.writeToKML ==0)):
	if( len(pullh2kml_eval_.lon_)>5):
            # print 'finished...and begin to write llh to kml'
            pullh2kml_eval_.writeToKML = 1
            # Use the first point to create a Folder
            fold = KML.Folder(KML.Placemark(
                KML.Point(KML.coordinates(str(pullh2kml_eval_.lon_[0]) + ',' + str(pullh2kml_eval_.lat_[0]) + ',0'))
            )
            )
            # Append the remaining points to the Folder
            for i in range(1, len(pullh2kml_eval_.lon_)):
                fold.append(KML.Placemark(
                    KML.Point(KML.coordinates(str(pullh2kml_eval_.lon_[i]) + ',' + str(pullh2kml_eval_.lat_[i]) + ',0')))
                )
            # Use etree to output the KML node as string data
            content = etree.tostring(etree.ElementTree(fold), pretty_print=True)
            # Save to file, then it can be opened in Google Earth
            with open('/home/wws/software_tim2/librsfllh_trajectory.kml', 'w') as fp:
                fp.write(content)

        preTim = pullh2kml_eval_.GPS_Week_Second
