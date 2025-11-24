#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import math
import os
from datetime import datetime, timedelta, timezone

# Import messages from your existing gnss_comm package
from gnss_comm.msg import GnssPVTSolnMsg, GnssTimeMsg

# --- Configuration ---
# The number of leap seconds between GPS time and UTC.
LEAP_SECONDS = 18

def convert_utc_to_gps_time(utc_date, utc_time_str):
    """
    Converts a UTC date and time string to GPS Week and Time of Week (TOW).
    """
    try:
        hour = int(utc_time_str[0:2])
        minute = int(utc_time_str[2:4])
        second = int(float(utc_time_str[4:]))
        microsecond = int((float(utc_time_str[4:]) % 1) * 1e6)
        
        utc_dt = datetime(utc_date.year, utc_date.month, utc_date.day, 
                          hour, minute, second, microsecond, tzinfo=timezone.utc)

        gps_dt = utc_dt + timedelta(seconds=LEAP_SECONDS)
        gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
        time_diff = gps_dt - gps_epoch
        
        gps_week = int(time_diff.total_seconds() / (7 * 86400))
        gps_tow = time_diff.total_seconds() % (7 * 86400)

        return gps_week, gps_tow
    except (ValueError, TypeError):
        return None, None

def parse_and_publish():
    """
    Parses an NMEA file, processing both $GNRMC (for date) and $PUBX,00 (for PVT),
    and publishes gnss_comm/GnssPVTSolnMsg messages.
    """
    rospy.init_node('nmea_pvt_publisher', anonymous=True)
    pub = rospy.Publisher('/nmea/pvt_solution', GnssPVTSolnMsg, queue_size=10)
    
    rospack = rospkg.RosPack()
    default_path = os.path.join(rospack.get_path('nmea_parser'), 'data', 'UrbanNav-HK-Deep-Urban-1.ublox.f9p.nmea')
    
    nmea_file_path = rospy.get_param('~nmea_file', default_path)
    publish_rate_hz = rospy.get_param('~publish_rate', 10.0)
    rate = rospy.Rate(publish_rate_hz)

    KNOTS_TO_MS = 0.514444
    current_date = None

    rospy.loginfo("Starting NMEA parser for file: %s", nmea_file_path)
    rospy.loginfo("Using %d leap seconds for time conversion.", LEAP_SECONDS)

    try:
        # THE FIX IS HERE: Added errors='ignore' to the open() call
        with open(nmea_file_path, 'r', errors='ignore') as f:
            for line in f:
                if rospy.is_shutdown():
                    break

                parts = line.strip().split(',')
                
                if line.startswith('$GNRMC') and len(parts) > 9 and parts[9]:
                    try:
                        date_str = parts[9]
                        current_date = datetime.strptime(date_str, '%d%m%y').date()
                    except ValueError:
                        rospy.logwarn_throttle(10, "Could not parse date from RMC sentence: %s", line.strip())
                    continue

                if not line.startswith('$PUBX,00'):
                    continue
                
                if not current_date:
                    rospy.logwarn_throttle(5, "Skipping PUBX sentence, no date received yet from a GNRMC message.")
                    continue

                if len(parts) < 18:
                    rospy.logwarn("Skipping malformed $PUBX,00 line: %s", line.strip())
                    continue

                try:
                    pvt_msg = GnssPVTSolnMsg()

                    utc_time_str = parts[2]
                    week, tow = convert_utc_to_gps_time(current_date, utc_time_str)
                    if week is None:
                        rospy.logwarn("Failed to convert time for line: %s", line.strip())
                        continue
                    
                    pvt_msg.time = GnssTimeMsg()
                    pvt_msg.time.week = week
                    pvt_msg.time.tow = tow
                    
                    lat_raw = float(parts[3])
                    pvt_msg.latitude = int(lat_raw / 100) + (lat_raw % 100) / 60
                    if parts[4] == 'S': pvt_msg.latitude *= -1

                    lon_raw = float(parts[5])
                    pvt_msg.longitude = int(lon_raw / 100) + (lon_raw % 100) / 60
                    if parts[6] == 'W': pvt_msg.longitude *= -1
                    
                    pvt_msg.altitude = float(parts[7])
                    pvt_msg.height_msl = 0.0

                    nav_status = parts[8]
                    if 'G3' in nav_status: pvt_msg.fix_type = 3
                    elif 'G2' in nav_status: pvt_msg.fix_type = 2
                    elif 'DR' in nav_status: pvt_msg.fix_type = 1
                    else: pvt_msg.fix_type = 0
                    pvt_msg.valid_fix = pvt_msg.fix_type >= 2
                    pvt_msg.diff_soln = 'D' in nav_status
                    if 'R' in nav_status: pvt_msg.carr_soln = 2
                    elif 'F' in nav_status: pvt_msg.carr_soln = 1
                    else: pvt_msg.carr_soln = 0

                    pvt_msg.h_acc = float(parts[9])
                    pvt_msg.v_acc = float(parts[10])
                    pvt_msg.p_dop = float(parts[15])

                    sog_knots = float(parts[11])
                    cog_deg = float(parts[12])
                    v_vel_up_ms = float(parts[13])
                    sog_ms = sog_knots * KNOTS_TO_MS
                    cog_rad = math.radians(cog_deg)
                    pvt_msg.vel_n = sog_ms * math.cos(cog_rad)
                    pvt_msg.vel_e = sog_ms * math.sin(cog_rad)
                    pvt_msg.vel_d = -v_vel_up_ms
                    pvt_msg.vel_acc = 0.0

                    pvt_msg.num_sv = int(parts[18].split('*')[0])

                    pub.publish(pvt_msg)
                    rate.sleep()

                except (ValueError, IndexError) as e:
                    rospy.logwarn("Could not parse line content: %s. Error: %s", line.strip(), e)
        
        rospy.loginfo("Finished reading NMEA file. Node will now exit.")

    except IOError as e:
        rospy.logerr("Could not open NMEA file at '%s'. Error: %s", nmea_file_path, e)
    except Exception as e:
        rospy.logerr("An unexpected error occurred: %s", e)

if __name__ == '__main__':
    try:
        parse_and_publish()
    except rospy.ROSInterruptException:
        pass