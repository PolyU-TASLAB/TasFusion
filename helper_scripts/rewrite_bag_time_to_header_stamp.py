# /**
# * This file is part of tasfusion.
# *
# * Copyright (C) 2025 Trustworthy AI and Autonomous Systems Laboratory, The Hong Kong Polytechnic University (PolyU)
# * Author: ZHAO Jiaqi (jiaqi.zhao@connect.polyu.hk) Wang Xiangru (xiangru.wang@polyu.edu.hk) Gao Yixin (yixin.gao@connect.polyu.hk)
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

import os
import argparse
import rosbag
import rospy
from datetime import datetime, timedelta, timezone

# GPS time is 18 seconds ahead of UTC (as of 2017)
# Note: This value may need to be updated if processing data from after the next leap second insertion.
LEAP_SECONDS = 18

def gps_to_ros_time(gps_week, gps_seconds):
    """
    Converts GPS week and seconds of week to a rospy.Time object.
    """
    # GPS epoch started at 1980-01-06 00:00:00 UTC
    gps_epoch = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
    
    # Calculate total time since the GPS epoch
    time_since_epoch = timedelta(weeks=gps_week, seconds=gps_seconds)
    
    # Current time in the GPS time scale
    current_gps_time = gps_epoch + time_since_epoch
    
    # Subtract leap seconds to get UTC
    current_utc_time = current_gps_time - timedelta(seconds=LEAP_SECONDS)
    
    # Convert UTC to a Unix timestamp and create a rospy.Time object
    return rospy.Time.from_sec(current_utc_time.timestamp())

def main():
    parser = argparse.ArgumentParser(description="Rewrites rosbag timestamps based on GPS time within messages.")
    parser.add_argument('input_bag', help='Path to the input rosbag file.')
    parser.add_argument('output_bag', help='Path for the output rosbag file.')
    args = parser.parse_args()

    input_bag_path = args.input_bag
    output_bag_path = args.output_bag

    output_dir = os.path.dirname(os.path.abspath(output_bag_path))
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"[INFO] Created output directory: {output_dir}")

    print(f"[INFO] Processing input file: {input_bag_path}")
    msg_count = 0
    
    # Message type names to check for (checks the suffix of the full type string)
    gnss_eph_type_suffix = 'GnssEphemMsg'
    glonass_eph_type_suffix = 'GnssGloEphemMsg'
    gnss_meas_type_suffix = 'GnssMeasMsg'
    # NEW: Define suffix for PVT solution message
    gnss_pvt_type_suffix = 'GnssPVTSolnMsg'

    with rosbag.Bag(output_bag_path, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
            new_t = t # Default to original timestamp

            # Handle Ephemeris messages (GPS, GLONASS, etc.)
            if msg._type.endswith(gnss_eph_type_suffix) or msg._type.endswith(glonass_eph_type_suffix):
                # Access path: msg.toe.week and msg.toe.tow
                if hasattr(msg, 'toe') and hasattr(msg.toe, 'week') and hasattr(msg.toe, 'tow'):
                    new_t = gps_to_ros_time(msg.toe.week, msg.toe.tow)

            # Handle GNSS Measurement messages
            elif msg._type.endswith(gnss_meas_type_suffix):
                # The timestamp is in the first observation of the 'means' array.
                # Access path: msg.means[0].time.week and msg.means[0].time.tow
                if (hasattr(msg, 'meas') and len(msg.meas) > 0 and
                        hasattr(msg.meas[0], 'time') and 
                        hasattr(msg.meas[0].time, 'week') and 
                        hasattr(msg.meas[0].time, 'tow')):
                    new_t = gps_to_ros_time(msg.meas[0].time.week, msg.meas[0].time.tow)
            
            # NEW: Handle GnssPVTSolnMsg
            # Based on the image, the time fields are directly in the message
            elif msg._type.endswith(gnss_pvt_type_suffix):
                if (hasattr(msg, 'time') and
                    hasattr(msg.time, 'week') and
                    hasattr(msg.time, 'tow')):
                    new_t = gps_to_ros_time(msg.time.week, msg.time.tow)

            # Fallback for any other message with a standard ROS header
            elif hasattr(msg, 'header') and hasattr(msg.header, 'stamp') and msg.header.stamp.secs != 0:
                new_t = msg.header.stamp
            
            # Write message with the new (or original) timestamp
            outbag.write(topic, msg, new_t)
            msg_count += 1
            if msg_count % 1000 == 0:
                print(f"  Processed {msg_count} messages...")

    print(f"[DONE] Processing complete! ✅ Wrote {msg_count} messages to: {output_bag_path}")

if __name__ == '__main__':
    main()