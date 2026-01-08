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

from std_msgs.msg import String
from numpy import *
import math
import llh2ecef #ecef coordinate to llh coordinate
import ecef2llh #ecef coordinate to llh coordinate
a=6378137.0
b=6356752.314
# llh= [22.30419607114303, 114.1812441949352, 121.42342057159652]
# line_index= 156 itera_xyz= [-2418380.163540058, 5385854.5559571795, 2405656.8610494756]

def enu2ecef(originllh, enu):
    e = float(enu[0])
    n = float(enu[1])
    u = float(enu[2])
    lon = float(originllh[1]) * 3.1415926 / 180.0;
    lat = float(originllh[0]) * 3.1415926 / 180.0;

    oxyz = []
    oxyz = llh2ecef.llh2xyz(originllh)
    ox = float(oxyz[0])
    oy = float(oxyz[1])
    oz = float(oxyz[2])

    # a1 = ox - sin(lon) * e - cos(lon) * sin(lat) * n + cos(lon) * cos(lat) * u
    # a2 = oy + cos(lon) * e - sin(lon) * sin(lat) * n + cos(lat) * sin(lon) * u
    # a3 = oz + cos(lat) * n + sin(lat) * u

    oxyz[0] = ox - sin(lon) * e - cos(lon) * sin(lat) * n + cos(lon) * cos(lat) * u
    oxyz[1] = oy + cos(lon) * e - sin(lon) * sin(lat) * n + cos(lat) * sin(lon) * u
    oxyz[2] = oz + cos(lat) * n + sin(lat) * u
    return oxyz;