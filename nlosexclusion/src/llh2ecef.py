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
a=6378137.0
b=6356752.314
# llh= [22.30419607114303, 114.1812441949352, 121.42342057159652]
# line_index= 156 itera_xyz= [-2418380.163540058, 5385854.5559571795, 2405656.8610494756]
def llh2xyz(llh):
    xyz=[]
    lat = float(llh[0]) * 3.1415926 / 180.0
    lon = float(llh[1]) * 3.1415926 / 180.0
    hr = float(llh[2])
    n = a * a / sqrt(a * a * cos(lat) * cos(lat) + b * b * sin(lat) * sin(lat))
    Rx = (n + hr) * cos(lat) * cos(lon)
    Ry = (n + hr) * cos(lat) * sin(lon)
    Rz = (b * b / (a * a) * n + hr) * sin(lat)
    xyz.append(float(Rx))
    xyz.append(float(Ry))
    xyz.append(float(Rz))
    return xyz