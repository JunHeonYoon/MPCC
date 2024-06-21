from math import pi
import numpy as np
from scipy.spatial.transform import Rotation as R

t1 = np.linspace(0, 1.0, 100)
t2 = np.linspace(0, 1.0, 100)
t3 = np.linspace(0, 1.0, 100)

x = -0.52*t1
y = 0*t1
z = 0*t1

x = np.concatenate([x, x[-1] + 0*t2],axis=0)
y = np.concatenate([y, y[-1] + 0*t2],axis=0)
z = np.concatenate([z, z[-1] -0.18*t2],axis=0)

x = np.concatenate([x, x[-1] + 0.52*t3],axis=0)
y = np.concatenate([y, y[-1] + 0*t3],axis=0)
z = np.concatenate([z, z[-1] + 0*t3],axis=0)



# stay initial orientation
rot = R.from_matrix([[1,  0,  0],
                     [0, -1, 0],
                     [0,  0, -1]])


quat = rot.as_quat()
quat_list = np.tile(quat, (x.size, 1))

total_desc = f""" 
{{
    "X": {list(map(np.double, x))},
    "Y": {list(map(np.double, y))},
    "Z": {list(map(np.double, z))},
    "quat_X": {list(map(np.double, quat_list[:,0]))},
    "quat_Y": {list(map(np.double, quat_list[:,1]))},
    "quat_Z": {list(map(np.double, quat_list[:,2]))},
    "quat_W": {list(map(np.double, quat_list[:,3]))}
}} 
"""

open('track.json','w').write(total_desc)
