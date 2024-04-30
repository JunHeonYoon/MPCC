from math import pi
import numpy as np

# 8-shaped path with 0.2m radius (along YZ-axis)
r = 0.3
t = np.linspace(0, 2*pi, 100) + pi/2
x = np.zeros(t.shape)
y = r * np.sin(2 * t)
z = r * np.sin(t)

# x = r * np.sin(t)
# y = r * np.sin(2 * t)
# z = r * np.cos(t)

total_desc = f""" 
{{
    "X": {list(map(np.double, x))},
    "Y": {list(map(np.double, y))},
    "Z": {list(map(np.double, z))}
}} 
"""

open('track.json','w').write(total_desc)
