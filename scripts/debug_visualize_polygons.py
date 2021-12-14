#!/usr/bin/env python

import rospkg
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pickle


def polygonstamped_to_nparray(polygon):
    return np.array([[pt.x, pt.y, pt.z] for pt in polygon.polygon.points])


r = rospkg.RosPack()
dump_file = r.get_path('yamaopt_ros') + "/accum_polygons.pickle"
with open(dump_file, 'rb') as f:
    polygons = pickle.load(f)

polygons = [polygonstamped_to_nparray(e) for e in polygons]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for polygon in polygons:
    tmp = np.vstack([polygon, polygon[0]])
    ax.plot(tmp[:, 0], tmp[:, 1], tmp[:, 2], marker='o')
plt.show()
