import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import numpy
import math
import pandas as pd
import seaborn
import numpy as np
import os
import fnmatch

classifier = "dep3"

filepath = "/media/davencyw/diskdata/mthesis/code/sfmsim/results/s8/"
file = "Tlarge4_"+classifier+"_camera.csv"
file_gt = "Tlarge4_"+classifier+"_camera_gt.csv"


df = pd.read_csv(os.path.join(filepath,file), delimiter=" ", header=None)
df_gt = pd.read_csv(os.path.join(filepath,file_gt), delimiter=" ", header=None)

camera = df.as_matrix()
camera_gt = df_gt.as_matrix();
numframes = camera_gt.shape[0] - 1
slidingwindowsize =  int(camera[0][0])
camera = camera[1:][:]
cameraerror = []

currentslidingwindowsize = 2
camerastart = 0

camera_gt_x = camera_gt[:,3]
camera_gt_y = camera_gt[:,4]
camera_gt_z = camera_gt[:,5]
xmin = 0
xmax = 0
ymin = 0
ymax = 0

currentslidingwindowsize = 2
camerastart = 0
fig, ax = plt.subplots()
ax.plot(camera_gt_x, camera_gt_y, '-^',label="ground truth trajectory")


for frame_i in range(0,numframes):
    camerarange =  range(camerastart, camerastart + currentslidingwindowsize)
    localcam = camera[camerarange]
    localcam =  localcam[:,3:6]

    ax.plot(localcam[:,0],localcam[:,1], '-o',label="trajectory at frame " + str(frame_i))
    camerastart += currentslidingwindowsize
    if currentslidingwindowsize < slidingwindowsize:
        currentslidingwindowsize += 1
    else:
        camera_gtstart += 1

    xtot = np.concatenate([localcam[:,0],camera_gt_x, [xmin,xmax]]);
    ytot = np.concatenate([localcam[:,1],camera_gt_y, [ymin,ymax]]);
    xmin = np.min(xtot)
    xmax = np.max(xtot)
    ymin = np.min(ytot)
    ymax = np.max(ytot)
plt.xlim(xmin-1,xmax+1)
plt.ylim(ymin-1,ymax+1)
# plt.legend(loc="upper left")
box = ax.get_position()
ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])
ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
