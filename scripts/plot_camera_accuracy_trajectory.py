import matplotlib.pyplot as plt
import sys
import numpy
import math
import pandas as pd
import seaborn
import numpy as np
import os
import fnmatch
from scipy.spatial import distance

filepath = "/media/davencyw/diskdata/mthesis/code/sfmsim/results/"

setpath = sys.argv[1]
set = sys.argv[2]
filepath += setpath

print "Set: " + set
seaborn.set(context="talk")
seaborn.set_style("darkgrid")

files = []

for file in os.listdir(filepath):
    if fnmatch.fnmatch(file, '*camera.csv') and fnmatch.fnmatch(file, set +'*' ):
        files.append(file)
files = sorted(files)

print "files: "
for file in files:
    groundtruthfile = file.rsplit('.', 1)[0]
    groundtruthfile += "_gt.csv"
    print file
    print groundtruthfile


    split = file.split("_")
    dep = split[1]

    df = pd.read_csv(os.path.join(filepath,file), delimiter=" ", header=None)
    df_gt = pd.read_csv(os.path.join(filepath,groundtruthfile), delimiter=" ", header=None)


    camera = df.as_matrix()
    camera_gt = df_gt.as_matrix();
    numframes = camera_gt.shape[0] - 1
    slidingwindowsize = camera[0][0]
    camera = camera[1:][:]
    cameraerror = []

    camera_gt_x = camera_gt[:,3]
    camera_gt_y = camera_gt[:,4]
    camera_gt_z = camera_gt[:,5]


    #translate to origin (also used for scaling)
    camera[:,3] -= camera_gt[0,3]
    camera[:,4] -= camera_gt[0,4]
    camera_gt[:,3] -= camera_gt[0,3]
    camera_gt[:,4] -= camera_gt[0,4]


    currentslidingwindowsize = 2
    camerastart = 0
    camera_gtstart = 0

    for frame_i in range(0,numframes):

        frameerror = 0

        camerarange =  range(camerastart, camerastart + currentslidingwindowsize)
        camera_gtrange = range(camera_gtstart, camera_gtstart + currentslidingwindowsize)

        localcam = camera[camerarange,0:6]
        localgtcam = camera_gt[camera_gtrange,0:6]

        ###scale
        averagescale=0
        #loop over every pair and average scaling
        numpairs = currentslidingwindowsize -1
        for pair_i in range(0,numpairs):
            pt1gt = (localgtcam[pair_i,3],localgtcam[pair_i,4])
            pt2gt = (localgtcam[pair_i+1,3],localgtcam[pair_i+1,4])
            pt1 = (localcam[pair_i,3],localcam[pair_i,4])
            pt2 = (localcam[pair_i+1,3],localcam[pair_i+1,4])

            gtdist = distance.euclidean(pt1gt,pt2gt)
            dist = distance.euclidean(pt1,pt2)
            localscale = dist / gtdist
            averagescale += localscale

        averagescale /= float(numpairs)
        localcam /= averagescale
        ###end scale

        for position_i in range(0, currentslidingwindowsize):
            positionerror = distance.euclidean(localcam[position_i],localgtcam[position_i])
            positionerror = (positionerror * positionerror)/3.0
            frameerror += positionerror
            positionerror /= 3
            frameerror += positionerror

        frameerror /= currentslidingwindowsize
        frameerror = math.sqrt(frameerror)

        cameraerror.append(frameerror)

        camerastart += currentslidingwindowsize
        if currentslidingwindowsize < slidingwindowsize:
            currentslidingwindowsize += 1
        else:
            camera_gtstart += 1



    x = range(2,numframes+2)

    label = dep + " RMSE"
    plt.plot(x,cameraerror,label=label)

title = set+"_cameratrajectory_error"
plt.title(title)
plt.xlim(2,numframes+1)
# plt.ylim(0,5)
plt.xticks(range(2,numframes+2,int((numframes+1)/10)))
plt.xlabel("frame")
plt.ylabel("error")
plt.legend(loc="upper right")
plt.savefig(title,dpi=300)
plt.show()

print "saved in: "+title
