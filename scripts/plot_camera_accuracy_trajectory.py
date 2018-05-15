import matplotlib.pyplot as plt
import sys
import numpy
import math
import pandas as pd
import seaborn
import numpy as np
import os
import fnmatch

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

    currentslidingwindowsize = 2
    camerastart = 0
    camera_gtstart = 0

    for frame_i in range(0,numframes):

        frameerror = 0

        camerarange =  range(camerastart, camerastart + currentslidingwindowsize)
        camera_gtrange = range(camera_gtstart, camera_gtstart + currentslidingwindowsize)


        for position_i in range(0, currentslidingwindowsize):
            positionerror = 0
            for i in range(0,6):
                 coefferror = camera[camerarange[position_i]][i] - camera_gt[camera_gtrange[position_i]][i]
                 positionerror += coefferror * coefferror
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
plt.xticks(x)
plt.xlabel("frame")
plt.ylabel("error")
plt.legend(loc="upper right")
plt.savefig(title,dpi=300)
plt.show()

print "saved in: "+title
