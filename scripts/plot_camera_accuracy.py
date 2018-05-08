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

set = sys.argv[1]
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

    print file
    split = file.split("_")
    dep = split[1]

    df = pd.read_csv(os.path.join(filepath,file), delimiter=" ", header=None)

    camera = df.as_matrix()
    numframes = camera.shape[0]/2

    cameraerror = []

    for frame_i in range(0,numframes):
        error=0
        for i in range(0,6):
            locerror = camera[2*frame_i][i] - camera[2*frame_i+1][i]
            error +=  locerror * locerror
        error /= 3
        error = math.sqrt(error)
        cameraerror.append(error)

    #print cameraerror

    x = range(2,numframes+2)

    label = dep + " RMSE"
    plt.plot(x,cameraerror,label=label)


title = set+"_camera_error"
plt.title(title)
plt.xlim(0,numframes+1)
plt.xlabel("frame")
plt.ylabel("error")
plt.legend(loc="upper right")
plt.savefig(title,dpi=300)
plt.show()

print "saved in: "+title
