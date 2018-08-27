import matplotlib.pyplot as plt
import pandas as pd
import seaborn
import numpy as np
import sys
import os
import fnmatch


setting = sys.argv[1]
set = sys.argv[2]
numdyn = int(sys.argv[3])

filepath = "../results/"+setting+"/"
print filepath
for file in os.listdir(filepath):
    print file
    if fnmatch.fnmatch(file, '*weights.csv') and fnmatch.fnmatch(file, set+'*'):
        print file

        split = file.split("_")
        dep = split[1]

        df = pd.read_csv(filepath +"/"+ file, delimiter=",", header=None)
        df = df.transpose()

        df.index += 1

        seaborn.set(context="talk")
        seaborn.set_style("darkgrid")

        labeltext = "frame "
        counter = 0

        dynamicweightaverage = [1]
        staticweightaverage = [1]
        array = df.as_matrix()
        length = (array.shape[0]-1)
        numframes = array.shape[1]
        x = range(1,numframes+2)

        for col in array.T:
            locaveragedyn = np.sum((col[0:numdyn])) / numdyn
            locaveragestat = np.sum((col[numdyn:-1])) / (length-numdyn)
            dynamicweightaverage.append(locaveragedyn)
            staticweightaverage.append(locaveragestat)

        label = "average weights of dynamic points"
        plt.plot(x,dynamicweightaverage,label=label)
        label = "average weights of static points"
        plt.plot(x,staticweightaverage,label=label)

        title = set+ "_"+ dep +"_average_weights"
        plt.title(title)
        plt.ylim(-0.1,1.1)
        plt.xlim(2,numframes+1)
        plt.xticks(range(2,numframes+1,int(numframes/10)))
        plt.xlabel("frame")
        plt.ylabel("weight")
        plt.legend(loc="lower right")
        plt.savefig(title,dpi=300)
        plt.show()
