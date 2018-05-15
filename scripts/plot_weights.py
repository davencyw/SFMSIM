import matplotlib.pyplot as plt
import pandas as pd
import seaborn
import numpy as np
import sys
import os
import fnmatch

setting = sys.argv[1]

filepath = "/media/davencyw/diskdata/mthesis/code/sfmsim/results/"+setting+"/"

for file in os.listdir(filepath):
    if fnmatch.fnmatch(file, '*weights.csv'):
        names = file.split("_")

        set = names[0]
        classifier = names[1]
        df = pd.read_csv(filepath + file , delimiter=",", header=None)
        df = df.transpose()
        #df = df.rename({0: "f1", 1: "f2", 2: "f3"}, axis="columns");

        df.index += 1

        seaborn.set(context="talk")
        seaborn.set_style("darkgrid")

        labeltext = "frame "
        counter = 0

        for column in df:
            counter += 1
            currentlabeltext = labeltext + str(counter)
            plt.plot(df[column], label=currentlabeltext, marker='o', linestyle='None')

        plt.title(set + "_"+"_weights")
        plt.ylim(0,1.1)
        plt.xlabel("point_i")
        plt.ylabel("weight")
        plt.legend(loc="lower right")
        plt.savefig(file,dpi=300)
        plt.show()
