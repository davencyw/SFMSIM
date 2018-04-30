import matplotlib.pyplot as plt
import pandas as pd
import seaborn
import numpy as np
import sys

file = sys.argv[1]

filepath = "/media/davencyw/diskdata/mthesis/code/sfmsim//results/"

df = pd.read_csv(filepath + file + ".csv", delimiter=",", header=None)
df = df.transpose();
#df = df.rename({0: "f1", 1: "f2", 2: "f3"}, axis="columns");

seaborn.set(context="talk")
seaborn.set_style("darkgrid")

labeltext = "frame "
counter = 0

for column in df:
    counter += 1
    currentlabeltext = labeltext + str(counter)
    plt.plot(df[column], label=currentlabeltext)

plt.title(file)
plt.ylim(0,1.1)
plt.xlabel("point_i")
plt.ylabel("weight")
plt.legend(loc="lower right")

plt.show()
