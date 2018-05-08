import matplotlib.pyplot as plt
import pandas as pd
import seaborn
import numpy as np
import sys

file = sys.argv[1]
numdyn = int(sys.argv[2])

filepath = "/media/davencyw/diskdata/mthesis/code/sfmsim/results/noise_0_3"

df = pd.read_csv(filepath + file + ".csv", delimiter=",", header=None)
df = df.transpose()

df.index += 1

seaborn.set(context="talk")
seaborn.set_style("darkgrid")

labeltext = "frame "
counter = 0

dynamicweightaverage = []
staticweightaverage = []
array = df.as_matrix()
length = (array.shape[0]-1)
numfrmes = array.shape[1]
print length

for col in array.T:
    locaveragedyn = np.sum(np.abs(col[0:numdyn])) / numdyn
    locaveragestat = np.sum(np.abs(col[numdyn:-1])) / (length-numdyn)
    dynamicweightaverage.append(locaveragedyn)
    staticweightaverage.append(locaveragestat)

label = "average weights of dynamic points"
plt.plot(dynamicweightaverage,label=label)
label = "average weights of static points"
plt.plot(staticweightaverage,label=label)


plt.title(file+"_average")
plt.ylim(-0.1,1.1)
plt.xlabel("frame")
plt.ylabel("weight")
plt.legend(loc="center right")
plt.savefig(file+"_average",dpi=300)
plt.show()
