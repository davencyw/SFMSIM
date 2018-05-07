import numpy as np
import sys

camerafile = "cameraposes.csv"
staticfile = "landmark_static_3d.csv"
dynamicfile = "landmark_dynamic_3d.csv"

imageplaneheight = 480
imageplanewidth  = 620

folder = sys.argv[1]
numstatic = int(sys.argv[2])
numdynamic = int(sys.argv[3])
numframes = int(sys.argv[4])

filestatic  = folder + "/landmark_static_3d.csv"
filedynamic = folder + "/landmark_dynamic_3d.csv"

print "\nfolder                   : " + folder
print "number of static points  : " + str(numstatic)
print "number of dynamic points : " + str(numdynamic)
print "number of frames         : " + str(numframes) + "\n\n"

dynamic  = np.empty((numdynamic,0))
diff     = np.empty((numdynamic,0))
static   = np.empty((numstatic,0))

minmax = [[-imageplanewidth/2, imageplanewidth/2],[-imageplaneheight/2, imageplaneheight/2],[1,2]]

for dim in range(0,3) :
    staticloc = np.random.uniform(low=minmax[dim][0], high=minmax[dim][1], size=(numstatic,1))
    static = np.append(static, staticloc, axis=1)
    dynamicloc = np.random.uniform(low=minmax[dim][0], high=minmax[dim][1], size=(numdynamic,1))
    dynamic = np.append(dynamic, dynamicloc, axis=1)

dynamicframe = dynamic
for frame in range(0,numframes-1):
    diffxy = np.random.uniform(low=-2, high=2, size=(numdynamic,2))
    diffz = np.random.uniform(low=-0.1, high=0.2, size=(numdynamic,1))
    diff = np.append(diffxy, diffz, axis=1)
    dynamicframe = dynamicframe + diff
    dynamic = np.append(dynamic,dynamicframe,axis=0)


np.savetxt(filestatic, static, delimiter=" ",fmt='%f')
np.savetxt(filedynamic, dynamic, delimiter=" ",fmt='%f')

firstlinestatic = str(numstatic)
with open(filestatic, 'r') as original: data = original.read()
with open(filestatic, 'w') as modified: modified.write(firstlinestatic + "\n" + data)

firstlinedynamic = str(numframes) + " " + str(numdynamic)
with open(filedynamic, 'r') as original: data = original.read()
with open(filedynamic, 'w') as modified: modified.write(firstlinedynamic + "\n" + data)

print "files generated: "
print "\t\t\t"+filestatic
print "\t\t\t"+filedynamic
print "finished...\n\n"
