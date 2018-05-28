import numpy as np
import sys

camerafile = "cameraposes.csv"
staticfile = "landmark_static_3d.csv"
dynamicfile = "landmark_dynamic_3d.csv"

imageplaneheight = 480
imageplanewidth  = 620
focal = 1

folder = sys.argv[1]
numstatic = int(sys.argv[2])
numdynamic = int(sys.argv[3])
numframes = int(sys.argv[4])

filestatic  = folder + "/landmark_static_3d.csv"
filedynamic = folder + "/landmark_dynamic_3d.csv"
filecamera = folder + "/camera_poses.csv"

print "\nfolder                   : " + folder
print "number of static points  : " + str(numstatic)
print "number of dynamic points : " + str(numdynamic)
print "number of frames         : " + str(numframes) + "\n\n"

dynamic_image  = np.empty((numdynamic,0))
diff     = np.empty((numdynamic,0))
static_image   = np.empty((numstatic,0))

max = [imageplanewidth*0.5*0.85, imageplaneheight*0.5*0.85]
for dim in range(0,2) :
    staticloc_image = np.random.uniform(low=-max[dim], high=max[dim], size=(numstatic,1))
    static_image = np.append(static_image, staticloc_image, axis=1)
    dynamicloc_image = np.random.uniform(low=-max[dim], high=max[dim], size=(numdynamic,1))
    dynamic_image = np.append(dynamic_image, dynamicloc_image, axis=1)

#project back onto 3d space by defining z-coordinate
maxz = 6
staticz = np.random.uniform(low=1,high=maxz, size=(numstatic))
dynamicz = np.random.uniform(low=1,high=maxz, size=(numdynamic))

staticx = static_image[:,0] / focal
staticx = staticx * staticz
staticy = static_image[:,1] / focal
staticy = staticy * staticz

dynamicx = dynamic_image[:,0] / focal
dynamicx = dynamicx * dynamicz
dynamicy = dynamic_image[:,1] / focal
dynamicy = dynamicy * dynamicz

static = np.column_stack((staticx,staticy,staticz))
dynamic = np.column_stack((dynamicx,dynamicy,dynamicz))

dynpointsstaticstart = int(numframes * 0.25)-1
dynpointsstaticstop  = int(numframes * 0.75)-1

dynamicframe = dynamic
for frame in range(0,numframes-1):
    diffxy = np.random.uniform(low=-2, high=2, size=(numdynamic,2))
    diffz = np.random.uniform(low=-0.1, high=0.2, size=(numdynamic,1))
    diff = np.append(diffxy, diffz, axis=1)
    if frame > dynpointsstaticstart and frame < dynpointsstaticstop :
        diff.fill(0.0)
    dynamicframe = dynamicframe + diff
    dynamic = np.append(dynamic,dynamicframe,axis=0)


camerarotation = np.zeros(numframes)
sinx = np.linspace(0,np.pi*6,numframes)
camerasiny = np.sin(sinx)
cameray = camerasiny * 5
camerax = np.linspace(-5,5,numframes)
cameraz = np.zeros(numframes)

camera = np.column_stack((camerarotation,camerarotation,camerarotation,camerax,cameray,cameraz))

np.savetxt(filestatic, static, delimiter=" ",fmt='%f')
np.savetxt(filedynamic, dynamic, delimiter=" ",fmt='%f')
np.savetxt(filecamera, camera, delimiter=" ", fmt='%f')

firstlinestatic = str(numstatic)
with open(filestatic, 'r') as original: data = original.read()
with open(filestatic, 'w') as modified: modified.write(firstlinestatic + "\n" + data)

firstlinedynamic = str(numframes) + " " + str(numdynamic)
with open(filedynamic, 'r') as original: data = original.read()
with open(filedynamic, 'w') as modified: modified.write(firstlinedynamic + "\n" + data)

firstlinecamera = str(numframes)
with open(filecamera, 'r') as original: data = original.read()
with open(filecamera, 'w') as modified: modified.write(firstlinecamera + "\n" + data)

print "files generated: "
print "\t\t\t"+filestatic
print "\t\t\t"+filedynamic
print "\t\t\t"+filecamera
print "finished...\n\n"
