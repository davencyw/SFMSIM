import subprocess
import os
import fnmatch
import sys, ast

setpath = sys.argv[1]
setlist = ast.literal_eval( sys.argv[2] )

classifiers = ["noclassifier","nodep","dep3"]
filepath = "/media/davencyw/diskdata/mthesis/code/sfmsim/results/"+setpath
framefilepath = filepath + "/trajectory_frames"
os.chdir(framefilepath)

for set in setlist :
    for classifier in classifiers :
        pngfilenames = set + "_" + classifier + "_*.png"
        outputname = set + "_" + classifier + "_.mp4"
        print pngfilenames
        print outputname
        subprocess.call(['ffmpeg', '-r', '4', '-pattern_type', 'glob', '-i', pngfilenames, '-c:v', 'libx264', '-vf', 'fps=25', '-pix_fmt', 'yuv420p', outputname])
