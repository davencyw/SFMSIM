# About this Project
This structure from motion simulator is created as a workspace to test classification and weighting methods on SLAM systems in dynamic environment. It uses an internal (non-visual) representation of dynamic and static features which are assumed to be tracked without any mismatch but are affected by noise on the image detection. Unlike common bundle adjustment, this simulator uses a modified costfunction inside ceres to classifiy and weight features based on their reprojection error. This simulator was used in a first stage in my master thesis about "Visual SLAM for Dynamic Environemnts" at the V4RL at ETH Zurich.

# Building
This project is built and tested with g++ (Debian 7.3.0-16) 7.3.0 (should work on other unix systems aswell).

## Dependencies
- c++17
- Ceres
- OpenCV
- OpenCV contrib modules (OpenCV_SFM)
- Eigen
- Boost (program_options)

## Build with CMake
```bash
mkdir build && cd build
cmake ..
make -j4
```

# Run the Simulation
## Generate Dynamic/Static Features and Cameraposes
Run the genpoints.py in the ```scripts``` folder to generate feature points with the following arguments:
```python
python genpoints.py <outputfolder> <numstaticfeatures> <numdynamicfeatures> <numframes> <movement> <startstatic> <stopstatic>
```

For example to generate a dataset of 30 static points and 20 dynamic (which move in random directions) for 120 frames:
```python
python genpoints.py ../data 30 20 120 false 1 1
```
## Run sfmsimulator
Just call the executable with the following arguments.
```bash
cd build
./bin/sfmsimulation -o <outputfolder> -c <classifier1> -c <classifier2> -t <dataset1> -t <dataset2>
```

## Plot data
These python scripts use pandas, seaborn, scipy, numpy and matplotlib to plot different data to analyse and evaluate the method mentioned above.

First, one has to change the filepaths to the result folder. In this result folder the output of the simulator should lie in a seperate folder for each run. Eg: ```<path>/result/run0```.

### Plot Average Weight of Features
```python
python plot_average.py <run> <dataset> <numberofdynamicpoints>
```
eg:
```python
python plot_average.py run0 dataset0 20
```


### Plot Camera Trajectory Accuracy
```python
python plot_camera_accuracy.py <dataset>
```
