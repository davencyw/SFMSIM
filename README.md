# About this project
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

# Run the simulator
## Generate Dynamic and Static Features
Run the genpoints.py in the ```scripts``` folder to generate feature points with the following arguments:
```python
python genpoints.py <outputfolder> <numstaticfeatures> <numdynamicfeatures> <numframes> <movement> <startstatic> <stopstatic>
```

For example to generate a dataset of 30 static points and 20 dynamic (which move in random directions) for 120 frames:
```python
python genpoints.py ../data 30 20 120 false 1 1
```
## Run sfmsimulator
Just call the executable, simple as that.
```bash
cd build
./bin/sfmsimulation
```

## Plot data
These scripts use pandas, seaborn, scipy, numpy and matplotlib to plot different data to analyse and evaluate the method mentioned above.
