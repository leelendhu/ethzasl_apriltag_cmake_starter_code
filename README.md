# [Ethzasl Apriltag](https://github.com/leelendhu/ethzasl_apriltag) Starter Code
This is the starter code for running Ethz's AprilGrid detection code.

Requirement Update: Install non-catkin version of ethzasl_apriltag2 from this repository https://github.com/leelendhu/ethzasl_apriltag to use the AprilGrid class
**Build**
```
mkdir build && cd build
cmake ..
make
```

**Running sample data**
```
./main
```
**Undistorting and rectifying images**
```
./undistort
```
**Creating disparity maps**
```
cd ..
python match.py
```
