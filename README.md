# [Ethzasl Apriltag2](https://github.com/ardiya/ethzasl_apriltag2) Starter Code
This is the starter code for running Ethz's AprilGrid detection code.

Requirement: install non-catkin version of ethzasl_apriltag2 from this repository https://github.com/ardiya/ethzasl_apriltag2

Update: Install non-catkin version of ethzasl_apriltag2 from this repository https://github.com/leelendhu/ethzasl_apriltag2 to use the AprilGrid class
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
python match.py
```
