# Lidar Obstacle Detection

<img src="media/Peek 2019-09-08 15-39.gif" width="700" height="400" />
<img src="media/Peek 2019-09-08 15-55.gif" width="700" height="400" />

## Project description

The goal of project is apply segmentation of objects around the ego motion car and as a result detect obstacles around it using LiDAR data.
In this project, the following tasks were completed:
* Implementation of customized [3D Ransac](https://github.com/brunoeducsantos/Lidar_Obstacle_Detection/blob/master/src/processPointClouds.cpp)
* Implementation of [KDTree](https://github.com/brunoeducsantos/Lidar_Obstacle_Detection/blob/master/src/kdtree.h)
* Implementation of [pre-processing cloud pipeline](https://github.com/brunoeducsantos/Lidar_Obstacle_Detection/blob/master/src/processPointClouds.cpp)

## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
