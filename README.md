# TSDF

## Background

TSDF Is a set of C++ classes implementing a Truncated Signed Distance Function as described in [1].
It uses GPU acceleration to deliver some kind of performance but is by no means optimised.  You might think of it as an example of how to build one rather than a performance version.

It also includes an early version of a Kinect Fusion implementation to reconstruct static scenes from depth and RGB images.  It does not capture camera data in real time, it's too slow for that, but it will process captured data from saved RGB and depth images files. It's tested against the TUM datasets found here http://vision.in.tum.de/data/datasets/rgbd-dataset/download  [2].

Finally there are a couple of 'useful' tools that I've built along the way to inspect intermediate results.

## Samples
Here are a couple of sample outputs from the code

__Rendered scene__

![Rendered scene](/images/scene.png)

__Normals__

![Normals](/images/normals.png)

__Rendered Mesh__

![Mesh](/images/mesh.png)


# Using It

## Dependencies
As this code depends on CUDA for implementing GPU acceleration, it will require an NVidia card to run as well as CUDA libraries installed. It definitely compiles with CUDA 7.5 on UBuntu 16 LTS. It's not tested on other platforms but does request compute capability 5.0.

## Build

Assuming that CUDA is installed, run 
    make -f kinfu.make

To build kinfu. This should result in a kinfu executable appearing in the bin directory.

## Use

kinfu should be run with one of the following sets of options:

-f filename (to load a previouslt created TSDF from file)

-m <number of frames>  -d <directory> (to create from raw RGB and D images )

If the -d option is used then the directpry specified is expected to be a directory unpacked from the TUM test data sets and as described below in TUM File Formats in this case, -m is the number of frames of data to merge.

As experimental code, other options are not surfaced as command line options yet but may be edited in the source, specificallly you will need to :

1. Change the name of the output files in `src/Tools/kinfu.cpp`
-- scene.png is the scene rendered from the pose of the first camera frame
-- normals.png are the normal for the scene
-- mesh.ply is a ply file for the mesh extracted from the TSDF, suitable for viewing in meshlab

2. If you want to write out the intermediate TSDF to file you'll need to add code `volume->save_to_file( <filename> );`


# TUM File Formats
We provide the time-stamped color and depth images as a gzipped tar file (TGZ).

The color images are stored as 640×480 8-bit RGB images in PNG format.
The depth maps are stored as 640×480 16-bit monochrome images in PNG format.
The color and depth images are already pre-registered using the OpenNI driver from PrimeSense, i.e., the pixels in the color and depth images correspond already 1:1.
The depth images are scaled by a factor of 5000, i.e., a pixel value of 5000 in the depth image corresponds to a distance of 1 meter from the camera, 10000 to 2 meter distance, etc. A pixel value of 0 means missing value/no data.



We provide the ground truth trajectory as a text file containing the translation and orientation of the camera in a fixed coordinate frame. Note that also our automatic evaluation tool expects both the ground truth and estimated trajectory to be in this format.

Each line in the text file contains a single pose.
The format of each line is 'timestamp tx ty tz qx qy qz qw'
timestamp (float) gives the number of seconds since the Unix epoch.
tx ty tz (3 floats) give the position of the optical center of the color camera with respect to the world origin as defined by the motion capture system.
qx qy qz qw (4 floats) give the orientation of the optical center of the color camera in form of a unit quaternion with respect to the world origin as defined by the motion capture system.

The file may contain comments that have to start with ”#”.


# References
[ 1] Curless, B. & Levoy, M. 1996, 'A volumetric method for building complex models from range images'.

[ 2] Sturm, J., Engelhard, N., Endres, F., Burgard, W. & Cremers, D. 2012, 'A benchmark for the evaluation of RGB-D SLAM systems', IEEE, pp. 573–80.
