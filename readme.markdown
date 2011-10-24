This program tests point cloud based object detection algorithms. I call it the Proctor.

# Installation / Environmental Considerations

This program depends on the following:

* CMake 2.6
* PCL 1.2.0 modified (see below)
* Eigen3
* VTK 5.8

Since the stock Detector implementation depends on a feature not found in PCL 1.2.0, for now, Proctor requires that you checkout the PCL trunk and modify `ia_ransac.h` such that the `using` and `typedef`s in the `SampleConsensusInitialAlignment` class are public.

In addition, you will have to change several constants:
* The path of the princeton benchmark database in main.cpp
* The path of the pcl trunk in CMakeLists.txt

# Usage

This program is hosted on gihub.

Supposing you can get a copy, you have a try by entering the following commands:

``` shell
mkdir build
cd build
cmake ..
make
cd ..
mkdir runtime
cd runtime
../build/main
```

Note that `main` can take up to two arguments:

``` shell
main [MODEL SUBSET SEED] [TESTING SEED]
```

Note that training range scans and features are stored in the runtime directory. If you change the scanning parameters (e.g. resolution), you should delete the `scan_` files. If you change the feature or the feature parameters, you should delete the `feature_` files.

# Class Overview

## Detector

Users can plug in a detection algorithm here.

### Interface

* `train` is called at training time with an array of registered point clouds. The detector should perform any preprocessing here.
* `query` is called at test time with a range scan. The detector should populate the `distance` array with some measure of dissimilarity between the range scan and each model. It should return the index of the best guess model, which probably ought to be the one with least distance.
* `enableVisualization` may be called in the beginning, prior to any calls to `train` or `query`, to indicate that the detector should display its progress on screen.

### Stock Implementation

The stock implementation uses FPFH, RANSAC, and ICP.

#### Training

1. compute the FPFH on a dense sampling points in each model

#### Testing

1. compute the FPFH on a dense sampling of points in the scene
1. for each scene feature point, find the closest matches in any model
   1. for each match, contribute a vote to the corresponding model
1. for each of the top candidate models, do the following:
   1. use RANSAC, using the feature histograms to select form an educated pose guess in each iteration
   1. if the result is closer than a threshold, run ICP to enhance the alignment
   1. the distance to the current model is the fitness score from the above registration procedure
1. guess the model that is the least distant from the scene cloud

#### Visualization

Enabling visualization creates a `CloudViewer`.
During training, it shows the models in red.
During testing, it shows the scene in white in the guessed pose.
It's not that fun to watch, but I think it can become much cooler when the next version of PCL comes out, with the `registerVisualizationCallback` API.

## Proctor

This class contains the code for handling everything around the `Detector`.

### Loading Dataset

This project uses the Princeton Shape Benchmark.

The code reads the `.off` file into a vtk representation and parses the `_info.txt` files.

### Generating Training and Test Data

It controls the scanning of the meshes to produce data appropriate for training and testing.

### Processing Test Results

It computes three popular evaluation metrics from the `Detector`'s outputs:

1. percentage of guesses correct
1. precision-recall curve
1. average vote rank of correct model
1. area under cumulative histogram of correct model rank
1. confusion matrix

## Timer

A timer stores a collection of "bins," which accumulate timing measurements. Both Detector and Proctor have a `timer` member and a `printTimer` method to format the collected data.

## Config

This namespace exposes certain configuration variables.

## Scanner

This class provides a layer of abstraction over mesh-to-cloud conversion. The current implementation uses [SyntheticLidarScanner](https://github.com/daviddoria/SyntheticLidarScanner).

# Things You Might Do

1. whatever the hell you want
1. write a new detection algorithm, duh 
1. try out the noise feature in vtkLidarScanner
1. get rid of vtkLidarScanner and roll your own with vtkRenderWindow::GetZBufferData
1. record individual times instead of adding them up
