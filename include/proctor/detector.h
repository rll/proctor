#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include "proctor/config.h"
#include "proctor/timer.h"

using namespace std;
using namespace pcl;

class Detector {
public:

  enum TimerBin {
    OBTAIN_FEATURES_TRAINING,
    COMPUTE_FEATURES_TESTING,
    RANSAC,
    ICP,
    NUM_BINS
  };

  /** a cloud and its features */
  typedef struct {
    PointCloud<PointNormal>::Ptr cloud;
    IndicesPtr indices;
    PointCloud<FPFHSignature33>::Ptr features;
  } Entry;

  Entry database[Config::num_models];

  /**
   * do any offline processing
   * models[i] is a registered point cloud of the ith model
   */
  void train(PointCloud<PointNormal>::Ptr *models);

  /**
   * do any online processing
   * scene is a range scan
   * return guessed model number
   * populate distance[candidate model] with some value
   */
  int query(PointCloud<PointNormal>::Ptr scene, double *distance);

  /** start a visualizer; if called, must be called before training/querying */
  void enableVisualization();

  /** print the timing data */
  void printTimer();

  /** the timer */
  Timer<NUM_BINS> timer;

private:

  /** run RANSAC and ICP to judge similarity */
  double computeRegistration(Entry &source, int mi);

  auto_ptr<visualization::CloudViewer> vis;

};

#endif //#ifndef DETECTOR_H_
