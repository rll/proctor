#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "proctor/timer.h"

using namespace std;
using namespace pcl;

class Detector {
public:

  enum TimerBin {
    OBTAIN_FEATURES_TRAINING,
    COMPUTE_FEATURES_TESTING,
    COMPUTE_REGISTRATION,
    NUM_BINS
  };

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

  /** print the timing data */
  void printTimer();

  /** the timer */
  Timer<NUM_BINS> timer;

};

#endif //#ifndef DETECTOR_H_
