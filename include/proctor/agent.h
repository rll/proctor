#ifndef AGENT_H_
#define AGENT_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "proctor/timer.h"

using namespace std;
using namespace pcl;

class Agent {
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
   * populate confidence[candidate model] with some value
   */
  int test(PointCloud<PointNormal>::Ptr scene, double *confidence);

  /** print the timing data */
  void printTimer();

  /** the timer */
  Timer<NUM_BINS> timer;

};

#endif //#ifndef AGENT_H_
