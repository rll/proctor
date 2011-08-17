#ifndef AGENT_H_
#define AGENT_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;
using namespace pcl;

class Agent {
public:

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

};

#endif //#ifndef AGENT_H_
