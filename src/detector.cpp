#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include "proctor/detector.h"
#include "proctor/ia_ransac_sub.h"
#include "proctor/proctor.h"

/** create a viewport for a model and add the model */
class show_model {
public:
  show_model(int mi, PointCloud<PointNormal>::ConstPtr model) : mi(mi), model(model) {}
  void operator()(visualization::PCLVisualizer &v) {
    int vp;
    v.createViewPort(double(mi) / Config::num_models, 0, double(mi + 1) / Config::num_models, 1, vp);
    {
      stringstream ss;
      ss << "model_" << mi;
      v.addPointCloud(model, visualization::PointCloudColorHandlerCustom<PointNormal>(model, 0xc0, 0x00, 0x40), ss.str(), vp);
    }
    {
      stringstream ss;
      ss << "aligned_" << mi;
      v.addPointCloud(model, visualization::PointCloudColorHandlerCustom<PointNormal>(model, 0xc0, 0x00, 0x40), ss.str(), vp);
    }
  }
private:
  int mi;
  PointCloud<PointNormal>::ConstPtr model;
};

/** show an aligned point cloud in the specified viewport */
class show_aligned {
public:
  show_aligned(int mi, PointCloud<PointNormal>::ConstPtr aligned) : mi(mi), aligned(aligned) {}
  void operator()(visualization::PCLVisualizer &v) {
    stringstream ss;
    ss << "aligned_" << mi;
    v.updatePointCloud(aligned, visualization::PointCloudColorHandlerCustom<PointNormal>(aligned, 0xff, 0xff, 0xff), ss.str());
  }
private:
  int mi;
  PointCloud<PointNormal>::ConstPtr aligned;
};

/** run the feature */
PointCloud<FPFHSignature33>::Ptr compute_features(PointCloud<PointNormal>::Ptr cloud, IndicesPtr indices) {
  PointCloud<FPFHSignature33>::Ptr features (new PointCloud<FPFHSignature33>());
  FPFHEstimation<PointNormal, PointNormal, FPFHSignature33> fpfh;
  fpfh.setRadiusSearch(0.1);
  fpfh.setInputCloud(cloud);
  fpfh.setIndices(indices);
  KdTree<PointNormal>::Ptr kdt (new KdTreeFLANN<PointNormal>());
  fpfh.setSearchMethod(kdt);
  fpfh.setInputNormals(cloud);
  fpfh.compute(*features);
  return features;
}

/** try to load the features from disk, or do it from scratch. for training only */
PointCloud<FPFHSignature33>::Ptr obtain_features(int mi, PointCloud<PointNormal>::Ptr cloud, IndicesPtr indices) {
  char name[17];
  sprintf(name, "feature_%04d.pcd", Proctor::models[mi].id);
  if (ifstream(name)) {
    PointCloud<FPFHSignature33>::Ptr features (new PointCloud<FPFHSignature33>());
    io::loadPCDFile(name, *features);
    return features;
  } else {
    PointCloud<FPFHSignature33>::Ptr features = compute_features(cloud, indices);
    io::savePCDFile(name, *features);
    return features;
  }
}

void Detector::train(PointCloud<PointNormal>::Ptr *models) {
  srand(time(NULL));
  for (int mi = 0; mi < Config::num_models; mi++) {
    Entry &e = database[mi];
    e.cloud = models[mi];
    e.indices = Proctor::randomSubset(e.cloud->points.size(), 512);
    timer.start();
    e.features = obtain_features(mi, e.cloud, e.indices);
    timer.stop(OBTAIN_FEATURES_TRAINING);
    cout << "finished model " << mi << endl;
    if (vis.get()) vis->runOnVisualizationThreadOnce(show_model(mi, e.cloud));
  }
}

int Detector::query(PointCloud<PointNormal>::Ptr scene, double *distance) {
  Entry e;
  e.cloud = scene;
  e.indices = Proctor::randomSubset(e.cloud->points.size(), 128);
  timer.start();
  e.features = compute_features(e.cloud, e.indices);
  timer.stop(COMPUTE_FEATURES_TESTING);
  double best = numeric_limits<double>::infinity();
  int guess = -1;
  for (int mi = 0; mi < Config::num_models; mi++) {
    distance[mi] = computeRegistration(e, mi);
    cout << mi << ": " << distance[mi] << endl;
    if (distance[mi] < best) {
      guess = mi;
      best = distance[mi];
    }
  }
  return guess;
}

void Detector::enableVisualization() {
  vis.reset(new visualization::CloudViewer("Detector Visualization"));
}

void Detector::printTimer() {
  printf(
    "obtain training features: %10.3f sec\n"
    "compute testing features: %10.3f sec\n"
    "RANSAC:                   %10.3f sec\n"
    "ICP:                      %10.3f sec\n",
    timer[OBTAIN_FEATURES_TRAINING],
    timer[COMPUTE_FEATURES_TESTING],
    timer[RANSAC],
    timer[ICP]
  );
}

double Detector::computeRegistration(Entry &source, int mi) {
  Entry &target = database[mi];

  timer.start();
  SubsetSAC_IA<PointNormal, PointNormal, FPFHSignature33> ia_ransac_sub;
  PointCloud<PointNormal>::Ptr aligned (new PointCloud<PointNormal>());
  ia_ransac_sub.setSourceIndices(source.indices);
  ia_ransac_sub.setTargetIndices(target.indices);
  ia_ransac_sub.setMinSampleDistance(0.05);
  ia_ransac_sub.setMaxCorrespondenceDistance(0.1);
  ia_ransac_sub.setMaximumIterations(256);
  ia_ransac_sub.setInputCloud(source.cloud);
  ia_ransac_sub.setSourceFeatures(source.features);
  ia_ransac_sub.setInputTarget(target.cloud);
  ia_ransac_sub.setTargetFeatures(target.features);
  ia_ransac_sub.align(*aligned);
  if (vis.get()) vis->runOnVisualizationThreadOnce(show_aligned(mi, aligned));
  if (ia_ransac_sub.getFitnessScore() > 0.006) return ia_ransac_sub.getFitnessScore();
  timer.stop(RANSAC);

  timer.start();
  IterativeClosestPoint<PointNormal, PointNormal> icp;
  PointCloud<PointNormal>::Ptr aligned2 (new PointCloud<PointNormal>());
  icp.setInputCloud(aligned);
  icp.setInputTarget(target.cloud);
  icp.setMaxCorrespondenceDistance(0.1);
  icp.setMaximumIterations(64);
  icp.align(*aligned2);
  timer.stop(ICP);
  if (vis.get()) vis->runOnVisualizationThreadOnce(show_aligned(mi, aligned2));
  return icp.getFitnessScore();
}
