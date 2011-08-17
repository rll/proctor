#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include "proctor/agent.h"
#include "proctor/ia_ransac_sub.h"
#include "proctor/proctor.h"

/** a cloud and its features */
typedef struct {
  PointCloud<PointNormal>::Ptr cloud;
  IndicesPtr indices;
  PointCloud<FPFHSignature33>::Ptr features;
} Entry;

Entry database[Proctor::num_models];

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

/** run RANSAC and ICP to judge similarity */
double compute_registration(Entry source, Entry target) {
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
  if (ia_ransac_sub.getFitnessScore() > 0.004) return ia_ransac_sub.getFitnessScore();

  IterativeClosestPoint<PointNormal, PointNormal> icp;
  PointCloud<PointNormal>::Ptr aligned2 (new PointCloud<PointNormal>());
  icp.setInputCloud(aligned);
  icp.setInputTarget(target.cloud);
  icp.setMaxCorrespondenceDistance(0.1);
  icp.setMaximumIterations(64);
  icp.align(*aligned2);
  return icp.getFitnessScore();
}

void Agent::train(PointCloud<PointNormal>::Ptr *models) {
  srand(time(NULL));
  for (int mi = 0; mi < Proctor::num_models; mi++) {
    Entry &e = database[mi];
    e.cloud = models[mi];
    e.indices = Proctor::randomSubset(e.cloud->points.size(), 512);
    timer.start();
    e.features = obtain_features(mi, e.cloud, e.indices);
    timer.stop(OBTAIN_FEATURES_TRAINING);
    cout << "finished model " << mi << endl;
  }
}

int Agent::test(PointCloud<PointNormal>::Ptr scene, double *confidence) {
  Entry e;
  e.cloud = scene;
  e.indices = Proctor::randomSubset(e.cloud->points.size(), 128);
  timer.start();
  e.features = compute_features(e.cloud, e.indices);
  timer.stop(COMPUTE_FEATURES_TESTING);
  double best = numeric_limits<double>::infinity();
  int guess = -1;
  for (int mi = 0; mi < Proctor::num_models; mi++) {
    timer.start();
    confidence[mi] = compute_registration(e, database[mi]);
    timer.stop(COMPUTE_REGISTRATION);
    cout << mi << ": " << confidence[mi] << endl;
    if (confidence[mi] < best) {
      guess = mi;
      best = confidence[mi];
    }
  }
  return guess;
}

void Agent::printTimer() {
  printf(
    "obtain training features: %10.3f sec\n"
    "compute testing features: %10.3f sec\n"
    "compute registration:     %10.3f sec\n",
    timer[OBTAIN_FEATURES_TRAINING],
    timer[COMPUTE_FEATURES_TESTING],
    timer[COMPUTE_REGISTRATION]
  );
}
