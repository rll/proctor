#ifndef PROCTOR_H_
#define PROCTOR_H_

#include <pcl/pcl_base.h>

#include <vtkPolyData.h>

#include "proctor/agent.h"
#include "proctor/scanner.h"
#include "proctor/timer.h"

using namespace pcl;

class Proctor {
public:

  /** a model from the dataset plus metadata */
  typedef struct {
    int id;
    vtkPolyData *mesh;
    float cx, cy, cz;
    float scale;
  } Model;

  enum TimerBin {
    OBTAIN_CLOUD_TRAINING,
    OBTAIN_CLOUD_TESTING,
    AGENT_TRAIN,
    AGENT_TEST,
    NUM_BINS
  };

  /** return a random indices list; use srand() to influence the output */
  static IndicesPtr randomSubset(int n, int r);

  /** read meshes and metadata from disk; this populates models */
  static void readModels(const char *base, int max_models, unsigned int seed);

  /** load/generate training data and pass to agent */
  void train(Agent &agent);

  /** generate testing data and pass to agent; this populates scenes, confusion, and distance */
  void test(Agent &agent, unsigned int seed);

  /** compute and print the precision and recall data */
  void printPrecisionRecall();

  /** print the timing data */
  void printTimer();

  /** print the results of testing */
  void printResults(Agent &agent);

  /** how many models to use in training and testing */
  static const int num_models = 4;

  // parameters for creating registered point cloud
  static const float theta_start = M_PI / 12;
  static const float theta_step = 0;
  static const int theta_count = 1;
  static const float phi_start = 0;
  static const float phi_step = M_PI / 6;
  static const int phi_count = 12;

  /** how many times to test the agent */
  static const int num_trials = 20;

  // parameters for test scans
  static const float theta_min = 0;
  static const float theta_max = M_PI / 6;
  static const float phi_min = 0;
  static const float phi_max = M_PI * 2;

  /** database of models and metadata */
  static Model models[num_models];

  /** the exact parameters used during test() */
  Scanner::Scan scenes[num_trials];

  /** histogram of [scene model][agent guess] */
  int confusion[num_models][num_models];

  /** agent's distance ratings [trial][model candidate] */
  double distance[num_trials][num_models];

  /** total number of correct guesses */
  int trace;

  /** the timer */
  Timer<NUM_BINS> timer;

};

#endif //#ifndef PROCTOR_H_
