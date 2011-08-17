#include "proctor/agent.h"
#include "proctor/proctor.h"

Agent agent;
Proctor proctor;

int main(int argc, char **argv) {
  unsigned int model_seed = 2;
  unsigned int test_seed = time(NULL);
  if (argc >= 2) model_seed = atoi(argv[1]);
  if (argc >= 3) test_seed = atoi(argv[2]);

  Proctor::readModels("/home/pabbeel/wh/benchmark/db", 1814, model_seed);
  proctor.train(agent);
  proctor.test(agent, test_seed);
  proctor.printResults(agent);

  return 0;
}
