#include <cstdio>

#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>

#include "proctor/proctor.h"

Proctor::Model Proctor::models[Proctor::num_models];

IndicesPtr Proctor::randomSubset(int n, int r) {
  IndicesPtr subset (new vector<int>());
  if (r < n / 2) {
    vector<int> bag (n);
    for (int i = 0; i < n; i++) bag[i] = i;
    int edge = n;
    subset->resize(r);
    for (int i = 0; i < r; i++) {
      int pick = rand() % edge;
      (*subset)[i] = bag[pick];
      bag[pick] = bag[--edge];
    }
  } else {
    subset->resize(n);
    for (int i = 0; i < n; i++) (*subset)[i] = i;
    for (int edge = n; edge > r; ) {
      int pick = rand() % edge;
      (*subset)[pick] = (*subset)[--edge];
    }
    subset->resize(r);
  }
  return subset;
}

void Proctor::readModels(const char *base, int max_models, unsigned int seed) {
  srand(seed);
  IndicesPtr model_subset = randomSubset(max_models, num_models);
  for (int mi = 0; mi < num_models; mi++) {
    int id = (*model_subset)[mi];
    char path[256];
    FILE *file;
    int vertices, faces, edges;

    models[mi].id = id;
    snprintf(path, sizeof(path), "%s/%d/m%d/m%d.off", base, id / 100, id, id);
    file = fopen(path, "r");
    int line = 1;

    // read header
    if (fscanf(file, "OFF\n%d %d %d\n", &vertices, &faces, &edges) != 3) {
      cerr << "invalid OFF header in file " << path << endl;
      exit(-1);
    } else {
      line += 2;
    }

    // read vertices
    vtkFloatArray *fa = vtkFloatArray::New();
    fa->SetNumberOfComponents(3);
    float (*v)[3] = reinterpret_cast<float (*)[3]>(fa->WritePointer(0, 3 * vertices));
    for (int j = 0; j < vertices; j++) {
      if (fscanf(file, "%f %f %f\n", &v[j][0], &v[j][1], &v[j][2]) != 3) {
        cerr << "invalid vertex in file " << path << " on line " << line << endl;
        exit(-1);
      } else {
        ++line;
      }
    }
    vtkPoints *p = vtkPoints::New();
    p->SetData(fa); fa->Delete();

    // read faces
    vtkCellArray *ca = vtkCellArray::New();
    vtkIdType (*f)[4] = reinterpret_cast<vtkIdType (*)[4]>(ca->WritePointer(faces, 4 * faces));
    for (int j = 0; j < faces; j++) {
      f[j][0] = 3; // only supports triangles...
      if (fscanf(file, "3 %lld %lld %lld\n", &f[j][1], &f[j][2], &f[j][3]) != 3) {
        cerr << "invalid face in file " << path << " on line " << line << endl;
        exit(-1);
      } else {
        ++line;
      }
    }

    fclose(file);

    models[mi].mesh = vtkPolyData::New(); // lives forever
    models[mi].mesh->SetPoints(p); p->Delete();
    models[mi].mesh->SetPolys(ca); ca->Delete();

    // read metadata
    snprintf(path, sizeof(path), "%s/%d/m%d/m%d_info.txt", base, id / 100, id, id);
    file = fopen(path, "r");
    while (!feof(file)) {
      char buf[256];
      fgets(buf, sizeof(buf), file);
      if (!strncmp("center: ", buf, 8)) {
        if (sscanf(buf, "center: (%f,%f,%f)\n", &models[mi].cx, &models[mi].cy, &models[mi].cz) != 3) {
          cerr << "invalid centroid in file " << path << endl;
          cerr << buf;
          exit(-1);
        }
      } else if (!strncmp("scale: ", buf, 7)) {
        if (sscanf(buf, "scale: %f\n", &models[mi].scale) != 1) {
          cerr << "invalid scale in file " << path << endl;
          cerr << buf;
          exit(-1);
        }
      }
    } // end while over info lines
    fclose(file);
  } // end for over models
}

void Proctor::train(Agent &agent) {
  PointCloud<PointNormal>::Ptr clouds[num_models];
  for (int mi = 0; mi < num_models; mi++) {
    clouds[mi] = PointCloud<PointNormal>::Ptr(new PointCloud<PointNormal>());
    for (int ti = 0; ti < theta_count; ti++) {
      for (int pi = 0; pi < phi_count; pi++) {
        *clouds[mi] += *Scanner::getCloudCached(mi, ti, pi);
        flush(cout << '.');
      }
    }
    cout << " finished model " << mi << " (" << models[mi].id << ")" << endl;
  }
  cout << "start training" << endl;
  agent.train(clouds);
  cout << "finish training" << endl;
}

void Proctor::test(Agent &agent, unsigned int seed) {
  srand(seed);
  const float theta_scale = (theta_max - theta_min) / RAND_MAX;
  const float phi_scale = (phi_max - phi_min) / RAND_MAX;

  // prepare test vectors in advance
  for (int ni = 0; ni < num_trials; ni++) {
    scenes[ni].mi = rand() % num_models;
    scenes[ni].theta = theta_min + rand() * theta_scale;
    scenes[ni].phi = phi_min + rand() * phi_scale;
  }

  // run the tests
  cout << "start testing" << endl;
  int trace = 0;
  memset(confusion, 0, sizeof(confusion));
  for (int ni = 0; ni < num_trials; ni++) {
    cout << "--------" << endl;
    PointCloud<PointNormal>::Ptr scene = Scanner::getCloud(scenes[ni]);
    cout << "scanned model " << scenes[ni].mi << endl;
    int guess = agent.test(scene, confidence[ni]);
    cout << "agent guessed " << guess << endl;
    confusion[scenes[ni].mi][guess]++;
    if (guess == scenes[ni].mi) trace++;
  }

  // results output
  // TODO: precision-recall
  printf("%d of %d correct (%.2f%%)\n", trace, num_trials, float(trace) / num_trials * 100);
  printf("confusion matrix (unscaled):\n");
  for (int i = 0; i < num_models; i++) {
    for (int j = 0; j < num_models; j++) {
      printf(" %3d", confusion[i][j]);
    }
    printf("\n");
  }
}
