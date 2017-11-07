#ifndef TYPES_H
#define TYPES_H

#include "snapshot.h"
#include <vector>
#include <map>

using namespace std;

typedef vector<vector<int>> Prediction;
typedef map<int, Prediction> Predictions;
typedef vector<Snapshot> Trajectory;

#endif //TYPES_H
