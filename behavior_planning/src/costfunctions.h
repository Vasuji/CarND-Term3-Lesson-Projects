#ifndef TERM3_LESSONS_COSTFUNCTIONS_H
#define TERM3_LESSONS_COSTFUNCTIONS_H

class Vehicle;

#include "vehicle.h"
#include "snapshot.h"

using namespace std;

double cost(
        const Vehicle &vehicle,
        const vector<Snapshot> &trajectory,
        const map<int, vector<vector<int>>> &predictions);

#endif //TERM3_LESSONS_COSTFUNCTIONS_H
