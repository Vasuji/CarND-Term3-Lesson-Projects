#ifndef TERM3_LESSONS_SNAPSHOT_H
#define TERM3_LESSONS_SNAPSHOT_H

#include <string>

using namespace std;

class Snapshot {
    int lane;
    int s;
    double v;
    double a;
    string state;

public:
    Snapshot(int lane, int s, double v, double a, const string &state);
};


#endif //TERM3_LESSONS_SNAPSHOT_H
