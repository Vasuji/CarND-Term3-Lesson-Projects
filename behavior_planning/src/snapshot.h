#ifndef TERM3_LESSONS_SNAPSHOT_H
#define TERM3_LESSONS_SNAPSHOT_H

#include <string>

using namespace std;

class Snapshot {
public:

    int lane;
    int s;
    int v;
    int a;
    string state;

    Snapshot(int lane, int s, int v, int a, const string &state);
};


#endif //TERM3_LESSONS_SNAPSHOT_H
