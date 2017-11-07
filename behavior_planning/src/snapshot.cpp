#include "snapshot.h"

Snapshot::Snapshot(int lane, int s, double v, double a, const string &state)
        : lane(lane),
          s(s),
          v(v),
          a(a),
          state(state) {}
