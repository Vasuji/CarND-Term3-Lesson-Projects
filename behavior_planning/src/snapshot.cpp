#include "snapshot.h"

Snapshot::Snapshot(int lane, int s, int v, int a, const string &state)
        : lane(lane),
          s(s),
          v(v),
          a(a),
          state(state) {}
