#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "types.h"
#include "costfunctions.h"

using namespace std;

static const string STATE_CONSTANT_SPEED = "CS";
static const string STATE_KEEP_LANE = "KL";
static const string STATE_CHANGE_LEFT = "LCL";
static const string STATE_CHANGE_RIGHT = "LCR";
static const string STATE_PREPARE_CHANGE_LEFT = "PLCL";
static const string STATE_PREPARE_CHANGE_RIGHT = "PLCR";

class Vehicle {
public:

    struct collider {

        bool collision; // is there a collision?
        int time; // time collision happens

    };

    int L = 1;

    int preferred_buffer = 6; // impacts "keep lane" behavior.

    int lane;

    int s;

    int v;

    int a;

    int target_speed;

    int lanes_available;

    int max_acceleration;

    int goal_lane;

    int goal_s;

    string state;

    /**
    * Constructor
    */
    Vehicle(int lane, int s, int v, int a);

    /**
    * Destructor
    */
    virtual ~Vehicle();

    void update_state(const Predictions &predictions);

    string getNextState(const Predictions &predictions);

    void configure(vector<int> road_data);

    string display();

    void increment(int dt);

    vector<int> state_at(int t);

    bool collides_with(Vehicle other, int at_time);

    collider will_collide_with(Vehicle other, int timesteps);

    void realize_state(const Predictions &predictions);

    void realize_constant_speed();

    int _max_accel_for_lane(const Predictions &predictions, int lane, int s);

    void realize_keep_lane(const Predictions &predictions);

    void realize_lane_change(const Predictions &predictions, string direction);

    void realize_prep_lane_change(const Predictions &predictions, string direction);

    Prediction generate_predictions(int horizon);

    Trajectory trajectoryForState(string state, const Predictions &predictions);

    long getMinIndex(vector<double> values);

    Snapshot create_snapshot();

    void restore_state_from_snapshot(Snapshot snapshot);
};

#endif
