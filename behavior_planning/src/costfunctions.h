#ifndef TERM3_LESSONS_COSTFUNCTIONS_H
#define TERM3_LESSONS_COSTFUNCTIONS_H

class Vehicle;

#include "types.h"
#include "vehicle.h"
#include "trajectorydata.h"

using namespace std;

static const double COLLISION = 10e6;
static const double DANGER = 10e5;
static const double REACH_GOAL = 10e5;
static const double COMFORT = 10e4;
static const double EFFICIENCY = 10e2;

typedef double (*CostFunction)(const Vehicle &vehicle,
                               const Trajectory &trajectory,
                               const Predictions &predictions,
                               const TrajectoryData &data);

/**
 * A lane further away from the goal lane should have a higher cost. Also, the less time we have until reaching the goal
 * the higher the lane cost should be.
 */
double distance_from_goal_lane(const Vehicle &vehicle,
                               const Trajectory &trajectory,
                               const Predictions &predictions,
                               const TrajectoryData &data);

/**
 * Calculate the overall cost for a trajectory based on all cost functions.
 */
double cost(
        const Vehicle &vehicle,
        const Trajectory &trajectory,
        const Predictions &predictions);

#endif //TERM3_LESSONS_COSTFUNCTIONS_H
