#include "costfunctions.h"

const double DESIRED_BUFFER = 1.5;
const int PLANNING_HORIZON = 2;

Predictions filter_predictions_by_lane(const Predictions &predictions, int lane) {
    Predictions predictionsInLane;

    for (auto pair : predictions) {
        int vehicleId = pair.first;
        Prediction prediction = pair.second;

        bool isOwnVehicle = vehicleId == -1;
        if (prediction[0][0] == lane && !isOwnVehicle) {
            predictionsInLane[vehicleId] = prediction;
        }
    }

    return predictionsInLane;
}

bool check_collision(const Snapshot &snapshot, double s_previous, double s_now) {
    double
            s = snapshot.s,
            v = snapshot.v,
            v_other = s_now - s_previous;

    if (s_previous < s) {
        return s_now >= s;
    }

    if (s_previous > s) {
        return s_now <= s;
    }

    if (s_previous == s) {
        return v_other <= v;
    }

    throw runtime_error("impossible state");
}

TrajectoryData getTrajectoryData(const Vehicle &vehicle,
                                 const Trajectory &trajectory,
                                 const Predictions &predictions) {
    Snapshot
            current_snapshot = trajectory[0],
            first = trajectory[1],
            last = trajectory[trajectory.size() - 1];
    double end_distance_to_goal = vehicle.goal_s - last.s;
    int end_lanes_from_goal = abs(vehicle.goal_lane - last.lane);
    auto dt = trajectory.size();
    int proposed_lane = first.lane;
    double avg_speed = (last.s - current_snapshot.s) / (double) dt;

    vector<double> accels;
    double closest_approach = 999999;
    bool collides = false;
    int time_until_collision = -1;
    Predictions filtered = filter_predictions_by_lane(predictions, proposed_lane);

    for (int i = 1; i <= PLANNING_HORIZON; i++) {
        Snapshot snapshot = trajectory[i];

        accels.push_back((double) snapshot.a);

        for (auto pair : filtered) {
            Prediction prediction = pair.second;

            double state_s = prediction[i][1]; // first is lane, second is s
            double last_state_s = prediction[i - 1][1]; // first is lane, second is s
            if (check_collision(snapshot, last_state_s, state_s)) {
                collides = true;
                time_until_collision = i;
            }

            double dist = abs(state_s - snapshot.s);
            if (dist < closest_approach) {
                closest_approach = dist;
            }
        }
    }


    double max_acceleration = *max_element(accels.begin(), accels.end(),
                                           [](double lhs, double rhs) {
                                               return abs(lhs) < abs(rhs);
                                           });

    double rms_acceleration = accumulate(accels.begin(), accels.end(), 0.0,
                                         [](double lhs, double rhs) {
                                             return lhs * lhs + rhs * rhs;
                                         }) / accels.size();

    return TrajectoryData(
            proposed_lane,
            avg_speed,
            max_acceleration,
            rms_acceleration,
            closest_approach,
            end_distance_to_goal,
            end_lanes_from_goal,
            collides,
            time_until_collision);
}

double distance_from_goal_lane(const Vehicle &vehicle,
                               const Trajectory &trajectory,
                               const Predictions &predictions,
                               const TrajectoryData &data) {
    double distance = abs(data.end_distance_to_goal);
    distance = max(distance, 1.0);
    double time_to_goal = distance / data.avg_speed;
    double multiplier = 5 * data.end_lanes_from_goal / time_to_goal;
    return multiplier * REACH_GOAL;
}

double collision_cost(const Vehicle &vehicle,
                      const Trajectory &trajectory,
                      const Predictions &predictions,
                      const TrajectoryData &data) {
    if (data.collides) {
        double time_until_collision = data.time_until_collision;
        return exp(-pow(time_until_collision, 2)) * COLLISION;
    }
    return 0.0;
}

double inefficiency_cost(const Vehicle &vehicle,
                         const Trajectory &trajectory,
                         const Predictions &predictions,
                         const TrajectoryData &data) {
    double speed = data.avg_speed;
    int target_speed = vehicle.target_speed;
    double diff = target_speed - speed;
    double pct = diff / target_speed;
    return pct * pct * EFFICIENCY;
}

double buffer_cost(const Vehicle &vehicle,
                   const Trajectory &trajectory,
                   const Predictions &predictions,
                   const TrajectoryData &data) {
    double closest = data.closest_approach;
    if (abs(closest) <= 0.000001) {
        return 10 * DANGER;
    }

    double timesteps_away = closest / data.avg_speed;
    if (timesteps_away > DESIRED_BUFFER) {
        return 0.0;
    }

    double multiplier = pow(1.0 - timesteps_away / DESIRED_BUFFER, 2);
    return multiplier * DANGER;
}

double change_lane_cost(const Vehicle &vehicle,
                        const Trajectory &trajectory,
                        const Predictions &predictions,
                        const TrajectoryData &data) {
    int currentLane = trajectory[0].lane;
    if (data.end_lanes_from_goal > currentLane) {
        return COMFORT;
    }
    return -COMFORT;
}


double cost(const Vehicle &vehicle,
            const Trajectory &trajectory,
            const Predictions &predictions) {

    vector<CostFunction> costFunctions = {
            distance_from_goal_lane,
            collision_cost,
            inefficiency_cost,
            buffer_cost,
            change_lane_cost
    };

    double cost = 0.0;
    for (CostFunction costFunction : costFunctions) {
        TrajectoryData data = getTrajectoryData(vehicle, trajectory, predictions);
        cost += costFunction(vehicle, trajectory, predictions, data);
    }
    return cost;
}
