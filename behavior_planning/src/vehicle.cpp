#include <iostream>
#include "vehicle.h"
#include "snapshot.h"
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include <cassert>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    state = STATE_CONSTANT_SPEED;
    max_acceleration = -1;

}

Vehicle::~Vehicle() {}

void Vehicle::update_state(const Predictions &predictions) {
    /*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */

    state = getNextState(predictions);
}

string Vehicle::getNextState(const Predictions &predictions) {
    vector<string> possibleStates = {STATE_KEEP_LANE};

    bool changeRightPossible = lane > 0;
    if (changeRightPossible) {
        possibleStates.push_back(STATE_CHANGE_RIGHT);
    }

    bool changeLeftPossible = lane < lanes_available - 1;
    if (changeLeftPossible) {
        possibleStates.push_back(STATE_CHANGE_LEFT);
    }

    // get costs for all possible states
    vector<double> costs;
    for (const string &possibleState : possibleStates) {
        vector<Snapshot> trajectory = trajectoryForState(possibleState, predictions);
        costs.push_back(cost(*this, trajectory, predictions));
    }

    // return the state with minimum cost
    return possibleStates[getMinIndex(costs)];
}


Snapshot Vehicle::create_snapshot() {
    return Snapshot(lane, s, v, a, state);
}

void Vehicle::restore_state_from_snapshot(Snapshot snapshot) {
    lane = snapshot.lane;
    s = snapshot.s;
    v = snapshot.v;
    a = snapshot.a;
    state = snapshot.state;
}

Trajectory Vehicle::trajectoryForState(string state, const Predictions &predictions) {
    int horizon = 5;

    Predictions predictionsCopy = predictions;

    Snapshot snapshot = create_snapshot();

    // pretend to be in new proposed state
    this->state = state;
    vector<Snapshot> trajectory = {snapshot};

    for (int i = 0; i < horizon; i++) {
        restore_state_from_snapshot(snapshot);
        this->state = state;
        realize_state(predictionsCopy);
        assert(0 <= lane < lanes_available);
        increment(1);
        trajectory.push_back(create_snapshot());

        // need to remove first prediction for each vehicle
        for (auto item : predictionsCopy) {
            Prediction prediction = item.second;
            prediction.erase(prediction.begin());
        }
    }

    // restore state from snapshot
    restore_state_from_snapshot(snapshot);

    return trajectory;
}


void Vehicle::configure(vector<int> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}

string Vehicle::display() {

    ostringstream oss;

    oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";

    return oss.str();
}

void Vehicle::increment(int dt = 1) {

    this->s += this->v * dt;
    this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t) {

    /*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

    /*
    Simple collision detection.
    */
    vector<int> check1 = state_at(at_time);
    vector<int> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

    Vehicle::collider collider_temp;
    collider_temp.collision = false;
    collider_temp.time = -1;

    for (int t = 0; t < timesteps + 1; t++) {
        if (collides_with(other, t)) {
            collider_temp.collision = true;
            collider_temp.time = t;
            return collider_temp;
        }
    }

    return collider_temp;
}

void Vehicle::realize_state(const Predictions &predictions) {
    /*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if (state == "CS") {
        realize_constant_speed();
    } else if (state == "KL") {
        realize_keep_lane(predictions);
    } else if (state == "LCL") {
        realize_lane_change(predictions, "L");
    } else if (state == "LCR") {
        realize_lane_change(predictions, "R");
    } else if (state == "PLCL") {
        realize_prep_lane_change(predictions, "L");
    } else if (state == "PLCR") {
        realize_prep_lane_change(predictions, "R");
    }

}

void Vehicle::realize_constant_speed() {
    a = 0;
}

int Vehicle::_max_accel_for_lane(const Predictions &predictions, int lane, int s) {

    int delta_v_til_target = target_speed - v;
    int max_acc = min(max_acceleration, delta_v_til_target);

    auto it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while (it != predictions.end()) {

        int v_id = it->first;

        vector<vector<int> > v = it->second;

        if ((v[0][0] == lane) && (v[0][1] > s)) {
            in_front.push_back(v);

        }
        it++;
    }

    if (!in_front.empty()) {
        int min_s = 1000;
        vector<vector<int>> leading = {};
        for (auto &i : in_front) {
            if ((i[0][1] - s) < min_s) {
                min_s = (i[0][1] - s);
                leading = i;
            }
        }

        int next_pos = leading[1][1];
        int my_next = s + this->v;
        int separation_next = next_pos - my_next;
        int available_room = separation_next - preferred_buffer;
        max_acc = min(max_acc, available_room);
    }

    return max_acc;

}

void Vehicle::realize_keep_lane(const Predictions &predictions) {
    this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(const Predictions &predictions, string direction) {
    int delta = -1;
    if (direction == "L") {
        delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(const Predictions &predictions, string direction) {
    int delta = -1;
    if (direction == "L") {
        delta = 1;
    }
    int lane = this->lane + delta;

    auto it = predictions.begin();
    vector<vector<vector<int> > > at_behind;
    while (it != predictions.end()) {
        int v_id = it->first;
        vector<vector<int> > v = it->second;

        if ((v[0][0] == lane) && (v[0][1] <= this->s)) {
            at_behind.push_back(v);

        }
        it++;
    }
    if (!at_behind.empty()) {

        int max_s = -1000;
        vector<vector<int> > nearest_behind = {};
        for (auto &i : at_behind) {
            if ((i[0][1]) > max_s) {
                max_s = i[0][1];
                nearest_behind = i;
            }
        }
        int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
        int delta_v = this->v - target_vel;
        int delta_s = this->s - nearest_behind[0][1];
        if (delta_v != 0) {

            int time = -2 * delta_s / delta_v;
            int a;
            if (time == 0) {
                a = this->a;
            } else {
                a = delta_v / time;
            }
            if (a > this->max_acceleration) {
                a = this->max_acceleration;
            }
            if (a < -this->max_acceleration) {
                a = -this->max_acceleration;
            }
            this->a = a;
        } else {
            int my_min_acc = max(-this->max_acceleration, -delta_s);
            this->a = my_min_acc;
        }

    }

}

Prediction Vehicle::generate_predictions(int horizon = 10) {

    Prediction predictions;
    for (int i = 0; i < horizon; i++) {
        vector<int> check1 = state_at(i);
        vector<int> lane_s = {check1[0], check1[1]};
        predictions.push_back(lane_s);
    }
    return predictions;

}

long Vehicle::getMinIndex(vector<double> values) {
    return distance(values.begin(), min_element(values.begin(), values.end()));
}
