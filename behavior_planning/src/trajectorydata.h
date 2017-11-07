#ifndef TERM3_LESSONS_TRAJECTORYDATA_H
#define TERM3_LESSONS_TRAJECTORYDATA_H

class TrajectoryData {
public:
    int proposed_lane;
    double avg_speed;
    double max_acceleration;
    double rms_acceleration;
    double closest_approach;
    double end_distance_to_goal;
    int end_lanes_from_goal;
    bool collides;

    TrajectoryData(int proposed_lane, double avg_speed, double max_acceleration, double rms_acceleration,
                   double closest_approach, double end_distance_to_goal, int end_lanes_from_goal, bool collides);
};


#endif //TERM3_LESSONS_TRAJECTORYDATA_H
