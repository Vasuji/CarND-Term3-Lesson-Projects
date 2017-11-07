#include "trajectorydata.h"

TrajectoryData::TrajectoryData(int proposed_lane,
                               double avg_speed,
                               double max_acceleration,
                               double rms_acceleration,
                               double closest_approach,
                               double end_distance_to_goal,
                               int end_lanes_from_goal,
                               bool collides,
                               int time_until_collision)
        : proposed_lane(proposed_lane),
          avg_speed(avg_speed),
          max_acceleration(max_acceleration),
          rms_acceleration(rms_acceleration),
          closest_approach(closest_approach),
          end_distance_to_goal(end_distance_to_goal),
          end_lanes_from_goal(end_lanes_from_goal),
          collides(collides),
          time_until_collision(time_until_collision) {}
