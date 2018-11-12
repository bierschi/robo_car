//
// Created by christian on 30.10.18.
//

#ifndef MAP_INTERFACE_TRAJECTORY_H
#define MAP_INTERFACE_TRAJECTORY_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

class Trajectory {

private:
    ros::Subscriber path_sub_;
    geometry_msgs::PoseStamped* allPoses;
    bool saveTrajectory_;

public:
    Trajectory();
    ~Trajectory();

    void pathCallback(const nav_msgs::PathConstPtr& path);
    bool getTrajectoryFlag() const;
    void setTrajectoryFlag(bool);
};

#endif //MAP_INTERFACE_TRAJECTORY_H
