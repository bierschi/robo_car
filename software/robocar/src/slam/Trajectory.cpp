//
// Created by christian on 30.10.18.
//

#include "slam/Trajectory.h"

Trajectory::Trajectory(): saveTrajectory_(false) {

    ros::NodeHandle n1;
    path_sub_ = n1.subscribe("trajectory", 2, &Trajectory::pathCallback, this);
}

Trajectory::~Trajectory() {

}

void Trajectory::pathCallback(const nav_msgs::PathConstPtr& path) {


    if (saveTrajectory_) {

        std::cout << "size: " <<path->poses.size() << std::endl;
        for (unsigned i=1; i<path->poses.size(); i++) {
            const geometry_msgs::Point& P0 = path->poses[i-1].pose.position;
            const geometry_msgs::Point& P1 = path->poses[i].pose.position;
            std::cout << "x: " << P0.x << " " << "y: " << P0.y << std::endl;
            std::cout << "x: " << P1.x << " " << "y: " << P1.y << std::endl;std::cout <<std::endl;
        }

    }

}

bool Trajectory::getTrajectoryFlag() const {
    return saveTrajectory_;
}

void Trajectory::setTrajectoryFlag(bool saveTrajectory) {
    saveTrajectory_ = saveTrajectory;
}