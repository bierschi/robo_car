//
// Created by christian on 27.10.18.
//

#ifndef MAP_INTERFACE_SLAMMAP_H
#define MAP_INTERFACE_SLAMMAP_H

#include <string>

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"


class SlamMap {

private:
    std::string mapname_;
    ros::Subscriber map_sub_;
    bool save_map_;
    int threshold_occupied_;
    int threshold_free_;
    int mapCounter;

public:
    SlamMap(const std::string& mapname, int threshold_occupied, int threshold_free);

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);
    void createMapFile(const nav_msgs::OccupancyGridConstPtr& map);
    void mapInterface(const nav_msgs::OccupancyGridConstPtr& map);

    bool getSaveMap() const;
    void setSaveMap(bool);
};



#endif //MAP_INTERFACE_SLAMMAP_H
