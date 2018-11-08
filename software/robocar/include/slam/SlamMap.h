//
// Created by christian on 27.10.18.
//

#ifndef MAP_INTERFACE_SLAMMAP_H
#define MAP_INTERFACE_SLAMMAP_H

#include <string>
#include <math.h>

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"

#define PI 3.14159265

/**
 * /CLASS SlamMap
 *
 * creates a SlamMap object to receive and manipulate the hector slam map
 */
class SlamMap {

private:
    std::string mapname_;

    ros::Subscriber map_sub_;
    ros::Subscriber map_metadata_sub_;
    ros::Subscriber pose_sub_;

    bool save_map_;

    int threshold_occupied_, threshold_free_;
    int mapCounter, mapHeight, mapWidth;

    std::vector<int> mapData;

    double origin_pos_x_, origin_pos_y_, origin_pos_z_;
    double origin_or_x_, origin_or_y_, origin_or_z_, origin_or_w_;


public:
    //constructor
    SlamMap(const std::string& mapname="hectorMap", int threshold_occupied=65, int threshold_free=25);
    ~SlamMap();

    //public attributes
    double position_x, position_y, position_z, orientation_w, theta;

    //callbacks
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);
    void mapMetadataCallback(const nav_msgs::MapMetaDataConstPtr& metadata);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose);

    void createPgmMapFile(const nav_msgs::OccupancyGridConstPtr& map);
    void createTxtMapFile(std::string fileName, std::vector<int> mapData);
    void createTxtPositionFile();
    void mapInterface(const nav_msgs::OccupancyGridConstPtr& map);

    //getter and setter
    bool getSaveMap() const;
    void setSaveMap(bool);

    std::vector<int> getMapData();

    double getOriginPosX() const;
    double getOriginPosY() const;
    double getOriginPosZ() const;

};

#endif //MAP_INTERFACE_SLAMMAP_H
