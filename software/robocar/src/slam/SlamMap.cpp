//
// Created by christian on 27.10.18.
//

#include <geos_c.h>
#include "slam/SlamMap.h"


SlamMap::SlamMap(const std::string& mapname, int threshold_occupied, int threshold_free)
        : mapname_(mapname),
          save_map_(false),
          threshold_occupied_(threshold_occupied),
          threshold_free_(threshold_free),
          mapCounter(0)
{

    ros::NodeHandle n;
    ROS_INFO("Waiting for the map!");
    map_sub_ = n.subscribe("map", 2, &SlamMap::mapCallback, this);

}



void SlamMap::mapCallback(const nav_msgs::OccupancyGridConstPtr& map) {

    ROS_INFO("Received a %d X %d map @ %.3f m/pix", map->info.width, map->info.height, map->info.resolution);
    //sleep(2);
    //if (save_map_)
    createMapFile(map);

    //mapInterface(map);

}

void SlamMap::mapInterface(const nav_msgs::OccupancyGridConstPtr& map) {

    std::cout << "Width: " << map->info.width << " Height: " << map->info.height << " Resolution: "
              << map->info.resolution << std::endl;
    /*
    for (std::vector<int>::size_type i = 0; i != map->data.size() ; i++) {
        printf("%d ", map->data[i]);
    }*/ //data -1, 0-100

    int mapData[map->data.size()];
    std::cout << "size: " << map->data.size() << std::endl; //map->info.height * map->info.width

    for (unsigned int y = 0; y < map->info.height; y++) {
        for (unsigned int x = 0; x < map->info.width; x++) {
            unsigned int i = x + (map->info.height - y - 1) * map->info.width;

            if (map->data[i] >= 0 && map->data[i] <= threshold_free_) {

                //fputc(254, out);
                mapData[i] = 254;

            } else if (map->data[i] >= threshold_occupied_) {

                //fputc(000, out);
                mapData[i] = 0;

            } else {

                //fputc(205, out);
                mapData[i] = 205;

            }
        }
    }

    FILE* out = fopen("intarray_SlamMap.txt", "w");
    for( int s = 1; s < sizeof(mapData)/sizeof(mapData[0]); s++) {

        fprintf(out, "%d ", mapData[s]);

        if (s && s%map->info.width == 0) {

            fprintf(out, "\n");

        }
    }
}

void SlamMap::createMapFile(const nav_msgs::OccupancyGridConstPtr& map) {

    std::string mapCounterStr = std::to_string(mapCounter);
    std::string mapdatafile = mapname_ + mapCounterStr +  ".pgm";

    ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());

    FILE* out = fopen(mapdatafile.c_str(), "w");

    if (!out) {

        ROS_ERROR("Could not save map file to %s", mapdatafile.c_str());
        return;

    }

    fprintf(out, "P2\n %d %d\n255\n",
            map->info.width,
            map->info.height);
    for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {

            unsigned int i = x + (map->info.height -y -1) * map->info.width;

            if (map->data[i] >= 0 && map->data[i] <= threshold_free_) {

                //fputc(254, out);
                fprintf(out, "%d ",254);

            } else if (map->data[i] >= threshold_occupied_) {

                //fputc(000, out);
                fprintf(out, "%d ",0);

            } else {

                //fputc(205, out);
                fprintf(out, "%d ",205);

            }
        }
        fprintf(out, "\n");
    }

    fclose(out);

    ROS_INFO("DONE\n");
    mapCounter++;

}

bool SlamMap::getSaveMap() const {
    return save_map_;
}

void SlamMap::setSaveMap(bool save_map) {
    save_map_ = save_map;
}