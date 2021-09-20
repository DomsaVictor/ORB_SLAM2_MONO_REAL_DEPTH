#ifndef LOAD_MAPS_H
#define LOAD_MAPS_H

#include <string>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <sys/resource.h>
#include <vector>
#include <iostream>
#include <thread>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/sync_policies/approximate_time.h>


namespace ORB_SLAM2
{
    class FrameDrawer;
    class Map;
    class Tracking;
    class LocalMapping;
    class LoopClosing;

    class load_maps
    {
        public:
            load_maps(const std::string &filename_map1, const std::string &filename_map2, const std::string strVocFile, ros::NodeHandle *nh);

            /*
            modified orb_slam function to load map:
                loads the maps in the private variables "map1" and "map2"

                input: 
                    filename:   absolute file pathe and name to read.
                    map_nr:     it can either be 1 or 2. Selects the private variable to be used. -> "hacked" as passing the global variable as pram is a little bit more troublesome...
            */
            bool LoadMap(const std::string &filename, int map_nr);
            
            void publish_point_clouds();

            int map_number;
            
            sensor_msgs::PointCloud2 cloud1;
            sensor_msgs::PointCloud2 cloud2;
            
        private:
            ros::NodeHandle nh_;
            ros::Publisher pub_cloud1_;
            ros::Publisher pub_cloud2_;
            
            ros::Rate loop_rate;

            // file names
            std::string f_n1;
            std::string f_n2;

            ORBVocabulary* mpVocabulary;

            Map* map1;
            Map* map2;

            ros::Time current_frame_time_;

            KeyFrameDatabase* kf_db1;
            KeyFrameDatabase* kf_db2;

            // "helper" functions for loading maps (from ORB_SLAM2)
            bool SetCallStackSize (const rlim_t kNewStackSize);
            rlim_t GetCurrentCallStackSize ();
            sensor_msgs::PointCloud2 MapPointsToPointCloud(std::vector<ORB_SLAM2::MapPoint*> map_points);
    };
}

#endif // LOAD_MAPS_H