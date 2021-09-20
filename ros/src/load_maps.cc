#include "load_maps.h"
#include "Converter.h"
#include <thread>
#include <iomanip>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "load_maps");
    ros::start();
    ros::NodeHandle nh;

    // ~~~~~~~~~~~~~~ CHANGE FILENAMES ACCORDINGLY ~~~~~~~~~~~~~~~
    // to do: pass the filenames as parameters as now it needs to
    // be recompiled whenever you change the filenames
    std::string filename_map1 = "/home/victor/.ros/test_map3.bin";
    std::string filename_map2 = "/home/victor/.ros/test_map4.bin";
    std::string filename_voc  = "/home/victor/orb_slam_ws/src/orb_slam_2_ros/orb_slam2/Vocabulary/ORBvoc.txt";
    
    ORB_SLAM2::load_maps loader(filename_map1, filename_map2, filename_voc, &nh);
    while(ros::ok())
    {
        loader.publish_point_clouds();
        ros::spinOnce();
    }
    ros::shutdown();
    return 0;
}

namespace ORB_SLAM2
{
    load_maps::load_maps(const std::string &filename_map1, const std::string &filename_map2, const std::string strVocFile, ros::NodeHandle *nh):
        pub_cloud1_(nh->advertise<sensor_msgs::PointCloud2> ("/orb_point_cloud_1", 1)),
        pub_cloud2_(nh->advertise<sensor_msgs::PointCloud2> ("/orb_point_cloud_2", 1)),
        loop_rate(1)
    {
        // Load ORB Vocabulary
        // Not really necessary for us but otherwise it may crash... As an upside the maps loaded in memory are 
        // 'fully functional' (can be used in ORB-SLAM)
        {
            std::cout << std::endl << "Loading ORB Vocabulary." << std::endl;

            mpVocabulary = new ORBVocabulary();

            //try to load from the binary file
            bool bVocLoad = mpVocabulary->loadFromBinFile(strVocFile + ".bin");

            if(!bVocLoad)
            {
                std::cerr << "Cannot find binary file for vocabulary. " << std::endl;
                std::cerr << "Failed to open at: " << strVocFile+".bin" << std::endl;
                std::cerr << "Trying to open the text file. This could take a while..." << std::endl;
                bool bVocLoad2 = mpVocabulary->loadFromTextFile(strVocFile);
                if(!bVocLoad2)
                {
                    std::cerr << "Wrong path to vocabulary. " << std::endl;
                    std::cerr << "Failed to open at: " << strVocFile << std::endl;
                    exit(-1);
                }
                std::cerr << "Saving the vocabulary to binary for the next time to " << strVocFile+".bin" << endl;
                mpVocabulary->saveToBinFile(strVocFile+".bin");
            }

            std::cout << "Vocabulary loaded!" << std::endl << std::endl;
        }

        if ( !LoadMap(filename_map1, 1) )
        {
            std::cout<<"ERROR IN LOADING MAP: ["<<filename_map1<<"]. Program will now exit." << std::endl;
            return;
        }

        if ( !LoadMap(filename_map2, 2) )
        {
            std::cout<<"ERROR IN LOADING MAP: ["<<filename_map2<<"]. Program will now exit."<< std::endl;
            return;
        }

        cloud1 = MapPointsToPointCloud(map1->GetAllMapPoints());
        cloud2 = MapPointsToPointCloud(map2->GetAllMapPoints());

        std::cout << "Number of points in cloud 1: [" << map1->GetAllMapPoints().size()<<"]\n";
        std::cout << "Number of points in cloud 2: [" << map2->GetAllMapPoints().size()<<"]\n";
        std::cout << "~~~ Publishing clouds ~~~\n";
    }

    bool load_maps::LoadMap(const string &filename, int map_nr)
    {
        unique_lock<mutex>MapPointGlobal(MapPoint::mGlobalMutex);
        std::ifstream in(filename, std::ios_base::binary);
        if (!in) {
            cerr << "Cannot open map file: " << filename << " , you need create it first!" << std::endl;
            return false;
        }

        const rlim_t kNewStackSize = 64L * 1024L * 1024L;   // min stack size = 64 Mb
        const rlim_t kDefaultCallStackSize = GetCurrentCallStackSize();
        if (!SetCallStackSize(kNewStackSize)) {
            std::cerr << "Error changing the call stack size; Aborting" << std::endl;
            return false;
        }

        std::cout << "Loading map file: " << filename << std::endl << std::flush;
        boost::archive::binary_iarchive ia(in, boost::archive::no_header);
        std::cout << "Putting data in map variable" << std::endl << std::flush;
        if (map_nr == 1)
        {
            ia >> map1;
            std::cout << "Putting data in kf_db variable" << std::endl << std::flush;
            ia >> kf_db1;
            kf_db1->SetORBvocabulary(mpVocabulary);
        }
        else 
        if(map_nr == 2)
        {
            ia >> map2;
            std::cout << "Putting data in kf_db variable" << std::endl << std::flush;
            ia >> kf_db2;
            kf_db2->SetORBvocabulary(mpVocabulary);
        }
        else
        {
            return false;
        }

        std::cout << " ... done" << std::endl;

        std::cout << "Map reconstructing" << std::flush;
        
        std::vector<KeyFrame*> vpKFS; 

        if(map_nr == 1)
        {
            vpKFS = map1->GetAllKeyFrames();
        }
        else if(map_nr == 2)
        {
            vpKFS = map2->GetAllKeyFrames();
        }

        // std::vector<KeyFrame*> vpKFS = map1->GetAllKeyFrames();
        unsigned long mnFrameId = 0;
        for (auto it:vpKFS) 
        {
            it->SetORBvocabulary(mpVocabulary);
            it->ComputeBoW();

            if (it->mnFrameId > mnFrameId) 
            {
                mnFrameId = it->mnFrameId;
            }
        }

        Frame::nNextId = mnFrameId;

        std::cout << " ... done" << std::endl;
        in.close();

        SetCallStackSize(kDefaultCallStackSize);

        return true;
    }

    bool load_maps::SetCallStackSize (const rlim_t kNewStackSize) {
        struct rlimit rlimit;
        int operation_result;

        operation_result = getrlimit(RLIMIT_STACK, &rlimit);
        if (operation_result != 0) {
            std::cerr << "Error getting the call stack struct" << std::endl;
            return false;
        }

        if (kNewStackSize > rlimit.rlim_max) {
            std::cerr << "Requested call stack size too large" << std::endl;
            return false;
        }

        if (rlimit.rlim_cur <= kNewStackSize) {
            rlimit.rlim_cur = kNewStackSize;
            operation_result = setrlimit(RLIMIT_STACK, &rlimit);
            if (operation_result != 0) {
                std::cerr << "Setrlimit returned result: " << operation_result << std::endl;
                return false;
            }
            return true;
        }
        return false;
    }

    rlim_t load_maps::GetCurrentCallStackSize () 
    {
        struct rlimit rlimit;
        int operation_result;

        operation_result = getrlimit(RLIMIT_STACK, &rlimit);
        if (operation_result != 0) {
            std::cerr << "Error getting the call stack struct" << std::endl;
            return 16 * 1024L * 1024L; //default
        }
        return rlimit.rlim_cur;
    }

    void load_maps::publish_point_clouds()
    {
        pub_cloud1_.publish(cloud1);
        pub_cloud2_.publish(cloud2);

        loop_rate.sleep();
    }

    sensor_msgs::PointCloud2 load_maps::MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points) {
    if (map_points.size() == 0) {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;

    const int num_channels = 3; // x y z

    cloud.header.stamp = current_frame_time_;
    cloud.header.frame_id = "map";
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};
    for (int i = 0; i<num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

        unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[num_channels];
    for (unsigned int i=0; i<cloud.width; i++) {
        if (map_points.at(i)->nObs >= 2) {
        data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
        data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
        data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
        //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

        memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
    }
}