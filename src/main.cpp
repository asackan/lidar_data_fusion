#include <lidar_fusion.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_fusion");
    ros::start();

    lidar_fusion::LidarFusion LF;

    ros::spin();
    return 0; 
}