#include <Eigen/Dense>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/time_synchronizer.h>
#include <laser_geometry/laser_geometry.h>

namespace lidar_fusion
{

using Laser = sensor_msgs::LaserScan;
using Sync  = message_filters::TimeSynchronizer<Laser, Laser>;

float inf = std::numeric_limits<float>::infinity();

class MergeLidar
{
public:
     MergeLidar();
    ~MergeLidar(){}

    void lidar_1_Cb(const Laser::ConstPtr&);
    void lidar_2_Cb(const Laser::ConstPtr&);

    void pointcloud_to_laserscan(Eigen::MatrixXf, pcl::PCLPointCloud2*);
    //void reconfigureCallback(laserscan_multi_mergerConfig&, uint32_t);

    void fuse_data(const ros::TimerEvent&);
    void fusion(const Laser::ConstPtr&);

    void set_mRanges_size(Laser&);

private:
    ros::NodeHandle n;
    ros::Timer      timer;
    
    ros::Subscriber scan_1_sub, scan_2_sub;
    ros::Publisher  scan_pub;

    boost::shared_ptr<Laser const> Laser_ptr;

    Laser mScan_1, mScan_2;

    uint32_t mRanges_size;
};

}