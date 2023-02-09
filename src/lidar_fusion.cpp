#include <lidar_fusion.hpp>

namespace lidar_fusion
{

MergeLidar::MergeLidar()
{
    scan_1_sub = n.subscribe("/scan_1", 1, &MergeLidar::lidar_1_Cb, this);
    scan_2_sub = n.subscribe("/scan_2", 1, &MergeLidar::lidar_2_Cb, this);

    scan_pub   = n.advertise<Laser>("concatenated_scan" ,1);

    timer = n.createTimer(ros::Duration(1), &MergeLidar::fuse_data, this);  
}

void MergeLidar::lidar_1_Cb(const Laser::ConstPtr& scan)
{ mScan_1.ranges = scan->ranges; }

void MergeLidar::lidar_2_Cb(const Laser::ConstPtr& scan)
{ mScan_2.ranges = scan->ranges; }

void MergeLidar::fuse_data(const ros::TimerEvent&)
{
    Laser_ptr = ros::topic::waitForMessage<Laser>("scan_1");
    if(Laser_ptr == NULL){ ROS_INFO("No laser messages received"); }
    else { mScan_1 = *Laser_ptr; }
    
    Laser_ptr = ros::topic::waitForMessage<Laser>("scan_2");
    if(Laser_ptr == NULL){ ROS_INFO("No laser messages received"); }
    else { mScan_2 = *Laser_ptr; }

    Sync sync(scan_1_sub, scan_2_sub, 2);
    sync.registerCallback(boost::bind(&fusion, _1, _2));
}

void MergeLidar::fusion(const Laser::ConstPtr& scan)
{
    //Solve all of perception here
}

void MergeLidar::set_mRanges_size(Laser& output)
{
    mRanges_size = std::ceil((output.angle_max - output.angle_min) /
        output.angle_increment);
    
    output.ranges.assign(mRanges_size, inf);
    output.intensities.assign(mRanges_size, 0);
}

}