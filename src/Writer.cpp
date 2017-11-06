//
// Created by jvonbehren on 06.11.17.
//

#include "point_cloud_io/Writer.hpp"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace ros;
using namespace pcl;
using namespace pcl::io;

namespace point_cloud_io
{

Writer::Writer(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      filePrefix_("point_cloud"),
      fileEnding_("ply")
{
    if (!readParameters()) ros::requestShutdown();
}

bool Writer::readParameters()
{
    bool allParametersRead = true;
    string topics_string;

    if (!nodeHandle_.getParam("folder_path", folderPath_)) allParametersRead = false;
    nodeHandle_.getParam("file_prefix", filePrefix_);
    nodeHandle_.getParam("file_ending", fileEnding_);
    nodeHandle_.getParam("add_counter_to_path", addCounterToPath_);
    nodeHandle_.getParam("add_frame_id_to_path", addFrameIdToPath_);
    nodeHandle_.getParam("add_stamp_sec_to_path", addStampSecToPath_);
    nodeHandle_.getParam("add_stamp_nsec_to_path", addStampNSecToPath_);

    return allParametersRead;
}

void Writer::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    ROS_INFO_STREAM("Received point cloud with " << cloud->height * cloud->width << " points.");
    stringstream filePath;
    filePath << folderPath_ << "/";
    if (!filePrefix_.empty())
    {
        filePath << filePrefix_;
    }
    if (addCounterToPath_)
    {
        filePath << "_" << counter_;
        counter_++;
    }
    if (addFrameIdToPath_)
    {
        filePath << "_" << cloud->header.frame_id;
    }
    if (addStampSecToPath_)
    {
        filePath << "_" << cloud->header.stamp.sec;
    }
    if (addStampNSecToPath_)
    {
        filePath << "_" << cloud->header.stamp.nsec;
    }
    filePath << ".";
    filePath << fileEnding_;

    if (fileEnding_ == "ply")
    {
        // Write .ply file.
        PointCloud<PointXYZRGBNormal> pclCloud;
        fromROSMsg(*cloud, pclCloud);

        PLYWriter writer;
        if (writer.write(filePath.str(), pclCloud) != 0)
        {
            ROS_ERROR_STREAM("Something went wrong when trying to write the point cloud file.");
            return;
        }
    }
    else
    {
        ROS_ERROR_STREAM("Data format not supported.");
        return;
    }

    ROS_INFO_STREAM("Saved point cloud to " << filePath.str() << ".");
}

} /* namespace point_cloud_io */
