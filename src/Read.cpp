/*
 * Read.cpp
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "point_cloud_io/Read.hpp"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS

#include <pcl/io/vtk_lib_io.h>

#include <unordered_set>

using namespace std;
using namespace ros;
using namespace pcl;
using namespace pcl::io;
using namespace pcl_conversions;

namespace point_cloud_io
{

Read::Read(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      pointCloudMessage_(new sensor_msgs::PointCloud2)
{
    if (!readParameters()) ros::requestShutdown();
    pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(pointCloudTopic_, 1, true);
    initialize();
}

Read::~Read()
{

}

bool Read::readParameters()
{
    bool allParametersRead = true;
    if (!nodeHandle_.getParam("file_path", filePath_)) allParametersRead = false;
    if (!nodeHandle_.getParam("topic", pointCloudTopic_)) allParametersRead = false;
    if (!nodeHandle_.getParam("frame", pointCloudFrameId_)) allParametersRead = false;

    double updateRate;
    nodeHandle_.param("rate", updateRate, 0.0);
    if (updateRate == 0.0)
    {
        isContinousPublishing_ = false;
    }
    else
    {
        isContinousPublishing_ = true;
        updateDuration_.fromSec(1.0 / updateRate);
    }

    if (!allParametersRead)
    {
        ROS_WARN("Could not read all parameters. Typical command-line usage:\n"
                     "rosrun point_cloud_io read"
                     " _file_path:=/home/user/my_point_cloud.ply"
                     " _topic:=/my_topic"
                     " _frame:=sensor_frame"
                     " (optional: _rate:=publishing_rate)");
        return false;
    }

    return true;
}

void Read::initialize()
{
    if (!readFile(filePath_, pointCloudFrameId_)) ros::requestShutdown();

    if (isContinousPublishing_)
    {
        timer_ = nodeHandle_.createTimer(updateDuration_, &Read::timerCallback, this);
    }
    else
    {
        if (!publish()) ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");

        // Just wait for other subscribers, latching will publish the message for them.
        ros::spin();
    }
}

bool Read::readFile(const std::string& filePath, const std::string& pointCloudFrameId)
{
    if (filePath.find(".ply") != std::string::npos)
    {
        // Load .ply file.
        auto dataIndicator = determinePLYData(filePath);
        if (1 == dataIndicator)
        {
            // XYZ found (Bitmask: 0000 0001)
            PointCloud<PointXYZ> pointCloudXYZ;
            if (loadPLYFile(filePath, pointCloudXYZ) != 0) return false;

            // Define PointCloud2 message.
            toROSMsg(pointCloudXYZ, *pointCloudMessage_);
        }
        else if (3 == dataIndicator)
        {
            // XYZ and RGB found (Bitmask: 0000 0011)
            PointCloud<PointXYZRGB> pointCloudXYZRGB;
            if (loadPLYFile(filePath, pointCloudXYZRGB) != 0) return false;

            // Define PointCloud2 message.
            toROSMsg(pointCloudXYZRGB, *pointCloudMessage_);
        }
        else if (5 == dataIndicator)
        {
            // XYZ and normals found (Bitmask: 0000 0101)
            PointCloud<PointXYZINormal> pointCloudXYZN;
            if (loadPLYFile(filePath, pointCloudXYZN) != 0) return false;

            // Define PointCloud2 message.
            toROSMsg(pointCloudXYZN, *pointCloudMessage_);
        }
        else if (7 == dataIndicator)
        {
            // XYZ, normals and RGB found (Bitmask: 0000 0111)
            PointCloud<PointXYZRGBNormal> pointCloudXYZRGBN;
            if (loadPLYFile(filePath, pointCloudXYZRGBN) != 0) return false;

            // Define PointCloud2 message.
            toROSMsg(pointCloudXYZRGBN, *pointCloudMessage_);
        }
        else
        {
            ROS_ERROR_STREAM("Fields defined in given .ply file are not parseable!");
            return false;
        }
    }
    else if (filePath.find(".vtk") != std::string::npos)
    {
        // Load .vtk file.
        PolygonMesh polygonMesh;
        loadPolygonFileVTK(filePath, polygonMesh);

        // Define PointCloud2 message.
        moveFromPCL(polygonMesh.cloud, *pointCloudMessage_);
    }
    else
    {
        ROS_ERROR_STREAM("Data format not supported.");
        return false;
    }

    pointCloudMessage_->header.frame_id = pointCloudFrameId;

    ROS_INFO_STREAM("Loaded point cloud with " << pointCloudMessage_->height * pointCloudMessage_->width << " points.");
    return true;
}

void Read::timerCallback(const ros::TimerEvent& timerEvent)
{
    if (!publish()) ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");
}

bool Read::publish()
{
    pointCloudMessage_->header.stamp = Time::now();
    if (pointCloudPublisher_.getNumSubscribers() > 0u)
    {
        pointCloudPublisher_.publish(pointCloudMessage_);
        ROS_INFO_STREAM("Point cloud published to topic \"" << pointCloudTopic_ << "\".");
    }
    return true;
}

uint8_t Read::determinePLYData(const std::string& filePath)
{
    PCLPointCloud2 tempCloud;
    pcl::PLYReader p;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int ply_version;
    int data_type;
    unsigned int data_idx;

    // Read header information from .ply file.
    auto a = p.readHeader(filePath, tempCloud, origin, orientation, ply_version, data_type, data_idx);
    if (a < 0)
    {
        ROS_ERROR_STREAM("Error reading header of " << filePath << "!");
        return -1;
    }

    // Check the found headers
    unordered_set<string> fields;
    for (auto field: tempCloud.fields)
    {
        fields.insert(field.name);
    }

    // Create bitmask (0 = LSB, 7 = MSB) to indicate which data was found
    // The bits mean the following, if they are set to 1:
    // Bit |  Meaning
    // ------------
    // 0      XYZ found
    // 1      RGB found
    // 2      Normals found
    // 3
    // 4
    // 5
    // 6
    // 7
    // ------------
    uint8_t bitmask = 0;
    if (fields.find("x") != fields.end() && fields.find("y") != fields.end() && fields.find("z") != fields.end())
    {
        bitmask |= 1 << 0;
    }
    if (fields.find("red") != fields.end() && fields.find("green") != fields.end() &&
        fields.find("blue") != fields.end())
    {
        bitmask |= 1 << 1;
    }
    if (fields.find("nx") != fields.end() && fields.find("ny") != fields.end() && fields.find("nz") != fields.end())
    {
        bitmask |= 1 << 2;
    }

    return bitmask;
}

} /* namespace */
