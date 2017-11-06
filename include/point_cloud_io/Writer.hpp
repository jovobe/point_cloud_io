//
// Created by jvonbehren on 06.11.17.
//

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace point_cloud_io
{

class Writer
{
public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    explicit Writer(ros::NodeHandle& nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~Writer() = default;

protected:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    virtual bool readParameters();

    /*!
     * Point cloud callback function
     * @param cloud point cloud message.
     */
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! Path to the point cloud folder.
    std::string folderPath_;

    //! Point cloud file prefix.
    std::string filePrefix_;

    //! Point cloud file ending.
    std::string fileEnding_;

    //! Point cloud counter.
    unsigned int counter_ = 0;

    //! Settings for generating file name.
    bool addCounterToPath_ = true;
    bool addFrameIdToPath_ = false;
    bool addStampSecToPath_ = false;
    bool addStampNSecToPath_ = false;
};

} /* namespace point_cloud_io */
