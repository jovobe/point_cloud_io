/*
 * Bag.hpp
 *
 *  Created on: Oct 06, 2017
 *      Author: Johan M. von Behren
 *   Institute: Universität Osnabrück, Knowledge-Based Systems
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <unordered_set>

namespace point_cloud_io
{

class Bag
{
public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    explicit Bag(ros::NodeHandle& nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~Bag() = default;

private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * Read the given bag file
     */
    void readBagFile();

    /*!
     * Point cloud callback function
     * @param cloud point cloud message.
     */
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! Path to bag file to read.
    std::string bagFilePath_;

    //! Point cloud topics to subscribe to.
    std::unordered_set<std::string> pointCloudTopics_;

    //! Path to the point cloud folder.
    std::string folderPath_;

    //! Point cloud file prefix.
    std::string filePrefix_;

    //! Point cloud file ending.
    std::string fileEnding_;

    //! Data type for messages to export.
    std::string messageDataType_;

    //! Point cloud counter.
    unsigned int counter_ = 0;

    //! Maximum number of messages to export to .ply files.
    int maxMessagesCount_;

    //! Settings for generating file name.
    bool addCounterToPath_ = true;
    bool addFrameIdToPath_ = false;
    bool addStampSecToPath_ = false;
    bool addStampNSecToPath_ = false;
};

} /* namespace */
