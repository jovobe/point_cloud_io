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

#include "Writer.hpp"

namespace point_cloud_io
{

class Bag: public Writer
{
public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    explicit Bag(ros::NodeHandle& nodeHandle);

private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters() override;

    /*!
     * Read the given bag file
     */
    void readBagFile();

    //! Path to bag file to read.
    std::string bagFilePath_;

    //! Point cloud topics to subscribe to.
    std::unordered_set<std::string> pointCloudTopics_;

    //! Data type for messages to export.
    std::string messageDataType_;

    //! Maximum number of messages to export to .ply files.
    int maxMessagesCount_;
};

} /* namespace */
