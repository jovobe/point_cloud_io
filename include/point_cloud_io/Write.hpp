/*
 * Write.hpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "Writer.hpp"

namespace point_cloud_io
{

class Write: public Writer
{
public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    explicit Write(ros::NodeHandle& nodeHandle);

private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters() override;

    //! Point cloud subscriber.
    ros::Subscriber pointCloudSubscriber_;

    //! Point cloud topic to subscribe to.
    std::string pointCloudTopic_;
};

} /* namespace */
