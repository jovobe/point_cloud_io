/*
 * Write.cpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "point_cloud_io/Write.hpp"

using namespace std;
using namespace ros;

namespace point_cloud_io
{

Write::Write(ros::NodeHandle& nodeHandle)
    : Writer(nodeHandle)
{
    if (!readParameters()) ros::requestShutdown();
    pointCloudSubscriber_ = nodeHandle_.subscribe(
        pointCloudTopic_,
        1,
        &Write::pointCloudCallback,
        dynamic_cast<Writer*>(this)
    );
    ROS_INFO_STREAM("Subscribed to topic \"" << pointCloudTopic_ << "\".");
}

bool Write::readParameters()
{
    bool allParametersRead = Writer::readParameters();
    if (!nodeHandle_.getParam("topic", pointCloudTopic_)) allParametersRead = false;

    if (!allParametersRead)
    {
        ROS_WARN("Could not read all parameters. Typical command-line usage:\n"
                     "rosrun point_cloud_io write"
                     " _topic:=/my_topic"
                     " _folder_path:=/home/user/my_point_clouds"
                     " (optional: _file_prefix:=my_prefix"
                     " _file_ending:=my_ending"
                     " _add_counter_to_path:=true/false"
                     " _add_frame_id_to_path:=true/false"
                     " _add_stamp_sec_to_path:=true/false"
                     " _add_stamp_nsec_to_path:=true/false)");
        return false;
    }

    return true;
}

} /* namespace */
