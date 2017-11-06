/*
 * Write.cpp
 *
 *  Created on: Oct 06, 2017
 *      Author: Johan M. von Behren
 *   Institute: Universität Osnabrück, Knowledge-Based Systems
 */

#include "point_cloud_io/Bag.hpp"

#include <boost/algorithm/string/join.hpp>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

//ROS
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace boost::algorithm;
using namespace rosbag;
using namespace std;
using namespace ros;
using namespace pcl;
using namespace pcl::io;

namespace point_cloud_io
{

Bag::Bag(ros::NodeHandle& nodeHandle)
    : Writer(nodeHandle),
      maxMessagesCount_(10),
      messageDataType_("sensor_msgs/PointCloud2")
{
    if (!readParameters()) ros::requestShutdown();
    ROS_INFO_STREAM("Read topics \"" << join(pointCloudTopics_, ", ") << "\" from bag \"" << bagFilePath_ << "\".");
    readBagFile();
    ROS_INFO_STREAM(
        "Read all messages from topics \"" << join(pointCloudTopics_, ", ") << "\" from bag \"" << bagFilePath_
                                           << "\".");
    ros::requestShutdown();
}

bool Bag::readParameters()
{
    bool allParametersRead = Writer::readParameters();
    string topics_string;

    if (!nodeHandle_.getParam("bag_path", bagFilePath_)) allParametersRead = false;
    if (!nodeHandle_.getParam("topics", topics_string)) allParametersRead = false;
    nodeHandle_.getParam("max_messages", maxMessagesCount_);

    // Split topics, trim their names and copy them into a set.
    vector<string> topics_vector;
    split(topics_vector, topics_string, is_any_of(","), token_compress_on);
    pointCloudTopics_.reserve(topics_vector.size());
    for (auto& elem: topics_vector)
    {
        trim(elem);
        pointCloudTopics_.insert(elem);
    }

    if (!allParametersRead)
    {
        ROS_WARN("Could not read all parameters. Typical command-line usage:\n"
                     "rosrun point_cloud_io bag"
                     " _topics:=/my_first_topic,/my_second_topic"
                     " _bag_path:=/home/user/bags/my_bag.bag"
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

void Bag::readBagFile()
{
    rosbag::Bag bag;
    bag.open(bagFilePath_, bagmode::Read);

    // Create query to match only PointCloud2 and the selected topics.
    View view(bag, [this](ConnectionInfo const *info)
    {
        return messageDataType_ == info->datatype
               && pointCloudTopics_.find(info->topic) != pointCloudTopics_.end();
    });

    // Check message count.
    ROS_INFO_STREAM("Found " << view.size() << " messages in topics \"" << join(pointCloudTopics_, ", ") << "\"!");
    if (view.size() > maxMessagesCount_)
    {
        ROS_ERROR_STREAM("Found more \"" << messageDataType_ << "\" messages than allowed! In order to prevent "
            "the creation of too many .ply files the export process will be canceled.");
        ros::requestShutdown();
        return;
    }

    // Export found messages as .ply files.
    for (const auto& messageRef: view)
    {
        ROS_INFO_STREAM("Message type: " << messageRef.getDataType());
        auto message = messageRef.instantiate<sensor_msgs::PointCloud2>();
        pointCloudCallback(message);
    }

    bag.close();
}

} /* namespace */
