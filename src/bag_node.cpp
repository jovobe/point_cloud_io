/*
 * bag_node.cpp
 *
 *  Created on: Oct 06, 2017
 *      Author: Johan M. von Behren
 *   Institute: Universität Osnabrück, Knowledge-Based Systems
 */

#include <ros/ros.h>
#include "point_cloud_io/Bag.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag");
    ros::NodeHandle nodeHandle("~");

    point_cloud_io::Bag bag(nodeHandle);

    ros::spin();
    return 0;
}
