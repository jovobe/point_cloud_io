//
// Created by jvonbehren on 14.10.17.
//

#pragma once

#include <string>
#include <pcl/point_cloud.h>

using namespace std;
using namespace pcl;

namespace point_cloud_io
{

/*!
 * Determines the data contained in the given .ply file.
 * @param filePath the path to the .ply file.
 * @return a bitmask representing the data contained in the given .ply file:
 *
 * Bit |  Meaning
 * ------------
 * 0      XYZ found
 * 1      RGB found
 * 2      Normals found
 * 3
 * 4
 * 5
 * 6
 * 7
 */
uint8_t determinePLYData(const std::string& filePath);

} /* namespace point_cloud_io */
