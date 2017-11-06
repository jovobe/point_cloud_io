//
// Created by jvonbehren on 14.10.17.
//

#pragma once

#include <string>
#include <unordered_set>

#include <boost/optional.hpp>
#include <pcl/point_cloud.h>

using namespace std;
using namespace pcl;

namespace point_cloud_io
{

/*!
 * Determines the data of a .ply file. See: determinePLYChannelsByFields(unordered_set<string>&)
 */
uint8_t determinePLYData(const std::string& filePath);

/*!
 * Extracts the data fields from a given .ply file.
 * @param filePath the path to the .ply file.
 * @return a set containing the names of the data fields in the given .ply file.
 */
boost::optional<unordered_set<string>> getFieldsFromPly(const std::string& filePath);

/*!
 * Determines the data of a .ply file or a PointCloud2 by it's fields.
 * @param fields the names of the data fields in the the .ply file or the PointCloud2.
 * @return a bitmask representing the data contained in the given fields.
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
uint8_t determinePLYChannelsByFields(unordered_set<string>& fields);

} /* namespace point_cloud_io */
