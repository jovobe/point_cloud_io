#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <unordered_set>
#include <ros/ros.h>
#include "point_cloud_io/Utility.hpp"

namespace point_cloud_io
{

uint8_t determinePLYData(const std::string& filePath)
{
    PCLPointCloud2 tempCloud;
    PLYReader p;
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
        return 255;
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

} /* namespace point_cloud_io */
