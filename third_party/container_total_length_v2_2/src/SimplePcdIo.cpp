#include "SimplePcdIo.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

#include <Eigen/Dense>

#include <pcl/common/transforms.h>

namespace
{
std::string LowerExtension(const std::string& path)
{
    const std::string::size_type dot = path.find_last_of('.');
    if (dot == std::string::npos)
    {
        return "";
    }

    std::string ext = path.substr(dot + 1);
    for (std::string::size_type i = 0; i < ext.size(); ++i)
    {
        if (ext[i] >= 'A' && ext[i] <= 'Z')
        {
            ext[i] = static_cast<char>(ext[i] - 'A' + 'a');
        }
    }
    return ext;
}

bool IsIdentityViewpoint(const Eigen::Vector3f& origin, const Eigen::Quaternionf& orientation)
{
    return origin.norm() < 1e-6f &&
        std::abs(orientation.w() - 1.0f) < 1e-6f &&
        std::abs(orientation.x()) < 1e-6f &&
        std::abs(orientation.y()) < 1e-6f &&
        std::abs(orientation.z()) < 1e-6f;
}

bool ParseViewpoint(const std::string& line, Eigen::Vector3f* origin, Eigen::Quaternionf* orientation)
{
    std::istringstream stream(line);
    std::string tag;
    float tx = 0.0f;
    float ty = 0.0f;
    float tz = 0.0f;
    float qw = 1.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    stream >> tag >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
    if (!stream)
    {
        return false;
    }

    *origin = Eigen::Vector3f(tx, ty, tz);
    *orientation = Eigen::Quaternionf(qw, qx, qy, qz);
    orientation->normalize();
    return true;
}

struct PcdField
{
    std::string name;
    std::string type; /* F/U/I */
    int size;
    int count;
};

int FieldByteSize(const PcdField& field)
{
    return field.size * field.count;
}

bool HasXyzFields(const std::vector<PcdField>& fields, size_t* xIndex, size_t* yIndex, size_t* zIndex)
{
    *xIndex = *yIndex = *zIndex = static_cast<size_t>(-1);
    for (size_t i = 0; i < fields.size(); ++i)
    {
        if (fields[i].name == "x")
        {
            *xIndex = i;
        }
        else if (fields[i].name == "y")
        {
            *yIndex = i;
        }
        else if (fields[i].name == "z")
        {
            *zIndex = i;
        }
    }
    return *xIndex != static_cast<size_t>(-1) &&
        *yIndex != static_cast<size_t>(-1) &&
        *zIndex != static_cast<size_t>(-1);
}

float ReadFloatField(const char* base, const PcdField& field)
{
    if (field.type == "F" && field.size == 4)
    {
        float value = 0.0f;
        std::memcpy(&value, base, sizeof(float));
        return value;
    }
    if (field.type == "F" && field.size == 8)
    {
        double value = 0.0;
        std::memcpy(&value, base, sizeof(double));
        return static_cast<float>(value);
    }
    return 0.0f;
}

void ApplyViewpointIfNeeded(pcl::PointCloud<pcl::PointXYZ>& cloud,
                            const Eigen::Vector3f& origin,
                            const Eigen::Quaternionf& orientation)
{
    if (IsIdentityViewpoint(origin, orientation))
    {
        return;
    }

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    transform.block<3, 1>(0, 3) = origin;
    pcl::transformPointCloud(cloud, cloud, transform);
}
} // namespace

bool LoadXyzPcd(const std::string& path,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                std::string* error)
{
    if (!cloud)
    {
        if (error)
        {
            *error = "cloud output pointer is null";
        }
        return false;
    }
    cloud->clear();

    if (LowerExtension(path) != "pcd")
    {
        if (error)
        {
            *error = "unsupported point cloud format for SimplePcdIo, only .pcd is supported: " + path;
        }
        return false;
    }

    std::ifstream input(path.c_str(), std::ios::binary);
    if (!input)
    {
        if (error)
        {
            *error = "failed to open point cloud: " + path;
        }
        return false;
    }

    std::vector<PcdField> fields;
    std::string dataType = "ascii";
    size_t width = 0;
    size_t points = 0;
    Eigen::Vector3f origin = Eigen::Vector3f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    bool hasViewpoint = false;
    std::string line;

    while (std::getline(input, line))
    {
        if (!line.empty() && line[line.size() - 1] == '\r')
        {
            line.erase(line.size() - 1);
        }
        if (line.empty())
        {
            continue;
        }

        if (line.find("FIELDS") == 0)
        {
            fields.clear();
            std::istringstream stream(line);
            std::string tag;
            stream >> tag;
            std::string name;
            while (stream >> name)
            {
                PcdField field;
                field.name = name;
                field.type = "F";
                field.size = 4;
                field.count = 1;
                fields.push_back(field);
            }
        }
        else if (line.find("SIZE") == 0)
        {
            std::istringstream stream(line);
            std::string tag;
            stream >> tag;
            for (size_t i = 0; i < fields.size(); ++i)
            {
                stream >> fields[i].size;
            }
        }
        else if (line.find("TYPE") == 0)
        {
            std::istringstream stream(line);
            std::string tag;
            stream >> tag;
            for (size_t i = 0; i < fields.size(); ++i)
            {
                stream >> fields[i].type;
            }
        }
        else if (line.find("COUNT") == 0)
        {
            std::istringstream stream(line);
            std::string tag;
            stream >> tag;
            for (size_t i = 0; i < fields.size(); ++i)
            {
                stream >> fields[i].count;
            }
        }
        else if (line.find("WIDTH") == 0)
        {
            std::istringstream stream(line);
            std::string tag;
            stream >> tag >> width;
        }
        else if (line.find("POINTS") == 0)
        {
            std::istringstream stream(line);
            std::string tag;
            stream >> tag >> points;
        }
        else if (line.find("VIEWPOINT") == 0)
        {
            hasViewpoint = ParseViewpoint(line, &origin, &orientation);
        }
        else if (line.find("DATA") == 0)
        {
            std::istringstream stream(line);
            std::string tag;
            stream >> tag >> dataType;
            break;
        }
    }

    size_t xIndex = 0;
    size_t yIndex = 0;
    size_t zIndex = 0;
    if (fields.empty() || !HasXyzFields(fields, &xIndex, &yIndex, &zIndex))
    {
        if (error)
        {
            *error = "PCD missing x/y/z fields: " + path;
        }
        return false;
    }

    if (points == 0)
    {
        points = width;
    }
    if (points == 0)
    {
        if (error)
        {
            *error = "PCD point count is zero: " + path;
        }
        return false;
    }

    cloud->reserve(points);

    for (size_t i = 0; i < dataType.size(); ++i)
    {
        if (dataType[i] >= 'A' && dataType[i] <= 'Z')
        {
            dataType[i] = static_cast<char>(dataType[i] - 'A' + 'a');
        }
    }

    if (dataType == "ascii")
    {
        for (size_t i = 0; i < points; ++i)
        {
            if (!std::getline(input, line))
            {
                if (error)
                {
                    *error = "unexpected EOF while reading ASCII PCD: " + path;
                }
                return false;
            }
            if (!line.empty() && line[line.size() - 1] == '\r')
            {
                line.erase(line.size() - 1);
            }

            std::istringstream stream(line);
            std::vector<double> values(fields.size(), 0.0);
            for (size_t f = 0; f < fields.size(); ++f)
            {
                stream >> values[f];
            }
            if (!stream && stream.eof())
            {
                // tolerate trailing whitespace
            }

            pcl::PointXYZ pt;
            pt.x = static_cast<float>(values[xIndex]);
            pt.y = static_cast<float>(values[yIndex]);
            pt.z = static_cast<float>(values[zIndex]);
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
            {
                cloud->push_back(pt);
            }
        }
    }
    else if (dataType == "binary")
    {
        int pointStep = 0;
        std::vector<int> offsets(fields.size(), 0);
        for (size_t i = 0; i < fields.size(); ++i)
        {
            offsets[i] = pointStep;
            pointStep += FieldByteSize(fields[i]);
        }

        std::vector<char> buffer(static_cast<size_t>(pointStep));
        for (size_t i = 0; i < points; ++i)
        {
            input.read(&buffer[0], pointStep);
            if (!input)
            {
                if (error)
                {
                    *error = "unexpected EOF while reading binary PCD: " + path;
                }
                return false;
            }

            pcl::PointXYZ pt;
            pt.x = ReadFloatField(&buffer[0] + offsets[xIndex], fields[xIndex]);
            pt.y = ReadFloatField(&buffer[0] + offsets[yIndex], fields[yIndex]);
            pt.z = ReadFloatField(&buffer[0] + offsets[zIndex], fields[zIndex]);
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
            {
                cloud->push_back(pt);
            }
        }
    }
    else
    {
        if (error)
        {
            *error = "unsupported PCD DATA type (only ascii/binary): " + path;
        }
        return false;
    }

    if (cloud->empty())
    {
        if (error)
        {
            *error = "failed to load point cloud or cloud is empty: " + path;
        }
        return false;
    }

    if (hasViewpoint)
    {
        ApplyViewpointIfNeeded(*cloud, origin, orientation);
    }

    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return true;
}
