#include "PointCloudIO.h"

#if defined(CONTAINER_TOTAL_LENGTH_NO_PCL_IO)
#include "SimplePcdIo.h"
#else
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#endif

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <vector>

namespace {

std::string Trim(const std::string& value)
{
    size_t first = 0;
    while (first < value.size() && std::isspace(static_cast<unsigned char>(value[first]))) ++first;
    size_t last = value.size();
    while (last > first && std::isspace(static_cast<unsigned char>(value[last - 1]))) --last;
    return value.substr(first, last - first);
}

std::string StripComment(const std::string& value)
{
    const size_t position = value.find("//");
    return position == std::string::npos ? value : value.substr(0, position);
}

std::string Lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](char c) {
        return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    });
    return value;
}

bool EndsWith(const std::string& value, const std::string& suffix)
{
    return value.size() >= suffix.size() &&
        Lower(value.substr(value.size() - suffix.size())) == Lower(suffix);
}

bool ParseVector3(std::string value, Eigen::Vector3f* output)
{
    if (!output) return false;
    std::replace(value.begin(), value.end(), ',', ' ');
    std::istringstream stream(value);
    return static_cast<bool>(stream >> (*output).x() >> (*output).y() >> (*output).z());
}

bool ParseBool(const std::string& value, bool* output)
{
    if (!output) return false;
    const std::string lowered = Lower(Trim(value));
    if (lowered == "1" || lowered == "true" || lowered == "yes" || lowered == "on") {
        *output = true;
        return true;
    }
    if (lowered == "0" || lowered == "false" || lowered == "no" || lowered == "off") {
        *output = false;
        return true;
    }
    return false;
}

bool LoadXYZText(const std::string& path, CloudPtr cloud, std::string* error)
{
    std::ifstream input(path.c_str());
    if (!input) {
        if (error) *error = "Cannot open cloud file: " + path;
        return false;
    }
    std::string line;
    while (std::getline(input, line)) {
        line = Trim(StripComment(line));
        if (line.empty()) continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream stream(line);
        PointT point;
        if (stream >> point.x >> point.y >> point.z) cloud->push_back(point);
    }
    cloud->width = static_cast<unsigned int>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return true;
}

}  // namespace

Config::Config()
    : cropInputCloud(false),
      cropMinPoint(Eigen::Vector3f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max())),
      cropMaxPoint(Eigen::Vector3f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max())),
      templateAxisDirection(Eigen::Vector3f::UnitZ()),
      voxelSize(3.0f), outlierK(12), outlierStd(2.0),
      icpMaxIterations(80), icpMaxCorrespondenceDistance(30.0f),
      icpTransformationEpsilon(1e-8), icpEuclideanFitnessEpsilon(1e-6),
      cylinderFitIterations(6), cylinderInlierBand(0.0f), updateCylinderAxis(false),
      normalK(20), endNormalMinAbsDot(0.90),
      
      endpointDetectionMethod("outsideScan"),
      axialBinWidth(2.0f), outsideScanPeakSearchWidth(20.0f),
      minPointsPerBin(20), minConsecutiveInsideBins(3), outsideCheckBins(2),
      refineHalfWidth(6.0f), refineEdgePercentile(0.001), endpointMaxRadius(0.0f)
{
}

bool LoadIniConfig(const std::string& path, Config* config, std::string* error)
{
    if (!config) return false;
    std::ifstream input(path.c_str());
    if (!input) {
        if (error) *error = "Cannot open config file: " + path;
        return false;
    }

    std::string section;
    std::string line;
    int lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        // UTF-8 BOM is legal at the beginning of a text file but not part of an INI token.
        if (lineNumber == 1 && line.size() >= 3 &&
            static_cast<unsigned char>(line[0]) == 0xEF &&
            static_cast<unsigned char>(line[1]) == 0xBB &&
            static_cast<unsigned char>(line[2]) == 0xBF) {
            line.erase(0, 3);
        }
        line = Trim(StripComment(line));
        if (line.empty()) continue;
        if (line[0] == '[' && line[line.size() - 1] == ']') {
            section = Lower(Trim(line.substr(1, line.size() - 2)));
            continue;
        }
        const size_t equals = line.find('=');
        if (equals == std::string::npos) {
            if (error) {
                std::ostringstream stream;
                stream << "Invalid ini line " << lineNumber << ": missing '='";
                *error = stream.str();
            }
            return false;
        }
        const std::string key = Lower(Trim(line.substr(0, equals)));
        const std::string value = Trim(line.substr(equals + 1));

        if (section == "input") {
            if (key == "inputcloud") config->inputCloudPath = value;
            else if (key == "cropinputcloud" && !ParseBool(value, &config->cropInputCloud)) return false;
            else if (key == "cropmin" && !ParseVector3(value, &config->cropMinPoint)) return false;
            else if (key == "cropmax" && !ParseVector3(value, &config->cropMaxPoint)) return false;
        } else if (section == "template") {
            if (key == "templatecloud") config->templateCloudPath = value;
            else if (key == "axisdirection" && !ParseVector3(value, &config->templateAxisDirection)) return false;
        } else if (section == "preprocess") {
            if (key == "voxelsize") config->voxelSize = static_cast<float>(std::atof(value.c_str()));
            else if (key == "outlierk") config->outlierK = std::atoi(value.c_str());
            else if (key == "outlierstd") config->outlierStd = std::atof(value.c_str());
        } else if (section == "icp") {
            if (key == "maxiterations") config->icpMaxIterations = std::atoi(value.c_str());
            else if (key == "maxcorrespondencedistance") config->icpMaxCorrespondenceDistance = static_cast<float>(std::atof(value.c_str()));
            else if (key == "transformationepsilon") config->icpTransformationEpsilon = std::atof(value.c_str());
            else if (key == "euclideanfitnessepsilon") config->icpEuclideanFitnessEpsilon = std::atof(value.c_str());
        } else if (section == "cylinderfit") {
            if (key == "iterations") config->cylinderFitIterations = std::atoi(value.c_str());
            else if (key == "inlierband") config->cylinderInlierBand = static_cast<float>(std::atof(value.c_str()));
            else if (key == "updateaxis" && !ParseBool(value, &config->updateCylinderAxis)) return false;
        } else if (section == "lengthmeasurement") {
            if (key == "endpointdetectionmethod") config->endpointDetectionMethod = value;
            else if (key == "normalk") config->normalK = std::atoi(value.c_str());
            else if (key == "endnormalminabsdot") config->endNormalMinAbsDot = std::atof(value.c_str());
            else if (key == "axialbinwidth") config->axialBinWidth = static_cast<float>(std::atof(value.c_str()));
            else if (key == "outsidescanpeaksearchwidth") config->outsideScanPeakSearchWidth = static_cast<float>(std::atof(value.c_str()));
            else if (key == "minpointsperbin") config->minPointsPerBin = std::atoi(value.c_str());
            else if (key == "minconsecutiveinsidebins") config->minConsecutiveInsideBins = std::atoi(value.c_str());
            else if (key == "outsidecheckbins") config->outsideCheckBins = std::atoi(value.c_str());
            else if (key == "refinehalfwidth") config->refineHalfWidth = static_cast<float>(std::atof(value.c_str()));
            else if (key == "refineedgepercentile") config->refineEdgePercentile = std::atof(value.c_str());
            else if (key == "endpointmaxradius") config->endpointMaxRadius = static_cast<float>(std::atof(value.c_str()));
        } else {
            if (error) {
                std::ostringstream stream;
                stream << "Unknown ini section '" << section << "' at line " << lineNumber;
                *error = stream.str();
            }
            return false;
        }
    }

    if (config->templateAxisDirection.norm() <= 1e-6f) {
        if (error) *error = "axisDirection must be non-zero.";
        return false;
    }
    config->templateAxisDirection.normalize();
    config->endpointDetectionMethod = Lower(config->endpointDetectionMethod);
    if (config->endpointDetectionMethod != "outsidescan") {
        if (error) *error = "endpointDetectionMethod must be outsideScan.";
        return false;
    }
    if (config->cropInputCloud &&
        (config->cropMinPoint.x() > config->cropMaxPoint.x() ||
         config->cropMinPoint.y() > config->cropMaxPoint.y() ||
         config->cropMinPoint.z() > config->cropMaxPoint.z())) {
        if (error) *error = "cropMin must be less than or equal to cropMax.";
        return false;
    }
    if (config->inputCloudPath.empty() || config->templateCloudPath.empty()) {
        if (error) *error = "inputCloud and templateCloud are required.";
        return false;
    }
    if (config->voxelSize <= 0.0f || config->axialBinWidth <= 0.0f ||
        config->outsideScanPeakSearchWidth < 0.0f || config->refineHalfWidth <= 0.0f) {
        if (error) *error = "voxel, bin and refine widths must be positive, and outside scan peak width cannot be negative.";
        return false;
    }
    return true;
}

CloudPtr LoadCloud(const std::string& path, std::string* error)
{
    CloudPtr cloud(new CloudT);
#if defined(CONTAINER_TOTAL_LENGTH_NO_PCL_IO)
    if (EndsWith(path, ".pcd")) {
        if (!LoadXyzPcd(path, cloud, error) || cloud->empty()) {
            return CloudPtr(new CloudT);
        }
        return cloud;
    }
    return LoadXYZText(path, cloud, error) ? cloud : CloudPtr(new CloudT);
#else
    int status = -1;
    if (EndsWith(path, ".pcd")) status = pcl::io::loadPCDFile<PointT>(path, *cloud);
    else if (EndsWith(path, ".ply")) status = pcl::io::loadPLYFile<PointT>(path, *cloud);
    else return LoadXYZText(path, cloud, error) ? cloud : CloudPtr(new CloudT);
    if (status != 0) {
        if (error) *error = "Failed to read point cloud: " + path;
        return CloudPtr(new CloudT);
    }
    return cloud;
#endif
}
