#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <string>
#include <vector>

namespace bevel
{
using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

struct BevelMeasurementResult
{
    bool ok = false;
    int bevelType = -1;
    double angleDeg = 0.0;
    double length = 0.0;
    double icpFitness = 0.0;
    int qualityCode = 10000;
    Eigen::Matrix4f scanToTemplate = Eigen::Matrix4f::Identity();
    std::string message;
};

struct BevelConfig
{
    bool uniformDownsample = true;
    double uniformLeafSize = 1.0;
    bool saveDownsampledCloud = true;
    std::string downsampledCloudPath = "data/debug/downsampled_cloud.pcd";

    bool poseCorrection = true;
    Eigen::Vector3f poseTranslation = Eigen::Vector3f::Zero();
    Eigen::Vector3f poseRotationDeg = Eigen::Vector3f::Zero();

    bool cropBox = true;
    Eigen::Vector4f cropMin = Eigen::Vector4f(-800.0f, -800.0f, -200.0f, 1.0f);
    Eigen::Vector4f cropMax = Eigen::Vector4f(800.0f, 800.0f, 200.0f, 1.0f);

    bool outlierRemoval = true;
    int sorMeanK = 30;
    double sorStddevMulThresh = 1.0;  // 标准差倍数，默认通常�?.0左右，来微调去噪的强烈程�?
    int imageWidth = 512;
    int imageHeight = 512;
    Eigen::Vector3f planeCenter = Eigen::Vector3f::Zero();
    Eigen::Vector3f planeNormal = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    double planeThickness = 2.0;
    double projectionScale = 0.0;
    bool saveProjectionImage = false;
    std::string projectionImagePath = "data/debug/projection.png";
    bool saveNormalizedProjectionImage = true;
    std::string normalizedProjectionImagePath = "data/debug/projection_pca.png";

    std::string svmModelPath = "D:/1 自研/15 兰铀算法/测量算法/坡口测量/坡口类型数据训练/坡口类型识别/opencv_test/svm_weight.xml";

    int icpMaxIterations = 80;
    double icpMaxCorrespondenceDistance = 5.0;
    double icpTransformationEpsilon = 1e-8;
    double icpEuclideanFitnessEpsilon = 1e-6;
    bool icpTrimEnable = true;
    double icpTrimOverlapRatio = 0.7;
    int icpTrimMinCorrespondences = 50;
    bool saveAlignedCloud = true;
    std::string alignedCloudPath = "data/debug/aligned_scan.pcd";

    std::string templatePathPattern = "data/templates/type_{type}.pcd";
    std::string measurementMethod = "plane_fit";
    std::string planeFitFeaturePathPattern = "data/templates/type_{type}_features_plane_fit.txt";
    std::string directFeaturePathPattern = "data/templates/type_{type}_features_direct.txt";

    double standardAngleMinDeg = 0.0;
    double standardAngleMaxDeg = 180.0;
    double standardLengthMin = 0.0;
    double standardLengthMax = 1000000.0;
};

struct TemplateFeature
{
    std::string name;
    Eigen::Vector3f point;
};

struct BevelSolveOptions
{
    int forcedBevelType = -1;
    bool overrideStandard = false;
    double standardAngleMinDeg = 0.0;
    double standardAngleMaxDeg = 0.0;
    double standardLengthMin = 0.0;
    double standardLengthMax = 0.0;
};

BevelConfig loadConfig(const std::string& configPath);

BevelMeasurementResult solveBevelFromRawCloud(const CloudT::ConstPtr& rawCloud,
                                              const std::string& configPath = "config.txt");

BevelMeasurementResult solveBevelFromRawCloud(const CloudT::ConstPtr& rawCloud,
                                              const std::string& configPath,
                                              const std::string& templateDir);

BevelMeasurementResult solveBevelFromRawCloud(const CloudT::ConstPtr& rawCloud,
                                              const std::string& configPath,
                                              const std::string& templateDir,
                                              const BevelSolveOptions& options);

bool loadTextPointCloud(const std::string& path, CloudT::Ptr cloud);

} // namespace bevel
