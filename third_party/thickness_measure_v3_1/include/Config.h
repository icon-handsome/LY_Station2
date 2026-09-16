#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include "AppConfig.h"

struct Point3d { double x; double y; double z; };

struct IcpConfig {
    int maxIterations;
    double maxCorrespondenceDistance;
    double transformationEpsilon;
    double euclideanFitnessEpsilon;
    int normalK;
    double maxCurvature;
    double minPlanarity;
    int normalSmoothingK;
    double normalSmoothingAngleDeg;
    int minReliablePointCount;
};

struct PreprocessConfig {
    bool enableOutlierRemoval;
    int meanK;
    double stddevMulThresh;
    bool enableVoxelDownsample;
    double leafSize;
};

struct ThicknessSectionConfig {
    Point3d planePoint;
    Point3d planeNormal;
    double thicknessMm;
};

struct DynamicSectionsConfig {
    bool enabled;
    Point3d axisPoint;
    Point3d axisDirection;
    Point3d referencePoint;
    Point3d centerPoint1;
    Point3d centerPoint2;
    double offset1Mm;
    double offset2Mm;
    double halfWidthMm;
    double thicknessMm;
};

struct ThicknessConfig {
    // 是否保存调试点云（PLY）；默认关闭。
    bool saveDebugData = false;
    std::string outerTemplateCloudPath;
    std::string innerScanInputPath;
    std::string outerScanInputPath;
    PreprocessConfig preprocess;
    IcpConfig innerOuterIcp;
    IcpConfig outerTemplateIcp;
    std::vector<ThicknessSectionConfig> sections;
    double toeOutwardOffsetMm;
    double sectionHalfLengthMm;
    double minSectionThicknessMm;
    weld::OnnxToeLocatorConfig onnxToeLocator;
    std::vector<Point3d> fallbackFeaturePoints;
    std::string resultPath;
    Point3d templateAxisPoint;
    Point3d templateAxisDirection;
    DynamicSectionsConfig dynamicSections;
    // Optional second measurement group. It uses the same ICP/preprocess settings.
    std::string outerTemplateCloudPath2;
    std::string innerScanInputPath2;
    std::string outerScanInputPath2;
    IcpConfig innerOuterIcp2;
    bool usePointToPlaneInnerOuter2 = false;
    std::vector<ThicknessSectionConfig> sections2;
    std::vector<Point3d> fallbackFeaturePoints2;
    double toeOutwardOffsetMm2;
    double sectionHalfLengthMm2;
    double minSectionThicknessMm2;
    DynamicSectionsConfig dynamicSections2;
    bool hasSecondGroup = false;
};

bool LoadConfig(const std::string& path, ThicknessConfig* config, std::string* error);
Eigen::Vector3d ToEigen(const Point3d& point);
