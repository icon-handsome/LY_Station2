#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include <string>
#include <vector>

namespace weld
{

struct UndercutConfig
{
    double sectionIntervalMm;
    double depthThresholdMm;
    int medianFilterWindow;
    double toeSearchWidthMm;
    double baseFitStartFromToeMm;
    double baseFitEndFromToeMm;
    double minGrooveWidthMm;
    int minGroovePoints;

    UndercutConfig()
        : sectionIntervalMm(5.0),
          depthThresholdMm(0.5),
          medianFilterWindow(3),
          toeSearchWidthMm(5.0),
          baseFitStartFromToeMm(5.0),
          baseFitEndFromToeMm(40.0),
          minGrooveWidthMm(0.5),
          minGroovePoints(3)
    {
    }
};

struct OnnxToeLocatorConfig
{
    bool enabled;
    std::string modelPath;
    std::string inputFeatureName;
    std::string outputName;
    int targetPointCount;
    int medianFilterWindow;
    int detrendFilterWindow;
    double fixedPhysicalScaleMm;
    double slopeRadiusMm;
    double fineCurvatureRadiusMm;
    double macroCurvatureRadiusMm;
    int continuityWindow;
    int seamClassId;
    int numClasses;
    int minSeamPoints;

    OnnxToeLocatorConfig()
        : enabled(true),
          modelPath("..\\Data\\pointnet_weld_seam_V8.2_good.onnx"),
          inputFeatureName("input_15d_features"),
          outputName("output_segmentation"),
          targetPointCount(512),
          medianFilterWindow(7),
          detrendFilterWindow(51),
          fixedPhysicalScaleMm(10.0),
          slopeRadiusMm(2.0),
          fineCurvatureRadiusMm(1.5),
          macroCurvatureRadiusMm(5.0),
          continuityWindow(9),
          seamClassId(1),
          numClasses(3),
          minSeamPoints(3)
    {
    }
};

struct Vector3Config
{
    double x;
    double y;
    double z;

    Vector3Config() : x(0.0), y(0.0), z(0.0) {}
};

struct SectionConfig
{
    double distMm;
    double widthMm;
    double lengthMm;
    double heightMm;
    Vector3Config startPoint;
    Vector3Config endPoint;

    SectionConfig()
        : distMm(1.0), widthMm(2.0), lengthMm(100.0), heightMm(100.0), startPoint(), endPoint() {}
};

struct FrameConfig
{
    std::string scanPath;
    std::string templatePath;
    Vector3Config cylinderAxis;
    Vector3Config cylinderCenter;
    SectionConfig section;
};

struct RoiConfig
{
    bool enabled;
    Vector3Config minPoint;
    Vector3Config maxPoint;
    RoiConfig() : enabled(false), minPoint(), maxPoint() {}
};
struct IcpConfig
{
    double voxelLeafSizeMm;
    double maxCorrespondenceDistanceMm;
    int maximumIterations;
    double transformationEpsilon;
    double euclideanFitnessEpsilon;
    double maxFitnessScore;
    int minCorrespondences;

    IcpConfig()
        : voxelLeafSizeMm(1.0), maxCorrespondenceDistanceMm(5.0), maximumIterations(50),
          transformationEpsilon(1e-8), euclideanFitnessEpsilon(1e-6), maxFitnessScore(100.0), minCorrespondences(10) {}
};

struct AppConfig
{
    int frameCount;
    bool isRingWeld;
    std::vector<FrameConfig> frames;
    RoiConfig scanRoi;
    IcpConfig icp;
    double maxFitErrorMm;
    double toeLocationErrorMm;
    UndercutConfig undercut;
    OnnxToeLocatorConfig onnxToeLocator;

    AppConfig()
        : frameCount(0),
          isRingWeld(false),
          frames(),
          scanRoi(),
          icp(),
          maxFitErrorMm(0.20),
          toeLocationErrorMm(0.50),
          undercut(),
          onnxToeLocator()
    {
    }
};

class AppConfigReader
{
public:
    bool Load(const std::string& filePath, AppConfig* config, std::string* message) const;
    bool Load(const std::wstring& filePath, AppConfig* config, std::string* message) const;

private:
    static std::string Trim(const std::string& value);
    static bool ParseInt(const std::string& value, int* output);
    static bool ParseDouble(const std::string& value, double* output);
    static bool ParseBool(const std::string& value, bool* output);
    static bool ParseVector3(const std::string& value, Vector3Config* output);
};

} // namespace weld

#endif // APP_CONFIG_H_
