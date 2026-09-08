#pragma once

#include <string>

#ifdef _WIN32
#  ifdef INNER_SURFACE_MEASURE_V2_IMPORTS
#    define ISM_V2_API __declspec(dllimport)
#  else
#    define ISM_V2_API
#  endif
#else
#  define ISM_V2_API
#endif

namespace ism {

struct Cylinder {
    float point[3];
    float axis[3];
    float radius;
};

struct MeasurementResult {
    Cylinder cylinder;
    double diameter;
    double circumference;
    double volumeLiters;
    double sectionRoundness[4];
    double averageRoundness;
    int usedPointCount;
    bool roundnessValid;
};

}  // namespace ism

class ISM_V2_API InnerSurfaceMeasure {
public:
    InnerSurfaceMeasure();
    ~InnerSurfaceMeasure();
    InnerSurfaceMeasure(const InnerSurfaceMeasure&) = delete;
    InnerSurfaceMeasure& operator=(const InnerSurfaceMeasure&) = delete;

    bool LoadIni(const std::string& iniPath, std::string* error = nullptr);
    bool IsLoaded() const;
    bool Measure(ism::MeasurementResult results[2],
                 double* averageDiameter,
                 double* averageCircumference,
                 double* averageRoundness,
                 double* averageVolumeLiters,
                 std::string* error = nullptr) const;
    double ContainerLength() const;
};

extern "C" ISM_V2_API InnerSurfaceMeasure* CreateInnerSurfaceMeasure();
extern "C" ISM_V2_API void DestroyInnerSurfaceMeasure(InnerSurfaceMeasure* instance);
