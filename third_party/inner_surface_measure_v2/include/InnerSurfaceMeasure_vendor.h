#pragma once

#include "Measurement.h"

#include <string>

// Public facade. PCL clouds and all temporary allocations stay inside the DLL.
class ISM_API InnerSurfaceMeasure
{
public:
    InnerSurfaceMeasure();
    ~InnerSurfaceMeasure();

    InnerSurfaceMeasure(const InnerSurfaceMeasure&) = delete;
    InnerSurfaceMeasure& operator=(const InnerSurfaceMeasure&) = delete;

    // Loads the existing INI format. Relative cloud paths are resolved from its directory.
    bool LoadIni(const std::string& iniPath, std::string* error = nullptr);
    bool IsLoaded() const;

    // Measures the two frames configured in the INI and returns per-frame plus average values.
    bool Measure(ism::MeasurementResult results[2],
                 double* averageDiameter,
                 double* averageCircumference,
                 double* averageRoundness,
                 double* averageVolumeLiters,
                 std::string* error = nullptr) const;

    // C++ callers can use the same workflow without exposing PCL ownership.
    bool MeasureConfigured(ism::MeasurementResult results[2], std::string* error = nullptr) const;
    double ContainerLength() const;

private:
    struct Impl;
    Impl* impl_;
};

// Allocate and destroy in the same module. Do not use delete on a handle from another module.
extern "C" ISM_API InnerSurfaceMeasure* CreateInnerSurfaceMeasure();
extern "C" ISM_API void DestroyInnerSurfaceMeasure(InnerSurfaceMeasure* instance);
