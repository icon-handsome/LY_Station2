#pragma once

#include <cstddef>

#include <QtCore/QString>
#include <QtCore/QVector>

namespace scan_tracking::thickness_measure_v2 {

struct ThicknessV2PairMeasurement {
    double innerOuterIcpFitness = 0.0;
    double outerTemplateIcpFitness = 0.0;
    double thicknessMm = 0.0;
    QString method;
    int sectionCount = 0;
    bool valid = false;
};

struct ThicknessV2AverageMeasurement {
    double thicknessMm = 0.0;
    size_t pairCount = 0;
    size_t successCount = 0;
    bool valid = false;
};

struct ThicknessV2CloudView {
    const float* xyz = nullptr;
    size_t pointCount = 0;
};

struct ThicknessV2PairClouds {
    ThicknessV2CloudView inner;
    ThicknessV2CloudView outer;
};

struct ThicknessV2Error {
    int statusCode = 0;
    QString message;
};

}  // namespace scan_tracking::thickness_measure_v2
