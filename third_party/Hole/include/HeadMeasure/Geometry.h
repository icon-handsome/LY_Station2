#pragma once

#include "HeadMeasure/Types.h"

#include <vector>

namespace hm {

struct CircleFit2D {
    Eigen::Vector2d center{0.0, 0.0};
    double radiusMm{0.0};
    FitReport report;
};

CircleFit2D fitCircleLeastSquares(const std::vector<Eigen::Vector2d>& points,
                                  const std::string& reportName);

double evaluateRoundnessMinimumZone(const std::vector<Eigen::Vector2d>& points,
                                    const Eigen::Vector2d& initialCenter);

double angleDeg(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

Eigen::Vector3d closestPointOnLine(const Eigen::Vector3d& p,
                                   const Eigen::Vector3d& linePoint,
                                   const Eigen::Vector3d& lineAxis);

}  // namespace hm
