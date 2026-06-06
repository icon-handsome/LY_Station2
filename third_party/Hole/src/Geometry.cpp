#include "HeadMeasure/Geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hm {
namespace {
const double kPi = 3.141592653589793238462643383279502884;
}

CircleFit2D fitCircleLeastSquares(const std::vector<Eigen::Vector2d>& points,
                                  const std::string& reportName) {
    CircleFit2D fit;
    fit.report.name = reportName;
    fit.report.inlierCount = static_cast<int>(points.size());
    if (points.size() < 3) {
        fit.report.rmsMm = std::numeric_limits<double>::quiet_NaN();
        fit.report.maxAbsMm = std::numeric_limits<double>::quiet_NaN();
        return fit;
    }

    Eigen::MatrixXd a(points.size(), 3);
    Eigen::VectorXd b(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        const double x = points[i].x();
        const double y = points[i].y();
        a(static_cast<int>(i), 0) = x;
        a(static_cast<int>(i), 1) = y;
        a(static_cast<int>(i), 2) = 1.0;
        b(static_cast<int>(i)) = -(x * x + y * y);
    }

    const Eigen::Vector3d sol = a.colPivHouseholderQr().solve(b);
    fit.center = {-0.5 * sol.x(), -0.5 * sol.y()};
    fit.radiusMm = std::sqrt(std::max(0.0, fit.center.squaredNorm() - sol.z()));

    double sum2 = 0.0;
    double maxAbs = 0.0;
    for (const auto& p : points) {
        const double residual = (p - fit.center).norm() - fit.radiusMm;
        sum2 += residual * residual;
        maxAbs = std::max(maxAbs, std::abs(residual));
    }
    fit.report.rmsMm = std::sqrt(sum2 / static_cast<double>(points.size()));
    fit.report.maxAbsMm = maxAbs;
    return fit;
}

// 圆度误差按最小区域圆思想计算：
// 对同一截面的轮廓点，寻找一个圆心，使所有点到该圆心的最大半径 maxR
// 与最小半径 minR 之差最小。这个最小的半径差 maxR - minR 即圆度误差。
//
// initialCenter 是圆心搜索初值，通常来自当前截面的圆拟合结果。
// 本函数使用局部网格搜索近似求解：从初值开始，在上下左右和对角 8 个方向
// 尝试移动圆心；如果某个候选圆心让 maxR - minR 更小，就接受该圆心；
// 如果 8 个方向都没有改善，就减小步长继续搜索，直到步长足够小。
//
// 注意：最小区域圆对外点非常敏感，因此调用前应先剔除明显外点。
double evaluateRoundnessMinimumZone(const std::vector<Eigen::Vector2d>& points,
                                    const Eigen::Vector2d& initialCenter)
{
    if (points.size() < 4)
	{
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 计算指定圆心下的圆度区域宽度：最大半径 - 最小半径。
    auto zoneAt = [&points](const Eigen::Vector2d& center) 
	{
        double minR = std::numeric_limits<double>::max();
        double maxR = 0.0;
        for (const auto& p : points)
		{
            const double r = (p - center).norm();
            minR = std::min(minR, r);
            maxR = std::max(maxR, r);
        }
        return maxR - minR;
    };

    // best 保存当前搜索到的最优圆心，bestZone 保存该圆心对应的最小区域宽度。
    Eigen::Vector2d best = initialCenter;
    double bestZone = zoneAt(best);
    // 初始搜索步长，单位与点云一致。后续若无法改善，会逐步减半。
    double step = 5.0;

    // GB/T 7235 style roundness evaluation commonly uses the minimum zone circle.
    // This local search minimizes max radius minus min radius around the section center.
    for (int iter = 0; iter < 120; ++iter) 
	{
        bool improved = false;
        for (const Eigen::Vector2d& d : {Eigen::Vector2d(step, 0.0), Eigen::Vector2d(-step, 0.0),
                                         Eigen::Vector2d(0.0, step), Eigen::Vector2d(0.0, -step),
                                         Eigen::Vector2d(step, step), Eigen::Vector2d(step, -step),
                                         Eigen::Vector2d(-step, step), Eigen::Vector2d(-step, -step)}) 
		{
            const Eigen::Vector2d candidate = best + d;
            const double zone = zoneAt(candidate); // zone = maxR - minR
            if (zone < bestZone)
			{
                best = candidate;
                bestZone = zone;
                improved = true;
            }
        }
        // 当前步长下找不到更优圆心时，缩小步长做更精细搜索。
        if (!improved)
		{
            step *= 0.5;
        }
        if (step < 1e-4) 
		{
            break;
        }
    }

    return bestZone;
}

double angleDeg(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    const double denom = a.norm() * b.norm();
    if (denom <= 0.0)
	{
        return std::numeric_limits<double>::quiet_NaN();
    }
    double c = a.dot(b) / denom;
    c = std::max(-1.0, std::min(1.0, c));
    return std::acos(c) * 180.0 / kPi;
}

Eigen::Vector3d closestPointOnLine(const Eigen::Vector3d& p,
                                   const Eigen::Vector3d& linePoint,
                                   const Eigen::Vector3d& lineAxis) 
{
    const Eigen::Vector3d axis = lineAxis.normalized();
    return linePoint + axis * ((p - linePoint).dot(axis));
}

}  // namespace hm

