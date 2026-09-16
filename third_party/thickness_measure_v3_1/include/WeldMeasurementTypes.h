#ifndef WELD_MEASUREMENT_TYPES_H_
#define WELD_MEASUREMENT_TYPES_H_

#include <limits>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace weld
{

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

enum WeldSide
{
    WeldSide_Left = 0,
    WeldSide_Right = 1
};

struct Line2D
{
    // z = k * x + b
    double k;
    double b;
    bool valid;

    Line2D() : k(0.0), b(0.0), valid(false) {}

    double Evaluate(double x) const
    {
        return k * x + b;
    }
};

struct WeldToe
{
    PointT point;
    bool valid;

    WeldToe() : point(), valid(false) {}
};

struct WeldToePair
{
    WeldToe left;
    WeldToe right;

    // 模型固定输出左右两个焊趾槽位；该数量用于调用方判定截面是否可参与计算。
    int ValidToeCount() const
    {
        return (left.valid ? 1 : 0) + (right.valid ? 1 : 0);
    }

    bool IsValid() const
    {
        return ValidToeCount() == 2;
    }
};

struct MeasurementOptions
{
    double mismatchFitLengthMm;
    double angularityHalfSpanMm;
    double minFitPoints;

    MeasurementOptions()
        : mismatchFitLengthMm(40.0),
          angularityHalfSpanMm(150.0),
          minFitPoints(6.0)
    {
    }
};

struct WeldMetrics
{
    double mismatchMm;
    double reinforcementMm;
    double angularityMm;
    double toeCenterX;
    double includedAngleRad;
    Line2D leftBaseLine;
    Line2D rightBaseLine;
    bool valid;

    WeldMetrics()
        : mismatchMm(0.0),
          reinforcementMm(0.0),
          angularityMm(0.0),
          toeCenterX(0.0),
          includedAngleRad(0.0),
          leftBaseLine(),
          rightBaseLine(),
          valid(false)
    {
    }
};

} // namespace weld

#endif // WELD_MEASUREMENT_TYPES_H_
