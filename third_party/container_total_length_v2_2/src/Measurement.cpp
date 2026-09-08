#include "Measurement.h"

#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

namespace {


float AxisCoordinate(const PointT& point, const Cylinder& cylinder)
{
    const Eigen::Vector3f value(point.x, point.y, point.z);
    return (value - cylinder.point).dot(cylinder.axis);
}

bool DetectEndsByOutsideScan(CloudConstPtr rawCloud,
                             const Eigen::Matrix4f& transform,
                             const Cylinder& cylinder,
                             const Config& config,
                             EndDetectionResult ends[2],
                             std::string* diagnostic)
{
    if (!rawCloud || rawCloud->empty()) return false;

    CloudPtr cloud(new CloudT);
    pcl::transformPointCloud(*rawCloud, *cloud, transform);

    std::vector<double> projections;
    projections.reserve(cloud->size());
    double minCoordinate = std::numeric_limits<double>::max();
    double maxCoordinate = -std::numeric_limits<double>::max();
    for (size_t i = 0; i < cloud->size(); ++i)
	{
        if (!pcl::isFinite(cloud->points[i])) continue;
        const double coordinate = AxisCoordinate(cloud->points[i], cylinder);
        projections.push_back(coordinate);
        minCoordinate = std::min(minCoordinate, coordinate);
        maxCoordinate = std::max(maxCoordinate, coordinate);
    }
    if (projections.empty()) return false;

    // 对投影坐标排序后取分位数，左右端分别落在极小分位和极大分位上。
    const double percentile = std::max(0.000001, std::min(0.2, config.refineEdgePercentile));
    const auto Quantile = [](std::vector<double> values, double ratio) -> double {
        if (values.empty()) return 0.0;
        ratio = std::max(0.0, std::min(1.0, ratio));
        const size_t index = static_cast<size_t>(ratio * (values.size() - 1));
        std::nth_element(values.begin(), values.begin() + index, values.end());
        return values[index];
    };
    const double leftPosition = Quantile(projections, percentile);
    const double rightPosition = Quantile(projections, 1.0 - percentile);
    if (!(rightPosition > leftPosition))
    {
        if (diagnostic)
        {
            std::ostringstream stream;
            stream << "outsideScan percentile failed: pointCount=" << projections.size()
                   << ", axialRange=[" << minCoordinate << ", " << maxCoordinate << "]"
                   << ", percentile=" << percentile;
            *diagnostic = stream.str();
        }
        return false;
    }

	ends[0].refinedPosition = leftPosition;
	ends[0].refinedPointCount = static_cast<int>(projections.size());
	ends[0].usedEndNormals = false;
	ends[1].refinedPosition = rightPosition;
	ends[1].refinedPointCount = static_cast<int>(projections.size());
	ends[1].usedEndNormals = false;
    return true;
}

}  // namespace

Cylinder::Cylinder() : point(Eigen::Vector3f::Zero()), axis(Eigen::Vector3f::UnitZ())
{
}

EndDetectionResult::EndDetectionResult() : templatePosition(0.0), coarsePosition(0.0), refinedPosition(0.0),
                                           usedEndNormals(false), coarsePointCount(0), refinedPointCount(0)
{
}

MeasurementResult::MeasurementResult() : icpTransform(Eigen::Matrix4f::Identity()), icpFitness(0.0),
                                         icpConverged(false), length(0.0)
{
}

CloudPtr VoxelDownsample(CloudConstPtr cloud, float voxelSize)
{
    CloudPtr output(new CloudT);
    if (!cloud || cloud->empty())
		return output;
    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    filter.filter(*output);
    return output;
}

CloudPtr RemoveStatisticalOutliers(CloudConstPtr cloud, int meanK, double stdMultiplier)
{
    CloudPtr output(new CloudT);
    if (!cloud || cloud->empty()) 
		return output;
    if (meanK <= 1)
	{
        *output = *cloud;
        return output;
    }
    pcl::StatisticalOutlierRemoval<PointT> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(meanK);
    filter.setStddevMulThresh(stdMultiplier);
    filter.filter(*output);
    return output;
}

bool MeasureContainerLength(CloudConstPtr rawInput,
                            CloudConstPtr templateCloud,
                            const Config& config,
                            MeasurementResult* result,
                            std::string* error)
{
    if (!rawInput || rawInput->empty() || !templateCloud || templateCloud->empty() || !result)
    {
        if (error)
        {
            *error = "Input, template and result must be valid.";
        }
        return false;
    }

    // 输入点云和模板点云先降采样，减少 ICP 计算量；离群点过滤只作用在扫描输入上。
    CloudPtr inputDown = VoxelDownsample(rawInput, config.voxelSize);
    //inputDown = RemoveStatisticalOutliers(inputDown, config.outlierK, config.outlierStd);
    //CloudPtr templateDown = VoxelDownsample(templateCloud, config.voxelSize);
	if (inputDown->size() < 30 || templateCloud->size() < 30)
    {
        if (error)
        {
            *error = "Too few points after voxel downsampling.";
        }
        return false;
    }

    // 与参考工程保持一致：把扫描点云匹配到模板点云，后续在模板坐标系中拟合轴线和统计端点。
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(inputDown);
	icp.setInputTarget(templateCloud);
    icp.setMaximumIterations(config.icpMaxIterations);
    icp.setMaxCorrespondenceDistance(config.icpMaxCorrespondenceDistance);
    icp.setTransformationEpsilon(config.icpTransformationEpsilon);
    icp.setEuclideanFitnessEpsilon(config.icpEuclideanFitnessEpsilon);
    CloudPtr aligned(new CloudT);
    icp.align(*aligned);
    result->icpConverged = icp.hasConverged();
    result->icpFitness = icp.getFitnessScore();
    result->icpTransform = icp.getFinalTransformation();
    if (!result->icpConverged || aligned->size() < 30)
    {
        if (error)
        {
            *error = "ICP did not converge.";
        }
        return false;
    }
	 
	result->cylinder.axis = config.templateAxisDirection.normalized();

	// 只要柱面轴线不变，没有必要拟合柱面，中心点在长度计算中也会抵消掉
    //// 先拟合容器圆柱轴线和半径，outsideScan 端点搜索需要沿该轴线做一维投影统计。
    // pcl::io::savePCDFileBinary("aligned.pcd", *aligned);
    //{
    //    if (error)
    //    {
    //    }
    //    return false;
    //}
    //          << ", point: (" << result->cylinder.point.x() << ", " << result->cylinder.point.y() << ", " << result->cylinder.point.z() << ")"

    // 容器总长只保留 outsideScan 这一条端点检测路径，不再使用模板窗口/法向端点分支。
    std::string diagnostic;
    if (!DetectEndsByOutsideScan(rawInput, result->icpTransform, result->cylinder, config, result->ends, &diagnostic))
    {
        if (error)
        {
            *error = "Cannot find both ends by outside scan. " + diagnostic;
        }
        return false;
    }

    // DetectEndsByOutsideScan 返回的是沿轴向直方图得到的左右端位置，这里直接用它计算总长。
	result->ends[0].refinedPosition = result->ends[0].refinedPosition;
	result->ends[1].refinedPosition = result->ends[1].refinedPosition;
	result->length = result->ends[1].refinedPosition - result->ends[0].refinedPosition;

    return result->length > 0.0;
}
