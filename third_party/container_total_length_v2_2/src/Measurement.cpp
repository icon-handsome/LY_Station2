#include "Measurement.h"
#include "CtlTrace.h"

#include <pcl/common/common.h>
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

void LogCloudStats(const char* tag, CloudConstPtr cloud)
{
    if (!cloud) {
        CTL_TRACE("%s: cloud=null", tag);
        return;
    }
    if (cloud->empty()) {
        CTL_TRACE("%s: empty size=0", tag);
        return;
    }

    Eigen::Vector4f minPt;
    Eigen::Vector4f maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    const double dx = static_cast<double>(maxPt.x() - minPt.x());
    const double dy = static_cast<double>(maxPt.y() - minPt.y());
    const double dz = static_cast<double>(maxPt.z() - minPt.z());
    const double estBytes = static_cast<double>(cloud->size()) * sizeof(PointT);
    CTL_TRACE("%s: size=%llu bbox=[(%.3f,%.3f,%.3f)-(%.3f,%.3f,%.3f)] span=(%.3f,%.3f,%.3f) estMB=%.1f",
              tag,
              static_cast<unsigned long long>(cloud->size()),
              minPt.x(), minPt.y(), minPt.z(),
              maxPt.x(), maxPt.y(), maxPt.z(),
              dx, dy, dz,
              estBytes / (1024.0 * 1024.0));
}

void LogVoxelGridPlan(const char* tag, CloudConstPtr cloud, float voxelSize)
{
    if (!cloud || cloud->empty() || voxelSize <= 0.0f) {
        CTL_TRACE("%s: skip voxel plan (invalid cloud/voxel)", tag);
        return;
    }
    Eigen::Vector4f minPt;
    Eigen::Vector4f maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    const double dx = std::max(0.0, static_cast<double>(maxPt.x() - minPt.x()));
    const double dy = std::max(0.0, static_cast<double>(maxPt.y() - minPt.y()));
    const double dz = std::max(0.0, static_cast<double>(maxPt.z() - minPt.z()));
    const double nx = std::floor(dx / voxelSize) + 1.0;
    const double ny = std::floor(dy / voxelSize) + 1.0;
    const double nz = std::floor(dz / voxelSize) + 1.0;
    const double leafCount = nx * ny * nz;
    CTL_TRACE("%s: voxel=%.3f leafGrid~=(%.0f x %.0f x %.0f)=%.3e cells (warning if huge)",
              tag, voxelSize, nx, ny, nz, leafCount);
}

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
    CTL_TRACE("DetectEnds: enter rawSize=%llu",
              rawCloud ? static_cast<unsigned long long>(rawCloud->size()) : 0ull);
    CTL_TRACE_MEM("DetectEnds:enter");
    if (!rawCloud || rawCloud->empty()) return false;

    LogCloudStats("DetectEnds:raw", rawCloud);
    CTL_TRACE("DetectEnds: before transformPointCloud (alloc+copy full raw)");
    const unsigned long long t0 = ctl_trace::NowTick();

    CloudPtr cloud(new CloudT);
    try {
        pcl::transformPointCloud(*rawCloud, *cloud, transform);
    } catch (const std::exception& ex) {
        CTL_TRACE("DetectEnds: transformPointCloud EXCEPTION: %s", ex.what());
        if (diagnostic) *diagnostic = std::string("transformPointCloud exception: ") + ex.what();
        return false;
    } catch (...) {
        CTL_TRACE("DetectEnds: transformPointCloud UNKNOWN EXCEPTION");
        if (diagnostic) *diagnostic = "transformPointCloud unknown exception";
        return false;
    }

    CTL_TRACE("DetectEnds: after transformPointCloud size=%llu elapsedMs=%.0f",
              static_cast<unsigned long long>(cloud->size()),
              ctl_trace::MsSince(t0));
    CTL_TRACE_MEM("DetectEnds:afterTransform");
    LogCloudStats("DetectEnds:transformed", cloud);

    CTL_TRACE("DetectEnds: begin axial projection");
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
    CTL_TRACE("DetectEnds: projections=%llu axialRange=[%.3f, %.3f]",
              static_cast<unsigned long long>(projections.size()),
              minCoordinate, maxCoordinate);
    if (projections.empty()) return false;

    // ��ͶӰ���������ȡ��λ�������Ҷ˷ֱ����ڼ�С��λ�ͼ����λ�ϡ�
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
    CTL_TRACE("DetectEnds: percentile=%.6f left=%.3f right=%.3f",
              percentile, leftPosition, rightPosition);
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
        CTL_TRACE("DetectEnds: FAIL percentile order");
        return false;
    }

	ends[0].refinedPosition = leftPosition;
	ends[0].refinedPointCount = static_cast<int>(projections.size());
	ends[0].usedEndNormals = false;
	ends[1].refinedPosition = rightPosition;
	ends[1].refinedPointCount = static_cast<int>(projections.size());
	ends[1].usedEndNormals = false;
    CTL_TRACE("DetectEnds: OK length=%.3f", rightPosition - leftPosition);
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
    CTL_TRACE("VoxelDownsample: enter size=%llu voxel=%.3f",
              static_cast<unsigned long long>(cloud->size()), voxelSize);
    LogCloudStats("VoxelDownsample:input", cloud);
    LogVoxelGridPlan("VoxelDownsample:plan", cloud, voxelSize);
    CTL_TRACE_MEM("VoxelDownsample:beforeFilter");
    const unsigned long long t0 = ctl_trace::NowTick();
    try {
        pcl::VoxelGrid<PointT> filter;
        filter.setInputCloud(cloud);
        filter.setLeafSize(voxelSize, voxelSize, voxelSize);
        filter.filter(*output);
    } catch (const std::exception& ex) {
        CTL_TRACE("VoxelDownsample: EXCEPTION: %s", ex.what());
        throw;
    } catch (...) {
        CTL_TRACE("VoxelDownsample: UNKNOWN EXCEPTION");
        throw;
    }
    CTL_TRACE("VoxelDownsample: done outSize=%llu elapsedMs=%.0f",
              static_cast<unsigned long long>(output->size()),
              ctl_trace::MsSince(t0));
    CTL_TRACE_MEM("VoxelDownsample:afterFilter");
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
    CTL_TRACE("MeasureContainerLength: ENTER");
    CTL_TRACE_MEM("Measure:enter");
    if (!rawInput || rawInput->empty() || !templateCloud || templateCloud->empty() || !result)
    {
        CTL_TRACE("MeasureContainerLength: invalid args raw=%p tmpl=%p result=%p",
                  static_cast<const void*>(rawInput.get()),
                  static_cast<const void*>(templateCloud.get()),
                  static_cast<const void*>(result));
        if (error)
        {
            *error = "Input, template and result must be valid.";
        }
        return false;
    }

    LogCloudStats("Measure:rawInput", rawInput);
    LogCloudStats("Measure:template", templateCloud);
    CTL_TRACE("Measure: config voxel=%.3f outlierK=%d icpMaxIter=%d icpMaxCorr=%.1f crop=%d",
              config.voxelSize,
              config.outlierK,
              config.icpMaxIterations,
              config.icpMaxCorrespondenceDistance,
              config.cropInputCloud ? 1 : 0);

    // ������ƺ�ģ������Ƚ����������� ICP ����������Ⱥ�����ֻ������ɨ�������ϡ�
    CTL_TRACE("Measure: STEP1 begin VoxelDownsample(raw)");
    CloudPtr inputDown;
    try {
        inputDown = VoxelDownsample(rawInput, config.voxelSize);
    } catch (const std::exception& ex) {
        CTL_TRACE("Measure: STEP1 VoxelDownsample EXCEPTION: %s", ex.what());
        if (error) *error = std::string("VoxelDownsample exception: ") + ex.what();
        return false;
    } catch (...) {
        CTL_TRACE("Measure: STEP1 VoxelDownsample UNKNOWN EXCEPTION");
        if (error) *error = "VoxelDownsample unknown exception";
        return false;
    }
    CTL_TRACE("Measure: STEP1 done inputDown=%llu",
              static_cast<unsigned long long>(inputDown ? inputDown->size() : 0ull));
    //inputDown = RemoveStatisticalOutliers(inputDown, config.outlierK, config.outlierStd);
    //CloudPtr templateDown = VoxelDownsample(templateCloud, config.voxelSize);
	if (inputDown->size() < 30 || templateCloud->size() < 30)
    {
        CTL_TRACE("Measure: FAIL too few points after voxel");
        if (error)
        {
            *error = "Too few points after voxel downsampling.";
        }
        return false;
    }

    // ��ο����̱���һ�£���ɨ�����ƥ�䵽ģ����ƣ�������ģ������ϵ��������ߺ�ͳ�ƶ˵㡣
    CTL_TRACE("Measure: STEP2 begin ICP source=%llu target=%llu",
              static_cast<unsigned long long>(inputDown->size()),
              static_cast<unsigned long long>(templateCloud->size()));
    CTL_TRACE_MEM("Measure:beforeIcp");
    const unsigned long long tIcp = ctl_trace::NowTick();
    CloudPtr aligned(new CloudT);
    try {
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(inputDown);
        icp.setInputTarget(templateCloud);
        icp.setMaximumIterations(config.icpMaxIterations);
        icp.setMaxCorrespondenceDistance(config.icpMaxCorrespondenceDistance);
        icp.setTransformationEpsilon(config.icpTransformationEpsilon);
        icp.setEuclideanFitnessEpsilon(config.icpEuclideanFitnessEpsilon);
        CTL_TRACE("Measure: STEP2 icp.align() calling...");
        icp.align(*aligned);
        result->icpConverged = icp.hasConverged();
        result->icpFitness = icp.getFitnessScore();
        result->icpTransform = icp.getFinalTransformation();
    } catch (const std::exception& ex) {
        CTL_TRACE("Measure: STEP2 ICP EXCEPTION: %s", ex.what());
        if (error) *error = std::string("ICP exception: ") + ex.what();
        return false;
    } catch (...) {
        CTL_TRACE("Measure: STEP2 ICP UNKNOWN EXCEPTION");
        if (error) *error = "ICP unknown exception";
        return false;
    }
    CTL_TRACE("Measure: STEP2 ICP done converged=%d fitness=%.6f aligned=%llu elapsedMs=%.0f",
              result->icpConverged ? 1 : 0,
              result->icpFitness,
              static_cast<unsigned long long>(aligned->size()),
              ctl_trace::MsSince(tIcp));
    CTL_TRACE_MEM("Measure:afterIcp");
    if (!result->icpConverged || aligned->size() < 30)
    {
        CTL_TRACE("Measure: FAIL ICP did not converge");
        if (error)
        {
            *error = "ICP did not converge.";
        }
        return false;
    }
	 
	result->cylinder.axis = config.templateAxisDirection.normalized();
    CTL_TRACE("Measure: cylinder.axis=(%.4f,%.4f,%.4f)",
              result->cylinder.axis.x(),
              result->cylinder.axis.y(),
              result->cylinder.axis.z());

	// ֻҪ�������߲��䣬û�б�Ҫ������棬���ĵ��ڳ��ȼ�����Ҳ�������
    //// ���������Բ�����ߺͰ뾶��outsideScan �˵�������Ҫ�ظ�������һάͶӰͳ�ơ�
    // pcl::io::savePCDFileBinary("aligned.pcd", *aligned);
    //{
    //    if (error)
    //    {
    //    }
    //    return false;
    //}
    //          << ", point: (" << result->cylinder.point.x() << ", " << result->cylinder.point.y() << ", " << result->cylinder.point.z() << ")"

    // �����ܳ�ֻ���� outsideScan ��һ���˵���·��������ʹ��ģ�崰��/����˵��֧��
    CTL_TRACE("Measure: STEP3 begin DetectEndsByOutsideScan (uses FULL raw cloud)");
    std::string diagnostic;
    if (!DetectEndsByOutsideScan(rawInput, result->icpTransform, result->cylinder, config, result->ends, &diagnostic))
    {
        CTL_TRACE("Measure: STEP3 FAIL %s", diagnostic.c_str());
        if (error)
        {
            *error = "Cannot find both ends by outside scan. " + diagnostic;
        }
        return false;
    }
    CTL_TRACE("Measure: STEP3 done ends=[%.3f, %.3f]",
              result->ends[0].refinedPosition,
              result->ends[1].refinedPosition);

    // DetectEndsByOutsideScan ���ص���������ֱ��ͼ�õ������Ҷ�λ�ã�����ֱ�����������ܳ���
	result->ends[0].refinedPosition = result->ends[0].refinedPosition;
	result->ends[1].refinedPosition = result->ends[1].refinedPosition;
	result->length = result->ends[1].refinedPosition - result->ends[0].refinedPosition;

    CTL_TRACE("MeasureContainerLength: OK length=%.3f", result->length);

    // TEST ONLY: skip destroying large clouds �� leak shared_ptr holders so refcount
    // stays >0 after local reset. Verifies crash-on-dtor hypothesis. Do not ship.
    CTL_TRACE("Measure: TEST_LEAK begin (skip dtor) inputDown=%llu aligned=%llu",
              static_cast<unsigned long long>(inputDown ? inputDown->size() : 0ull),
              static_cast<unsigned long long>(aligned ? aligned->size() : 0ull));
    CTL_TRACE_MEM("Measure:beforeLeak");
    (void)new CloudPtr(inputDown);
    (void)new CloudPtr(aligned);
    inputDown.reset();
    aligned.reset();
    CTL_TRACE("Measure: TEST_LEAK holders kept; locals reset without free");
    CTL_TRACE_MEM("Measure:exit");
    return result->length > 0.0;
}
