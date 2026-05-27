#include "scan_tracking/mech_eye/point_cloud_processor.h"

#include <QLoggingCategory>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>

Q_LOGGING_CATEGORY(LOG_POINT_CLOUD_PROC, "mech_eye.point_cloud_processor")

namespace scan_tracking::mech_eye {

// PCL/Eigen 在 Windows 下非线程安全；多段后台 refinement 与蓝友检测并发时会偶发 aligned_free 崩溃。
std::mutex& pointCloudAlgorithmMutex()
{
    static std::mutex mutex;
    return mutex;
}

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = Cloud::Ptr;

bool isFinitePoint(float x, float y, float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

CloudPtr toPclCloud(const PointCloudFrame& frame)
{
    auto cloud = pcl::make_shared<Cloud>();
    if (!frame.pointsXYZ || frame.pointCount <= 0) {
        return cloud;
    }

    const auto& points = *frame.pointsXYZ;
    const int availablePointCount = static_cast<int>(points.size() / 3);
    const int pointCount = std::min(frame.pointCount, availablePointCount);
    cloud->points.reserve(static_cast<std::size_t>(pointCount));

    for (int index = 0; index < pointCount; ++index) {
        const auto base = static_cast<std::size_t>(index * 3);
        const float x = points[base];
        const float y = points[base + 1];
        const float z = points[base + 2];
        if (!isFinitePoint(x, y, z)) {
            continue;
        }
        cloud->points.emplace_back(x, y, z);
    }

    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

PointCloudFrame fromPclCloud(const CloudPtr& cloud, const PointCloudFrame& metadata)
{
    PointCloudFrame frame;
    frame.frameId = metadata.frameId;
    frame.timestampMs = metadata.timestampMs;
    frame.pointsXYZ = std::make_shared<std::vector<float>>();
    frame.normalsXYZ.reset();

    if (!cloud || cloud->empty()) {
        frame.width = 0;
        frame.height = 0;
        frame.pointCount = 0;
        return frame;
    }

    const int count = static_cast<int>(cloud->size());
    frame.pointsXYZ->reserve(static_cast<std::size_t>(count) * 3);
    for (const auto& point : cloud->points) {
        frame.pointsXYZ->push_back(point.x);
        frame.pointsXYZ->push_back(point.y);
        frame.pointsXYZ->push_back(point.z);
    }

    frame.pointCount = count;
    frame.width = count;
    frame.height = 1;
    return frame;
}

PointCloudFrame clonePointCloudFrameFull(const PointCloudFrame& src)
{
    PointCloudFrame dst = src;
    if (src.pointsXYZ) {
        dst.pointsXYZ = std::make_shared<std::vector<float>>(*src.pointsXYZ);
    }
    if (src.normalsXYZ) {
        dst.normalsXYZ = std::make_shared<std::vector<float>>(*src.normalsXYZ);
    }
    return dst;
}

bool checkMinPoints(const CloudPtr& cloud, int minPoints, const QString& stepLabel, QString* message)
{
    if (!cloud || static_cast<int>(cloud->size()) < minPoints) {
        if (message != nullptr) {
            *message = QStringLiteral("%1 后点数不足: %2 < %3")
                           .arg(stepLabel)
                           .arg(cloud ? static_cast<int>(cloud->size()) : 0)
                           .arg(minPoints);
        }
        return false;
    }
    return true;
}

}  // namespace

bool processPointCloudFrame(
    const PointCloudFrame& input,
    const common::PointCloudProcessingConfig& config,
    PointCloudFrame* output,
    PointCloudProcessReport* report)
{
    if (output == nullptr) {
        return false;
    }

    std::lock_guard<std::mutex> pclLock(pointCloudAlgorithmMutex());

    PointCloudProcessReport localReport;
    localReport.inputPointCount = input.pointCount;

    if (!input.isValid()) {
        localReport.message = QStringLiteral("输入点云无效。");
        if (report != nullptr) {
            *report = localReport;
        }
        return false;
    }

    if (!config.enabled) {
        *output = clonePointCloudFrameFull(input);
        localReport.outputPointCount = output->pointCount;
        localReport.message = QStringLiteral("点云后处理已禁用，直通原始点云。");
        if (report != nullptr) {
            *report = localReport;
        }
        return true;
    }

    CloudPtr cloud = toPclCloud(input);
    if (cloud->empty()) {
        localReport.message = QStringLiteral("有效点数为 0，无法后处理。");
        if (report != nullptr) {
            *report = localReport;
        }
        return false;
    }

    qDebug(LOG_POINT_CLOUD_PROC).noquote()
        << QStringLiteral("点云后处理开始，有效点数=") << cloud->size();

    const int minPoints = std::max(1, config.minPointsAfterProcessing);
    QString failMessage;

    if (config.depthMinMm < config.depthMaxMm) {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(config.depthMinMm, config.depthMaxMm);
        CloudPtr filtered = pcl::make_shared<Cloud>();
        pass.filter(*filtered);
        cloud = filtered;
        qDebug(LOG_POINT_CLOUD_PROC).noquote()
            << QStringLiteral("深度裁剪 z=[") << config.depthMinMm << QStringLiteral(",") << config.depthMaxMm
            << QStringLiteral("] mm，剩余点数=") << cloud->size();
        if (!checkMinPoints(cloud, minPoints, QStringLiteral("深度裁剪"), &failMessage)) {
            localReport.message = failMessage;
            if (report != nullptr) {
                *report = localReport;
            }
            return false;
        }
    }

    if (config.outlierRemovalEnabled && cloud->size() > static_cast<std::size_t>(config.outlierMeanK)) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(static_cast<int>(std::max(1, config.outlierMeanK)));
        sor.setStddevMulThresh(config.outlierStddevMul);
        CloudPtr filtered = pcl::make_shared<Cloud>();
        sor.filter(*filtered);
        cloud = filtered;
        qDebug(LOG_POINT_CLOUD_PROC).noquote()
            << QStringLiteral("离群点去除 meanK=") << config.outlierMeanK
            << QStringLiteral(" stddevMul=") << config.outlierStddevMul
            << QStringLiteral("，剩余点数=") << cloud->size();
        if (!checkMinPoints(cloud, minPoints, QStringLiteral("离群点去除"), &failMessage)) {
            localReport.message = failMessage;
            if (report != nullptr) {
                *report = localReport;
            }
            return false;
        }
    }

    if (config.smoothingEnabled && config.mlsSearchRadiusMm > 0.0f && !cloud->empty()) {
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setInputCloud(cloud);
        mls.setSearchRadius(config.mlsSearchRadiusMm);
        mls.setPolynomialOrder(std::max(1, config.mlsPolynomialOrder));
        mls.setComputeNormals(false);
        CloudPtr smoothed = pcl::make_shared<Cloud>();
        mls.process(*smoothed);
        cloud = smoothed;
        qDebug(LOG_POINT_CLOUD_PROC).noquote()
            << QStringLiteral("表面平滑 MLS 半径=") << config.mlsSearchRadiusMm
            << QStringLiteral(" mm 阶数=") << config.mlsPolynomialOrder
            << QStringLiteral("，剩余点数=") << cloud->size();
        if (!checkMinPoints(cloud, minPoints, QStringLiteral("表面平滑"), &failMessage)) {
            localReport.message = failMessage;
            if (report != nullptr) {
                *report = localReport;
            }
            return false;
        }
    }

    if (config.downsampleEnabled && config.voxelLeafSizeMm > 0.0f) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(
            config.voxelLeafSizeMm,
            config.voxelLeafSizeMm,
            config.voxelLeafSizeMm);
        CloudPtr filtered = pcl::make_shared<Cloud>();
        voxel.filter(*filtered);
        cloud = filtered;
        qDebug(LOG_POINT_CLOUD_PROC).noquote()
            << QStringLiteral("体素降采样 leaf=") << config.voxelLeafSizeMm
            << QStringLiteral(" mm，剩余点数=") << cloud->size();
        if (!checkMinPoints(cloud, minPoints, QStringLiteral("体素降采样"), &failMessage)) {
            localReport.message = failMessage;
            if (report != nullptr) {
                *report = localReport;
            }
            return false;
        }
    }

    *output = fromPclCloud(cloud, input);
    localReport.outputPointCount = output->pointCount;
    localReport.message = QStringLiteral(
        "点云后处理完成: %1 -> %2 点")
                              .arg(localReport.inputPointCount)
                              .arg(localReport.outputPointCount);

    qInfo(LOG_POINT_CLOUD_PROC).noquote() << localReport.message;

    if (report != nullptr) {
        *report = localReport;
    }
    return output->isValid();
}

}  // namespace scan_tracking::mech_eye
