#include "scan_tracking/vision/lbn_pose_detection_adapter.h"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>

#include <algorithm>
#include <cstring>

#include <opencv2/opencv.hpp>

#include "lbn_pose_core.h"

namespace scan_tracking {
namespace vision {

namespace {

LbnPoseResult makeFailure(const QString& message)
{
    LbnPoseResult result;
    result.invoked = true;
    result.success = false;
    result.message = message;
    return result;
}

cv::Mat grayTextureToMat(const scan_tracking::mech_eye::GrayTextureFrame& texture)
{
    cv::Mat gray;
    if (!texture.isValid()) {
        return gray;
    }
    lbn_pose::grayImageToCvMat(
        texture.pixels->data(),
        texture.width,
        texture.height,
        gray);
    return gray;
}

lbn_pose::AlignedOrganizedCloud toAlignedCloud(const scan_tracking::mech_eye::PointCloudFrame& cloud)
{
    lbn_pose::AlignedOrganizedCloud aligned;
    aligned.width = cloud.width;
    aligned.height = cloud.height;
    if (!cloud.isValid() || cloud.width <= 0 || cloud.height <= 0 || !cloud.pointsXYZ) {
        return aligned;
    }

    const int pointCount = cloud.width * cloud.height;
    if (static_cast<int>(cloud.pointsXYZ->size()) < pointCount * 3) {
        return aligned;
    }

    thread_local std::vector<cv::Point3f> organizedPoints;
    organizedPoints.resize(static_cast<std::size_t>(pointCount));
    for (int index = 0; index < pointCount; ++index) {
        const float x = (*cloud.pointsXYZ)[static_cast<std::size_t>(index * 3 + 0)];
        const float y = (*cloud.pointsXYZ)[static_cast<std::size_t>(index * 3 + 1)];
        const float z = (*cloud.pointsXYZ)[static_cast<std::size_t>(index * 3 + 2)];
        organizedPoints[static_cast<std::size_t>(index)] = cv::Point3f(x, y, z);
    }

    aligned.points = organizedPoints.data();
    return aligned;
}

PoseMatrix4x4 toPoseMatrix(const cv::Mat& rtGlobal, const QString& sourceCameraKey, quint64 frameId)
{
    PoseMatrix4x4 pose;
    if (rtGlobal.empty() || rtGlobal.rows != 4 || rtGlobal.cols != 4) {
        return pose;
    }

    cv::Mat rt64;
    if (rtGlobal.type() == CV_64F) {
        rt64 = rtGlobal;
    } else {
        rtGlobal.convertTo(rt64, CV_64F);
    }

    float values[16] = {};
    if (!lbn_pose::rtGlobalToRowMajor16(rt64, values)) {
        return pose;
    }

    for (int i = 0; i < 16; ++i) {
        pose.values[static_cast<std::size_t>(i)] = values[i];
    }
    pose.frameId = frameId;
    pose.timestampMs = QDateTime::currentMSecsSinceEpoch();
    pose.sourceCameraKey = sourceCameraKey;
    pose.valid = true;
    return pose;
}

QString buildTemplatePath(const scan_tracking::common::LbnPoseConfig& config)
{
    if (!config.templateFile.trimmed().isEmpty()) {
        return config.templateFile.trimmed();
    }

    const QString templateName = QStringLiteral("template-3D-ALL-Shift-Cut-Cut.txt");
    const QStringList candidateRoots = {
        config.dataRoot.trimmed(),
        QStringLiteral("D:/work/LY/IPC-192.168.110.173_track-main/third_party/LBN/data"),
        QStringLiteral("D:/work/LY/IPC-192.168.110.173_track-main/third_party/LB/Data"),
    };

    for (const QString& root : candidateRoots) {
        if (root.isEmpty()) {
            continue;
        }
        const QString candidate = QDir(root).filePath(templateName);
        if (QFileInfo::exists(candidate)) {
            return candidate;
        }
    }

    return QDir(candidateRoots.at(1)).filePath(templateName);
}

// 将 IPC 配置映射到 lbn_pose::Config（含检测/投票参数，影响生产匹配松紧）
lbn_pose::Config toCoreConfig(const scan_tracking::common::LbnPoseConfig& config)
{
    lbn_pose::Config coreConfig;
    coreConfig.maxDistance = config.maxDistance;
    coreConfig.minDistance = config.minDistance;
    coreConfig.cosTolerance = config.cosTolerance;
    coreConfig.minPercent = config.minPercent;
    coreConfig.cloudSearchRadiusPx = config.cloudSearchRadiusPx;
    coreConfig.markerMinArea = config.markerMinArea;
    coreConfig.markerMaxArea = config.markerMaxArea;
    coreConfig.markerIntensityThreshold = config.markerIntensityThreshold;
    coreConfig.markerDebscanDistPx = config.markerDebscanDistPx;
    coreConfig.templateFilePath = buildTemplatePath(config).toStdString();
    return coreConfig;
}

void fillMarkerDebug(const lbn_pose::Result& coreResult, LbnPoseMarkerDebug& debug)
{
    debug.centers2dCount = static_cast<int>(coreResult.centers2d.size());
    debug.centers3dCount = static_cast<int>(coreResult.centers3d.size());

    const int count2d = std::min(debug.centers2dCount, 32);
    for (int i = 0; i < count2d; ++i) {
        debug.centers2dU[i] = coreResult.centers2d[static_cast<std::size_t>(i)].x;
        debug.centers2dV[i] = coreResult.centers2d[static_cast<std::size_t>(i)].y;
    }

    const int count3d = std::min(debug.centers3dCount, 32);
    for (int i = 0; i < count3d; ++i) {
        debug.centers3dX[i] = coreResult.centers3d[static_cast<std::size_t>(i)].x;
        debug.centers3dY[i] = coreResult.centers3d[static_cast<std::size_t>(i)].y;
        debug.centers3dZ[i] = coreResult.centers3d[static_cast<std::size_t>(i)].z;
    }
}

}  // namespace

LbnPoseResult runLbnPoseDetection(
    const scan_tracking::mech_eye::CaptureResult& mechEyeResult,
    const scan_tracking::common::LbnPoseConfig& config)
{
    return runLbnPoseDetection(mechEyeResult, config, nullptr);
}

LbnPoseResult runLbnPoseDetection(
    const scan_tracking::mech_eye::CaptureResult& mechEyeResult,
    const scan_tracking::common::LbnPoseConfig& config,
    LbnPoseMarkerDebug* markerDebug)
{
    LbnPoseResult result;
    result.invoked = true;
    result.textureWidth = mechEyeResult.texture2D.width;
    result.textureHeight = mechEyeResult.texture2D.height;
    result.pointCloudWidth = mechEyeResult.pointCloud.width;
    result.pointCloudHeight = mechEyeResult.pointCloud.height;

    if (!config.enabled) {
        result.invoked = false;
        result.message = QStringLiteral("LBN 位姿检测已在配置中禁用。");
        return result;
    }

    if (!mechEyeResult.success()) {
        return makeFailure(QStringLiteral("Mech-Eye 采集失败，无法执行 LBN 位姿检测。"));
    }

    if (mechEyeResult.mode != scan_tracking::mech_eye::CaptureMode::Capture2DAnd3D) {
        return makeFailure(QStringLiteral("LBN 需要 Capture2DAnd3D 模式采集。"));
    }

    if (!mechEyeResult.texture2D.isValid()) {
        return makeFailure(QStringLiteral("Mech-Eye 2D 纹理图无效。"));
    }

    if (!mechEyeResult.pointCloud.isValid() || mechEyeResult.pointCloud.width <= 0 ||
        mechEyeResult.pointCloud.height <= 0) {
        return makeFailure(QStringLiteral("Mech-Eye 对齐点云无效。"));
    }

    const cv::Mat grayImage = grayTextureToMat(mechEyeResult.texture2D);
    if (grayImage.empty()) {
        return makeFailure(QStringLiteral("无法将 Mech-Eye 纹理图转换为灰度 cv::Mat。"));
    }

    const lbn_pose::AlignedOrganizedCloud alignedCloud = toAlignedCloud(mechEyeResult.pointCloud);
    if (alignedCloud.points == nullptr) {
        return makeFailure(QStringLiteral("无法构建 LBN 对齐点云视图。"));
    }

    try {
        lbn_pose::Estimator estimator(toCoreConfig(config));
        const lbn_pose::Result coreResult = estimator.estimate(grayImage, alignedCloud);
        if (markerDebug != nullptr) {
            fillMarkerDebug(coreResult, *markerDebug);
        }

        if (!coreResult.success) {
            return makeFailure(
                QString::fromStdString(coreResult.message.empty() ? "LBN pose estimation failed."
                                                                  : coreResult.message));
        }

        result.success = true;
        result.message = QStringLiteral("LBN 位姿检测成功完成。");
        result.matchedPointCount = coreResult.matchedPointCount;
        result.poseMatrix = toPoseMatrix(
            coreResult.rtGlobal,
            mechEyeResult.cameraKey,
            mechEyeResult.pointCloud.frameId);
        if (!result.poseMatrix.isValid()) {
            return makeFailure(QStringLiteral("LBN 位姿检测产生了无效的 Rt 矩阵。"));
        }
        return result;
    } catch (const std::exception& ex) {
        return makeFailure(
            QStringLiteral("LBN 位姿检测抛出异常：%1").arg(QString::fromLocal8Bit(ex.what())));
    } catch (...) {
        return makeFailure(QStringLiteral("LBN 位姿检测抛出未知异常。"));
    }
}

}  // namespace vision
}  // namespace scan_tracking
