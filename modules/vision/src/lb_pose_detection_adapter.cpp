#include "scan_tracking/vision/lb_pose_detection_adapter.h"

#include <QtCore/QByteArray>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QVector>

#include <opencv2/opencv.hpp>

#include "TR_Mark_3D_Recon.h"
#include "TR_Mark_Track.h"

namespace scan_tracking {
namespace vision {

namespace {

// 双目立体标定参数集合，对应 config.ini [LbPose] 中的内参/畸变/外参
struct StereoCalibSet {
    cv::Mat I1; // 左目 3×3 内参
    cv::Mat D1; // 左目 1×5 畸变
    cv::Mat E1; // 左目 4×4 外参
    cv::Mat I2; // 右目 3×3 内参
    cv::Mat D2; // 右目 1×5 畸变
    cv::Mat E2; // 右目 4×4 外参
};

bool vectorHasSize(const QVector<double>& values, int expectedCount)
{
    return values.size() == expectedCount;
}

// 将行优先 QVector<double> 转为 OpenCV 矩阵（标定参数均为行优先存储）
cv::Mat matrixFromRowMajor(const QVector<double>& values, int rows, int cols)
{
    cv::Mat matrix(rows, cols, CV_64F);
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            matrix.at<double>(row, col) =
                values[static_cast<int>(row * cols + col)];
        }
    }
    return matrix;
}

// 从 LbPoseConfig 解析立体标定；任一数组长度不符则返回空 I1 作为失败标记
StereoCalibSet makeStereoCalibSetFromConfig(const scan_tracking::common::LbPoseConfig& config)
{
    StereoCalibSet calib;
    if (!vectorHasSize(config.leftIntrinsic3x3, 9) ||
        !vectorHasSize(config.leftDistortion5, 5) ||
        !vectorHasSize(config.leftExtrinsic4x4, 16) ||
        !vectorHasSize(config.rightIntrinsic3x3, 9) ||
        !vectorHasSize(config.rightDistortion5, 5) ||
        !vectorHasSize(config.rightExtrinsic4x4, 16)) {
        return {};
    }

    calib.I1 = matrixFromRowMajor(config.leftIntrinsic3x3, 3, 3);
    calib.D1 = matrixFromRowMajor(config.leftDistortion5, 1, 5);
    calib.E1 = matrixFromRowMajor(config.leftExtrinsic4x4, 4, 4);
    calib.I2 = matrixFromRowMajor(config.rightIntrinsic3x3, 3, 3);
    calib.D2 = matrixFromRowMajor(config.rightDistortion5, 1, 5);
    calib.E2 = matrixFromRowMajor(config.rightExtrinsic4x4, 4, 4);
    return calib;
}

HikPoseCaptureResult makeFailure(
    quint64 requestId,
    const QString& cameraKey,
    const QString& logicalName,
    VisionErrorCode code,
    const QString& message)
{
    HikPoseCaptureResult result;
    result.requestId = requestId;
    result.cameraKey = cameraKey;
    result.logicalName = logicalName;
    result.errorCode = code;
    result.errorMessage = message;
    result.elapsedMs = 0;
    return result;
}

LbPoseResult makeFailure(const QString& message)
{
    LbPoseResult result;
    result.invoked = true;
    result.success = false;
    result.message = message;
    return result;
}

LbPoseResult makeCalibFailure()
{
    return makeFailure(QStringLiteral("LB 立体标定参数无效，请检查 config.ini [LbPose]。"));
}

// 海康 Mono8 帧 → cv::Mat 灰度图（clone 避免引用外部 buffer 生命周期）
cv::Mat frameToGrayMat(const HikMonoFrame& frame)
{
    if (!frame.isValid() || frame.pixels == nullptr || frame.width <= 0 || frame.height <= 0) {
        return {};
    }
    if (frame.stride < frame.width) {
        return {};
    }

    cv::Mat view(frame.height, frame.width, CV_8UC1, frame.pixels->data(), static_cast<std::size_t>(frame.stride));
    return view.clone();
}

// TR_Mark 输出的 4×4 double Rt → PoseMatrix4x4（行优先 float）
PoseMatrix4x4 toPoseMatrix(const cv::Mat& rt, const QString& sourceCameraKey, quint64 frameId)
{
    PoseMatrix4x4 pose;
    if (rt.empty() || rt.rows != 4 || rt.cols != 4 || rt.type() != CV_64F) {
        return pose;
    }
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            pose.values[static_cast<std::size_t>(row * 4 + col)] = static_cast<float>(rt.at<double>(row, col));
        }
    }
    pose.frameId = frameId;
    pose.timestampMs = QDateTime::currentMSecsSinceEpoch();
    pose.sourceCameraKey = sourceCameraKey;
    pose.valid = true;
    return pose;
}

// 模板点云路径：优先 templateFile，否则 dataRoot/template-3D-ALL-Shift-Cut-Cut.txt
QString buildTemplatePath(const scan_tracking::common::LbPoseConfig& config)
{
    if (!config.templateFile.trimmed().isEmpty()) {
        return config.templateFile.trimmed();
    }
    const QString dataRoot = config.dataRoot.trimmed().isEmpty()
        ? QStringLiteral("D:/work/scan-tracking/third_party/lb_pose_detection/data")
        : config.dataRoot.trimmed();
    return QDir(dataRoot).filePath(QStringLiteral("template-3D-ALL-Shift-Cut-Cut.txt"));
}

}  // namespace

LbPoseResult runLbPoseDetection(
    const HikMonoFrame& leftFrame,
    const HikMonoFrame& rightFrame,
    const scan_tracking::common::LbPoseConfig& config)
{
    LbPoseResult result;
    result.invoked = true;
    result.leftImageWidth = leftFrame.width;
    result.leftImageHeight = leftFrame.height;
    result.rightImageWidth = rightFrame.width;
    result.rightImageHeight = rightFrame.height;

    // ---- 1. 输入校验与图像转换 ----
    if (!leftFrame.isValid() || !rightFrame.isValid()) {
        return makeFailure(QStringLiteral("LB 位姿检测需要两个有效的灰度图像帧。"));
    }

    const cv::Mat leftImage = frameToGrayMat(leftFrame);
    const cv::Mat rightImage = frameToGrayMat(rightFrame);
    if (leftImage.empty() || rightImage.empty()) {
        return makeFailure(QStringLiteral("将海康图像帧转换为 cv::Mat 灰度图像失败。"));
    }

    try {
        // ---- 2. 立体 3D 重建：标定 + 极线匹配 → frame_3d_points ----
        TR_INSPECT_3D_Recon_Marker recon;
        const auto calib = makeStereoCalibSetFromConfig(config);
        if (calib.I1.empty()) {
            return makeCalibFailure();
        }
        if (recon.Set_Calib_Config(calib.I1, calib.D1, calib.E1, calib.I2, calib.D2, calib.E2) != 0) {
            return makeFailure(QStringLiteral("配置 LB 立体标定失败。"));
        }
        if (recon.Set_2D_Config(
                config.epipolarThreshold,
                config.minZRange,
                config.maxZRange,
                config.maxReprojErr,
                config.maxRatio) != 0) {
            return makeFailure(QStringLiteral("配置 LB 二维重建参数失败。"));
        }

        if (recon.Get_3D_Recon_Marker(const_cast<cv::Mat&>(leftImage), const_cast<cv::Mat&>(rightImage)) != 0) {
            return makeFailure(QStringLiteral("TR_INSPECT_3D_Recon_Marker 重建立体点云失败。"));
        }

        // ---- 3. 几何哈希模板匹配：加载模板 → build → Get_Track_Pose → Rt ----
        FastGeoHash geoHash(config.maxDistance, config.minDistance);
        if (geoHash.set_template_config(config.minDistance, config.maxDistance) != 0) {
            return makeFailure(QStringLiteral("设置 LB 模板配置失败。"));
        }
        if (geoHash.set_query_config(config.cosTolerance, config.minPercent) != 0) {
            return makeFailure(QStringLiteral("设置 LB 查询配置失败。"));
        }
        
        const QString templatePath = buildTemplatePath(config);
        QByteArray templateBytes = templatePath.toLocal8Bit();
        if (geoHash.read_template_pnts(templateBytes.data()) != 0) {
            return makeFailure(QStringLiteral("从 %1 加载 LB 模板点云失败。").arg(templatePath));
        }
        if (geoHash.build() != 0) {
			return makeFailure(QStringLiteral("构建 LB 几何哈希表失败。"));
        }

        const int trackResult = geoHash.Get_Track_Pose(
            recon.frame_3d_points,
            config.cosTolerance,
            config.minPercent);
        if (trackResult != 0) {
            return makeFailure(QStringLiteral("LB 跟踪返回错误代码 %1。").arg(trackResult));
        }

        result.success = true;
        result.message = QStringLiteral("LB 位姿检测成功完成。");
        result.framePointCount = static_cast<int>(recon.frame_3d_points.size());
        result.poseMatrix = toPoseMatrix(geoHash.Rt, QStringLiteral("lb_pose_detection"), 0);
        if (!result.poseMatrix.isValid()) {
            return makeFailure(QStringLiteral("LB 位姿检测产生了无效的 Rt 矩阵。"));
        }
        return result;
    } catch (const std::exception& ex) {
        // TR_Mark / OpenCV 可能抛出 std::exception，统一转为 LbPoseResult 失败
        return makeFailure(QStringLiteral("LB 位姿检测抛出异常：%1").arg(QString::fromLocal8Bit(ex.what())));
    } catch (...) {
        return makeFailure(QStringLiteral("LB 位姿检测抛出未知异常。"));
    }
}

}  // namespace vision
}  // namespace scan_tracking
