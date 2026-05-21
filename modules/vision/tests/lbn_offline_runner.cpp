#include "scan_tracking/vision/lbn_pose_detection_adapter.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <QtCore/QCommandLineParser>
#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct OrganizedCloudLoadResult {
    scan_tracking::mech_eye::PointCloudFrame frame;
    int validPoints = 0;
    int nanPoints = 0;
    QString error;
};

bool parseOrganizedDimensions(int vertexCount, int& width, int& height)
{
    static const int kCandidates[][2] = {
        {2400, 1800},
        {1920, 1200},
        {1280, 1024},
        {2048, 1536},
    };

    for (const auto& candidate : kCandidates) {
        if (candidate[0] * candidate[1] == vertexCount) {
            width = candidate[0];
            height = candidate[1];
            return true;
        }
    }

    for (int w = 1; w * w <= vertexCount; ++w) {
        if (vertexCount % w != 0) {
            continue;
        }
        const int h = vertexCount / w;
        if (w >= 640 && h >= 480) {
            width = w;
            height = h;
            return true;
        }
    }
    return false;
}

OrganizedCloudLoadResult loadOrganizedAsciiPly(const QString& plyPath, int forcedWidth, int forcedHeight)
{
    OrganizedCloudLoadResult result;
    QFile file(plyPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        result.error = QStringLiteral("无法打开 PLY: %1").arg(plyPath);
        return result;
    }

    int vertexCount = 0;
    bool inHeader = true;
    while (inHeader && !file.atEnd()) {
        const QString line = QString::fromUtf8(file.readLine()).trimmed();
        if (line.startsWith(QStringLiteral("element vertex"))) {
            vertexCount = line.section(' ', -1).toInt();
        }
        if (line == QStringLiteral("end_header")) {
            inHeader = false;
        }
    }

    if (vertexCount <= 0) {
        result.error = QStringLiteral("PLY 未解析到 vertex 数量");
        return result;
    }

    int width = forcedWidth;
    int height = forcedHeight;
    if (width <= 0 || height <= 0) {
        if (!parseOrganizedDimensions(vertexCount, width, height)) {
            result.error = QStringLiteral("无法推断组织化点云宽高，请指定 --cloud-width/--cloud-height");
            return result;
        }
    } else if (width * height != vertexCount) {
        result.error = QStringLiteral("宽高与 vertex 数量不一致: %1x%2 != %3")
                           .arg(width)
                           .arg(height)
                           .arg(vertexCount);
        return result;
    }

    auto points = std::make_shared<std::vector<float>>(static_cast<std::size_t>(vertexCount) * 3U, 0.0f);
    int index = 0;
    while (!file.atEnd() && index < vertexCount) {
        const QString line = QString::fromUtf8(file.readLine()).trimmed();
        if (line.isEmpty()) {
            continue;
        }
        const QStringList parts = line.split(' ', Qt::SkipEmptyParts);
        if (parts.size() < 3) {
            continue;
        }

        bool okX = false;
        bool okY = false;
        bool okZ = false;
        const float x = parts[0].toFloat(&okX);
        const float y = parts[1].toFloat(&okY);
        const float z = parts[2].toFloat(&okZ);
        if (!okX || !okY || !okZ) {
            continue;
        }

        const std::size_t offset = static_cast<std::size_t>(index) * 3U;
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            (*points)[offset + 0] = x;
            (*points)[offset + 1] = y;
            (*points)[offset + 2] = z;
            ++result.validPoints;
        } else {
            (*points)[offset + 0] = 0.0f;
            (*points)[offset + 1] = 0.0f;
            (*points)[offset + 2] = 0.0f;
            ++result.nanPoints;
        }

        ++index;
        if (index % 500000 == 0) {
            std::cout << "  PLY 已读取 " << index << " / " << vertexCount << " 点\n";
        }
    }

    if (index != vertexCount) {
        result.error = QStringLiteral("PLY 实际读取点数 %1 与 header %2 不一致").arg(index).arg(vertexCount);
        return result;
    }

    result.frame.pointsXYZ = std::move(points);
    result.frame.width = width;
    result.frame.height = height;
    result.frame.pointCount = vertexCount;
    result.frame.frameId = 1;
    return result;
}

bool loadGrayTextureResized(
    const QString& imagePath,
    int targetWidth,
    int targetHeight,
    scan_tracking::mech_eye::GrayTextureFrame& texture,
    QString& message)
{
    const cv::Mat loaded = cv::imread(imagePath.toStdString(), cv::IMREAD_UNCHANGED);
    if (loaded.empty()) {
        message = QStringLiteral("无法读取图像: %1").arg(imagePath);
        return false;
    }

    cv::Mat gray;
    if (loaded.channels() == 1) {
        gray = loaded;
    } else if (loaded.channels() == 3 || loaded.channels() == 4) {
        cv::cvtColor(loaded, gray, loaded.channels() == 4 ? cv::COLOR_BGRA2GRAY : cv::COLOR_BGR2GRAY);
    } else {
        message = QStringLiteral("不支持的图像通道数: %1").arg(loaded.channels());
        return false;
    }

    cv::Mat resized;
    if (gray.cols != targetWidth || gray.rows != targetHeight) {
        cv::resize(gray, resized, cv::Size(targetWidth, targetHeight), 0, 0, cv::INTER_LINEAR);
        message = QStringLiteral("纹理由 %1x%2 缩放到 %3x%4")
                      .arg(gray.cols)
                      .arg(gray.rows)
                      .arg(targetWidth)
                      .arg(targetHeight);
    } else {
        resized = gray;
        message = QStringLiteral("纹理尺寸已与点云一致 %1x%2").arg(targetWidth).arg(targetHeight);
    }

    if (!resized.isContinuous()) {
        resized = resized.clone();
    }

    texture.width = targetWidth;
    texture.height = targetHeight;
    texture.pixels = std::make_shared<std::vector<uint8_t>>(
        static_cast<std::size_t>(targetWidth * targetHeight));
    std::memcpy(
        texture.pixels->data(),
        resized.data,
        texture.pixels->size());
    return texture.isValid();
}

scan_tracking::common::LbnPoseConfig defaultLbnConfig()
{
    scan_tracking::common::LbnPoseConfig config;
    config.enabled = true;
    config.dataRoot = QStringLiteral("D:/work/LY/IPC-192.168.110.173_track-main/third_party/LBN/data");
    config.templateFile =
        QStringLiteral("D:/work/LY/IPC-192.168.110.173_track-main/third_party/LBN/data/template-3D-ALL-Shift-Cut-Cut.txt");
    config.minDistance = 30.0f;
    config.maxDistance = 650.0f;
    config.cosTolerance = 0.015f;
    config.minPercent = 0.5f;
    config.cloudSearchRadiusPx = 20;
    return config;
}

bool resolveGroupFiles(const QString& groupDir, QString& imagePath, QString& plyPath, QString& error)
{
    QDir dir(groupDir);
    if (!dir.exists()) {
        error = QStringLiteral("目录不存在: %1").arg(groupDir);
        return false;
    }

    const auto jpgs = dir.entryList(QStringList() << "*.jpg" << "*.jpeg" << "*.png", QDir::Files);
    const auto plys = dir.entryList(QStringList() << "*.ply", QDir::Files);
    if (jpgs.isEmpty() || plys.isEmpty()) {
        error = QStringLiteral("目录内需要 1 张图像和 1 个 PLY: %1").arg(groupDir);
        return false;
    }

    imagePath = dir.filePath(jpgs.front());
    QString texturedPly;
    for (const QString& name : plys) {
        if (name.contains(QStringLiteral("textured"), Qt::CaseInsensitive)) {
            texturedPly = dir.filePath(name);
            break;
        }
    }
    plyPath = texturedPly.isEmpty() ? dir.filePath(plys.front()) : texturedPly;
    return true;
}

void printPoseMatrix(const scan_tracking::vision::PoseMatrix4x4& pose)
{
    for (int row = 0; row < 4; ++row) {
        std::cout << "  ["
                  << pose.values[static_cast<std::size_t>(row * 4 + 0)] << ", "
                  << pose.values[static_cast<std::size_t>(row * 4 + 1)] << ", "
                  << pose.values[static_cast<std::size_t>(row * 4 + 2)] << ", "
                  << pose.values[static_cast<std::size_t>(row * 4 + 3)] << "]\n";
    }
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("scan_tracking_lbn_offline_runner"));

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("离线加载 Mech-Eye 纹理图 + 组织化 PLY，调用 LBN 位姿检测"));
    parser.addHelpOption();

    QCommandLineOption groupOption(
        QStringList() << "g"
                      << "group",
        QStringLiteral("测试数据目录（自动选取 jpg/png + textured ply）"),
        QStringLiteral("dir"));
    QCommandLineOption imageOption(
        QStringList() << "i"
                      << "image",
        QStringLiteral("纹理图路径（彩色或灰度）"),
        QStringLiteral("path"));
    QCommandLineOption plyOption(
        QStringList() << "p"
                      << "ply",
        QStringLiteral("组织化 textured PLY 路径"),
        QStringLiteral("path"));
    QCommandLineOption cloudWidthOption(
        QStringList() << "cloud-width", QStringLiteral("点云宽（默认从 vertex 数推断）"), QStringLiteral("w"));
    QCommandLineOption cloudHeightOption(
        QStringList() << "cloud-height", QStringLiteral("点云高（默认从 vertex 数推断）"), QStringLiteral("h"));
    QCommandLineOption templateOption(
        QStringList() << "t"
                      << "template",
        QStringLiteral("GeoHash 模板文件"),
        QStringLiteral("path"));

    parser.addOption(groupOption);
    parser.addOption(imageOption);
    parser.addOption(plyOption);
    parser.addOption(cloudWidthOption);
    parser.addOption(cloudHeightOption);
    parser.addOption(templateOption);
    parser.addPositionalArgument(
        QStringLiteral("files"),
        QStringLiteral("可选: <image> <ply>（未使用 --group 时）"));
    parser.process(app);

    QString imagePath = parser.value(imageOption);
    QString plyPath = parser.value(plyOption);
    if (parser.isSet(groupOption)) {
        QString error;
        if (!resolveGroupFiles(parser.value(groupOption), imagePath, plyPath, error)) {
            std::cerr << error.toStdString() << '\n';
            return 2;
        }
    } else {
        const QStringList pos = parser.positionalArguments();
        if (imagePath.isEmpty() && pos.size() >= 1) {
            imagePath = pos.at(0);
        }
        if (plyPath.isEmpty() && pos.size() >= 2) {
            plyPath = pos.at(1);
        }
    }

    if (imagePath.isEmpty() || plyPath.isEmpty()) {
        std::cerr << "用法:\n"
                  << "  scan_tracking_lbn_offline_runner --group testdata/group1\n"
                  << "  scan_tracking_lbn_offline_runner -i texture.jpg -p cloud.ply\n";
        return 2;
    }

    const int forcedWidth = parser.value(cloudWidthOption).toInt();
    const int forcedHeight = parser.value(cloudHeightOption).toInt();

    std::cout << "图像: " << imagePath.toStdString() << '\n';
    std::cout << "点云: " << plyPath.toStdString() << '\n';

    const auto cloudLoad = loadOrganizedAsciiPly(plyPath, forcedWidth, forcedHeight);
    if (!cloudLoad.error.isEmpty()) {
        std::cerr << cloudLoad.error.toStdString() << '\n';
        return 3;
    }

    std::cout << "点云网格: " << cloudLoad.frame.width << "x" << cloudLoad.frame.height
              << " valid=" << cloudLoad.validPoints << " nan=" << cloudLoad.nanPoints << '\n';

    scan_tracking::mech_eye::GrayTextureFrame texture;
    QString textureMessage;
    if (!loadGrayTextureResized(
            imagePath,
            cloudLoad.frame.width,
            cloudLoad.frame.height,
            texture,
            textureMessage)) {
        std::cerr << textureMessage.toStdString() << '\n';
        return 4;
    }
    std::cout << textureMessage.toStdString() << '\n';

    scan_tracking::mech_eye::CaptureResult capture;
    capture.errorCode = scan_tracking::mech_eye::CaptureErrorCode::Success;
    capture.mode = scan_tracking::mech_eye::CaptureMode::Capture2DAnd3D;
    capture.cameraKey = QStringLiteral("offline");
    capture.pointCloud = cloudLoad.frame;
    capture.texture2D = texture;

    auto lbnConfig = defaultLbnConfig();
    if (parser.isSet(templateOption)) {
        lbnConfig.templateFile = parser.value(templateOption);
    }

    std::cout << "LBN 模板: " << lbnConfig.templateFile.toStdString() << '\n';
    std::cout << "调用 runLbnPoseDetection...\n";

    const auto lbnResult = scan_tracking::vision::runLbnPoseDetection(capture, lbnConfig);

    std::cout << "invoked=" << lbnResult.invoked << '\n';
    std::cout << "success=" << lbnResult.success << '\n';
    std::cout << "message=" << lbnResult.message.toStdString() << '\n';
    std::cout << "matchedPointCount=" << lbnResult.matchedPointCount << '\n';

    if (lbnResult.poseMatrix.isValid()) {
        std::cout << "Rt 4x4:\n";
        printPoseMatrix(lbnResult.poseMatrix);
    }

    return lbnResult.success ? 0 : 5;
}
