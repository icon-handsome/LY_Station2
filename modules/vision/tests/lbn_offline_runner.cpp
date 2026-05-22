#include "scan_tracking/vision/lbn_pose_detection_adapter.h"

#include "lbn_pose_core.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <QtCore/QCommandLineParser>
#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QTextStream>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

// -----------------------------------------------------------------------------
// 离线 LBN runner ↔ third_party/LBN/main.cpp 对照（旧选项均保留，勿删）
//
// 【默认 = main.cpp 离线等价】（未加下列“旧/扩展”开关时）
//   1) PLY 无效点 → quiet_NaN（同 SDK / main isFinitePoint）
//   2) 纹理：优先 *texture_aligned*.bmp，尺寸须与点云网格一致，不缩放（同 frame2dToCvMat）
//   3) 2D：MarkPointDetector::ProcessFrame → 3D：queryPointInterpolated(radius=20) → 成功才 push
//   4) FastGeoHash(650,30) + Get_Track_Pose（由 lbn_pose_core / adapter 执行）
//
// 【扩展】--texture-from-ply：用 PLY 顶点 RGB 作灰度（main 联机不用此路径）
// 【扩展】--allow-resize-texture：允许 jpg 缩放到点云网格（旧 testdata 缩略图）
// 【旧行为】--legacy-zero-nan：无效点写 0（历史离线 bug，仅对比）
// -----------------------------------------------------------------------------

namespace {

struct OrganizedCloudLoadResult {
    scan_tracking::mech_eye::PointCloudFrame frame;
    scan_tracking::mech_eye::GrayTextureFrame texture;
    bool textureFromPly = false;
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

bool plyHeaderHasVertexColor(const QString& plyPath)
{
    QFile file(plyPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }

    while (!file.atEnd()) {
        const QString line = QString::fromUtf8(file.readLine()).trimmed().toLower();
        if (line == QStringLiteral("end_header")) {
            break;
        }
        if (line.contains(QStringLiteral("property uchar red")) ||
            line.contains(QStringLiteral("property uchar green"))) {
            return true;
        }
    }
    return false;
}

uint8_t rgbToGray(int red, int green, int blue)
{
    const int gray = (299 * red + 587 * green + 114 * blue + 500) / 1000;
    return static_cast<uint8_t>(std::max(0, std::min(gray, 255)));
}

OrganizedCloudLoadResult loadOrganizedAsciiPly(
    const QString& plyPath,
    int forcedWidth,
    int forcedHeight,
    bool extractGrayTexture,
    bool legacyZeroNaN)
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

    if (extractGrayTexture && !plyHeaderHasVertexColor(plyPath)) {
        result.error = QStringLiteral("PLY 无顶点颜色属性，无法 --texture-from-ply");
        return result;
    }

    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    const float invalidFill = legacyZeroNaN ? 0.0f : nanValue;
    auto points = std::make_shared<std::vector<float>>(
        static_cast<std::size_t>(vertexCount) * 3U,
        invalidFill);
    std::shared_ptr<std::vector<uint8_t>> grayPixels;
    if (extractGrayTexture) {
        grayPixels = std::make_shared<std::vector<uint8_t>>(static_cast<std::size_t>(vertexCount), 0U);
        result.textureFromPly = true;
    }

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
            // 旧版离线曾写 0,0,0（见 --legacy-zero-nan）；默认写 NaN，与 SDK / main.cpp 一致
            (*points)[offset + 0] = invalidFill;
            (*points)[offset + 1] = invalidFill;
            (*points)[offset + 2] = invalidFill;
            ++result.nanPoints;
        }

        if (grayPixels && parts.size() >= 6) {
            bool okR = false;
            bool okG = false;
            bool okB = false;
            const int red = parts[3].toInt(&okR);
            const int green = parts[4].toInt(&okG);
            const int blue = parts[5].toInt(&okB);
            if (okR && okG && okB) {
                (*grayPixels)[static_cast<std::size_t>(index)] = rgbToGray(red, green, blue);
            }
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

    if (grayPixels) {
        result.texture.width = width;
        result.texture.height = height;
        result.texture.pixels = std::move(grayPixels);
        if (!result.texture.isValid()) {
            result.error = QStringLiteral("PLY 顶点灰度纹理无效");
        }
    }

    return result;
}

bool decodeImageToGray(const cv::Mat& loaded, cv::Mat& gray, QString& message)
{
    if (loaded.empty()) {
        message = QStringLiteral("图像为空");
        return false;
    }
    if (loaded.channels() == 1) {
        gray = loaded;
    } else if (loaded.channels() == 3 || loaded.channels() == 4) {
        cv::cvtColor(loaded, gray, loaded.channels() == 4 ? cv::COLOR_BGRA2GRAY : cv::COLOR_BGR2GRAY);
    } else {
        message = QStringLiteral("不支持的图像通道数: %1").arg(loaded.channels());
        return false;
    }
    return true;
}

// main.cpp：frame2dToCvMat / textureGray 与点云同宽高，不做 resize
bool loadGrayTextureMainCppStyle(
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
    if (!decodeImageToGray(loaded, gray, message)) {
        message = QStringLiteral("%1: %2").arg(imagePath, message);
        return false;
    }

    if (gray.cols != targetWidth || gray.rows != targetHeight) {
        message = QStringLiteral(
                       "纹理 %1x%2 与点云 %3x%4 不一致（main.cpp 要求对齐）。请使用 main 导出的 "
                       "*texture_aligned.bmp，或加 --allow-resize-texture / --texture-from-ply")
                       .arg(gray.cols)
                       .arg(gray.rows)
                       .arg(targetWidth)
                       .arg(targetHeight);
        return false;
    }

    if (!gray.isContinuous()) {
        gray = gray.clone();
    }

    texture.width = targetWidth;
    texture.height = targetHeight;
    texture.pixels = std::make_shared<std::vector<uint8_t>>(
        static_cast<std::size_t>(targetWidth * targetHeight));
    std::memcpy(texture.pixels->data(), gray.data, texture.pixels->size());
    message = QStringLiteral("纹理 %1x%2 与点云同网格（main.cpp 对齐模式）")
                  .arg(targetWidth)
                  .arg(targetHeight);
    return texture.isValid();
}

// 旧 testdata：jpg 缩略图缩放到点云网格（非 main 默认路径）
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
    if (!decodeImageToGray(loaded, gray, message)) {
        return false;
    }

    cv::Mat resized;
    if (gray.cols != targetWidth || gray.rows != targetHeight) {
        cv::resize(gray, resized, cv::Size(targetWidth, targetHeight), 0, 0, cv::INTER_LINEAR);
        message = QStringLiteral("纹理由 %1x%2 缩放到 %3x%4（--allow-resize-texture）")
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
    std::memcpy(texture.pixels->data(), resized.data, texture.pixels->size());
    return texture.isValid();
}

QString repoRootFromCwd()
{
    QDir dir(QDir::currentPath());
    for (int depth = 0; depth < 6; ++depth) {
        if (QFileInfo::exists(dir.filePath(QStringLiteral("CMakeLists.txt"))) &&
            QDir(dir.filePath(QStringLiteral("third_party/LBN/data"))).exists()) {
            return dir.absolutePath();
        }
        if (!dir.cdUp()) {
            break;
        }
    }
    return QDir::currentPath();
}

scan_tracking::common::LbnPoseConfig defaultLbnConfig()
{
    const QString root = repoRootFromCwd();
    const QString dataRoot = QDir(root).filePath(QStringLiteral("third_party/LBN/data"));

    scan_tracking::common::LbnPoseConfig config;
    config.enabled = true;
    config.dataRoot = dataRoot;
    config.templateFile = QDir(dataRoot).filePath(QStringLiteral("template-3D-ALL-Shift-Cut-Cut.txt"));
    // 下列为 testdata/test scan_150200 离线验收参数，已同步 config.ini 仅供联调参考。
    // 生产 scan-tracking.exe 读 config.ini：容差偏松时 success 率升但误匹配风险升，勿未验证即上线。
    config.minDistance = 20.0f;
    config.maxDistance = 650.0f;
    config.cosTolerance = 0.05f;
    config.minPercent = 0.2f;
    config.cloudSearchRadiusPx = 20;
    config.markerMinArea = 200;
    config.markerMaxArea = 30000;
    config.markerIntensityThreshold = 40;
    config.markerDebscanDistPx = 120.0f;
    return config;
}

lbn_pose::Config toLbnCoreConfig(const scan_tracking::common::LbnPoseConfig& config)
{
    lbn_pose::Config core;
    core.maxDistance = config.maxDistance;
    core.minDistance = config.minDistance;
    core.cosTolerance = config.cosTolerance;
    core.minPercent = config.minPercent;
    core.cloudSearchRadiusPx = config.cloudSearchRadiusPx;
    core.markerMinArea = config.markerMinArea;
    core.markerMaxArea = config.markerMaxArea;
    core.markerIntensityThreshold = config.markerIntensityThreshold;
    core.markerDebscanDistPx = config.markerDebscanDistPx;
    core.templateFilePath = config.templateFile.toStdString();
    return core;
}

bool writeCenters3dTemplate(const QString& path, const scan_tracking::vision::LbnPoseMarkerDebug& debug)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(8);
    const int count = std::min(debug.centers3dCount, 32);
    for (int i = 0; i < count; ++i) {
        out << debug.centers3dX[i] << ' ' << debug.centers3dY[i] << ' ' << debug.centers3dZ[i] << '\n';
    }
    return count >= 3;
}

std::vector<cv::Point3f> markerDebugToPoints3d(const scan_tracking::vision::LbnPoseMarkerDebug& debug)
{
    std::vector<cv::Point3f> points;
    const int count = std::min(debug.centers3dCount, 32);
    points.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        points.emplace_back(debug.centers3dX[i], debug.centers3dY[i], debug.centers3dZ[i]);
    }
    return points;
}

bool saveMarkerDebugImage(
    const QString& imagePath,
    const QString& outputPath,
    const scan_tracking::vision::LbnPoseMarkerDebug& debug)
{
    cv::Mat bgr = cv::imread(imagePath.toStdString(), cv::IMREAD_COLOR);
    if (bgr.empty()) {
        return false;
    }
    const int count = std::min(debug.centers2dCount, 32);
    for (int i = 0; i < count; ++i) {
        const cv::Point center(
            static_cast<int>(std::round(debug.centers2dU[i])),
            static_cast<int>(std::round(debug.centers2dV[i])));
        cv::circle(bgr, center, 14, cv::Scalar(0, 0, 255), 2);
        cv::putText(
            bgr,
            std::to_string(i),
            center + cv::Point(16, -8),
            cv::FONT_HERSHEY_SIMPLEX,
            0.8,
            cv::Scalar(0, 255, 0),
            2);
    }
    return cv::imwrite(outputPath.toStdString(), bgr);
}

bool resolveGroupFiles(
    const QString& groupDir,
    QString& imagePath,
    QString& plyPath,
    bool textureFromPly,
    QString& error)
{
    QDir dir(groupDir);
    if (!dir.exists()) {
        error = QStringLiteral("目录不存在: %1").arg(groupDir);
        return false;
    }

    const auto plys = dir.entryList(QStringList() << "*.ply", QDir::Files);
    if (plys.isEmpty()) {
        error = QStringLiteral("目录内需要 1 个 PLY: %1").arg(groupDir);
        return false;
    }

    QString texturedPly;
    for (const QString& name : plys) {
        if (name.contains(QStringLiteral("textured"), Qt::CaseInsensitive)) {
            texturedPly = dir.filePath(name);
            break;
        }
    }
    plyPath = texturedPly.isEmpty() ? dir.filePath(plys.front()) : texturedPly;

    if (textureFromPly) {
        imagePath.clear();
        return true;
    }

    // 与 LBN main.cpp 导出对齐：优先 texture_aligned.bmp，避免误用缩略 jpg
    const auto images = dir.entryList(
        QStringList() << "*.bmp" << "*.png" << "*.jpg" << "*.jpeg",
        QDir::Files);
    if (images.isEmpty()) {
        error = QStringLiteral("目录内需要 1 张图像和 1 个 PLY（或使用 --texture-from-ply）: %1").arg(groupDir);
        return false;
    }

    QString alignedImage;
    QString fallbackImage;
    for (const QString& name : images) {
        if (name.contains(QStringLiteral("texture_aligned"), Qt::CaseInsensitive)) {
            alignedImage = dir.filePath(name);
            break;
        }
        if (fallbackImage.isEmpty()) {
            fallbackImage = dir.filePath(name);
        }
    }

    imagePath = alignedImage.isEmpty() ? fallbackImage : alignedImage;
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

void printMarkerDebug(const scan_tracking::vision::LbnPoseMarkerDebug& debug)
{
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "2D 圆心数量: " << debug.centers2dCount << '\n';
    const int count2d = std::min(debug.centers2dCount, 32);
    for (int i = 0; i < count2d; ++i) {
        std::cout << "  2D[" << i << "] u=" << debug.centers2dU[i] << " v=" << debug.centers2dV[i] << '\n';
    }

    std::cout << "3D 提升数量: " << debug.centers3dCount << '\n';
    const int count3d = std::min(debug.centers3dCount, 32);
    for (int i = 0; i < count3d; ++i) {
        std::cout << "  3D[" << i << "] x=" << debug.centers3dX[i] << " y=" << debug.centers3dY[i]
                  << " z=" << debug.centers3dZ[i] << '\n';
    }
    std::cout.unsetf(std::ios::floatfield);
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("scan_tracking_lbn_offline_runner"));

    QCommandLineParser parser;
    parser.setApplicationDescription(
        QStringLiteral("离线加载 Mech-Eye 纹理图 + 组织化 PLY，调用 LBN 位姿检测"));
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
    QCommandLineOption textureFromPlyOption(
        QStringList() << "texture-from-ply",
        QStringLiteral("从 PLY 顶点 RGB 生成与点云同网格的灰度纹理（推荐，避免 jpg 缩放错位）"));
    QCommandLineOption legacyZeroNaNOption(
        QStringList() << "legacy-zero-nan",
        QStringLiteral("【旧行为】PLY 无效点写 0 而非 NaN，仅用于对比历史离线结果"));
    QCommandLineOption allowResizeTextureOption(
        QStringList() << "allow-resize-texture",
        QStringLiteral("【扩展】允许纹理缩放到点云网格（非 main.cpp 默认）"));
    QCommandLineOption minDistanceOption(
        QStringList() << "min-distance", QStringLiteral("GeoHash 最小边长(mm)"), QStringLiteral("mm"));
    QCommandLineOption maxDistanceOption(
        QStringList() << "max-distance", QStringLiteral("GeoHash 最大边长(mm)"), QStringLiteral("mm"));
    QCommandLineOption cosToleranceOption(
        QStringList() << "cos-tolerance", QStringLiteral("角度余弦容差"), QStringLiteral("val"));
    QCommandLineOption minPercentOption(
        QStringList() << "min-percent", QStringLiteral("投票占比阈值"), QStringLiteral("val"));
    QCommandLineOption cloudRadiusOption(
        QStringList() << "cloud-radius", QStringLiteral("2D→3D 插值搜索半径(px)"), QStringLiteral("px"));
    QCommandLineOption markerMinAreaOption(
        QStringList() << "marker-min-area", QStringLiteral("标记点最小面积"), QStringLiteral("px"));
    QCommandLineOption markerMaxAreaOption(
        QStringList() << "marker-max-area", QStringLiteral("标记点最大面积"), QStringLiteral("px"));
    QCommandLineOption markerIntensityOption(
        QStringList() << "marker-intensity", QStringLiteral("标记点亮度上限"), QStringLiteral("0-255"));
    QCommandLineOption debscanDistOption(
        QStringList() << "debscan-dist", QStringLiteral("DBSCAN 去重像素距离"), QStringLiteral("px"));
    QCommandLineOption bootstrapTemplateOption(
        QStringList() << "bootstrap-template",
        QStringLiteral("用本次检测 3D 点生成临时模板并二次匹配（验证链路）"),
        QStringLiteral("path"));
    QCommandLineOption saveDebugImageOption(
        QStringList() << "save-debug-image",
        QStringLiteral("保存标注 2D 圆心的调试图"),
        QStringLiteral("path"));

    parser.addOption(groupOption);
    parser.addOption(imageOption);
    parser.addOption(plyOption);
    parser.addOption(cloudWidthOption);
    parser.addOption(cloudHeightOption);
    parser.addOption(templateOption);
    parser.addOption(textureFromPlyOption);
    parser.addOption(legacyZeroNaNOption);
    parser.addOption(allowResizeTextureOption);
    parser.addOption(minDistanceOption);
    parser.addOption(maxDistanceOption);
    parser.addOption(cosToleranceOption);
    parser.addOption(minPercentOption);
    parser.addOption(cloudRadiusOption);
    parser.addOption(markerMinAreaOption);
    parser.addOption(markerMaxAreaOption);
    parser.addOption(markerIntensityOption);
    parser.addOption(debscanDistOption);
    parser.addOption(bootstrapTemplateOption);
    parser.addOption(saveDebugImageOption);
    parser.addPositionalArgument(
        QStringLiteral("files"),
        QStringLiteral("可选: <image> <ply>（未使用 --group 时）"));
    parser.process(app);

    const bool textureFromPly = parser.isSet(textureFromPlyOption);
    const bool legacyZeroNaN = parser.isSet(legacyZeroNaNOption);
    const bool allowResizeTexture = parser.isSet(allowResizeTextureOption);

    QString imagePath = parser.value(imageOption);
    QString plyPath = parser.value(plyOption);
    if (parser.isSet(groupOption)) {
        QString error;
        if (!resolveGroupFiles(parser.value(groupOption), imagePath, plyPath, textureFromPly, error)) {
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

    if (plyPath.isEmpty()) {
        std::cerr << "用法:\n"
                  << "  scan_tracking_lbn_offline_runner --group testdata/group1\n"
                  << "    （默认 main.cpp：texture_aligned.bmp + 同尺寸 PLY）\n"
                  << "  scan_tracking_lbn_offline_runner --group testdata/group1 --allow-resize-texture\n"
                  << "  scan_tracking_lbn_offline_runner --group testdata/group1 --texture-from-ply\n";
        return 2;
    }

    if (!textureFromPly && imagePath.isEmpty()) {
        std::cerr << "请指定 -i 图像，或使用 --texture-from-ply 从 PLY 生成纹理\n";
        return 2;
    }

    const int forcedWidth = parser.value(cloudWidthOption).toInt();
    const int forcedHeight = parser.value(cloudHeightOption).toInt();

    if (!imagePath.isEmpty()) {
        std::cout << "图像: " << imagePath.toStdString() << '\n';
    }
    std::cout << "点云: " << plyPath.toStdString() << '\n';
    std::cout << "流程: " << (textureFromPly ? "扩展/PLY纹理" : (allowResizeTexture ? "扩展/缩放纹理" : "main.cpp 对齐纹理"))
              << '\n';
    std::cout << "PLY 无效点: " << (legacyZeroNaN ? "legacy 写 0,0,0" : "quiet_NaN（与 SDK/main 一致）") << '\n';

    const auto cloudLoad =
        loadOrganizedAsciiPly(plyPath, forcedWidth, forcedHeight, textureFromPly, legacyZeroNaN);
    if (!cloudLoad.error.isEmpty()) {
        std::cerr << cloudLoad.error.toStdString() << '\n';
        return 3;
    }

    std::cout << "点云网格: " << cloudLoad.frame.width << "x" << cloudLoad.frame.height
              << " valid=" << cloudLoad.validPoints << " nan=" << cloudLoad.nanPoints << '\n';

    scan_tracking::mech_eye::GrayTextureFrame texture;
    QString textureMessage;
    if (textureFromPly) {
        texture = cloudLoad.texture;
        textureMessage = QStringLiteral("纹理由 PLY 顶点 RGB 转灰度 %1x%2（与点云同网格）")
                             .arg(texture.width)
                             .arg(texture.height);
    } else {
        const bool textureOk = allowResizeTexture
            ? loadGrayTextureResized(
                  imagePath,
                  cloudLoad.frame.width,
                  cloudLoad.frame.height,
                  texture,
                  textureMessage)
            : loadGrayTextureMainCppStyle(
                  imagePath,
                  cloudLoad.frame.width,
                  cloudLoad.frame.height,
                  texture,
                  textureMessage);
        if (!textureOk) {
            std::cerr << textureMessage.toStdString() << '\n';
            return 4;
        }
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
    if (parser.isSet(minDistanceOption)) {
        lbnConfig.minDistance = parser.value(minDistanceOption).toFloat();
    }
    if (parser.isSet(maxDistanceOption)) {
        lbnConfig.maxDistance = parser.value(maxDistanceOption).toFloat();
    }
    if (parser.isSet(cosToleranceOption)) {
        lbnConfig.cosTolerance = parser.value(cosToleranceOption).toFloat();
    }
    if (parser.isSet(minPercentOption)) {
        lbnConfig.minPercent = parser.value(minPercentOption).toFloat();
    }
    if (parser.isSet(cloudRadiusOption)) {
        lbnConfig.cloudSearchRadiusPx = parser.value(cloudRadiusOption).toInt();
    }
    if (parser.isSet(markerMinAreaOption)) {
        lbnConfig.markerMinArea = parser.value(markerMinAreaOption).toInt();
    }
    if (parser.isSet(markerMaxAreaOption)) {
        lbnConfig.markerMaxArea = parser.value(markerMaxAreaOption).toInt();
    }
    if (parser.isSet(markerIntensityOption)) {
        lbnConfig.markerIntensityThreshold = parser.value(markerIntensityOption).toInt();
    }
    if (parser.isSet(debscanDistOption)) {
        lbnConfig.markerDebscanDistPx = parser.value(debscanDistOption).toFloat();
    }

    std::cout << "LBN 参数: minDist=" << lbnConfig.minDistance << " maxDist=" << lbnConfig.maxDistance
              << " cosTol=" << lbnConfig.cosTolerance << " minPct=" << lbnConfig.minPercent
              << " cloudR=" << lbnConfig.cloudSearchRadiusPx << " markerArea=["
              << lbnConfig.markerMinArea << "," << lbnConfig.markerMaxArea << "] intensity<="
              << lbnConfig.markerIntensityThreshold << " debscan=" << lbnConfig.markerDebscanDistPx
              << '\n';
    std::cout << "LBN 模板: " << lbnConfig.templateFile.toStdString() << '\n';
    std::cout << "调用 runLbnPoseDetection...\n";

    scan_tracking::vision::LbnPoseMarkerDebug markerDebug;
    const auto lbnResult =
        scan_tracking::vision::runLbnPoseDetection(capture, lbnConfig, &markerDebug);

    printMarkerDebug(markerDebug);

    if (parser.isSet(saveDebugImageOption) && !imagePath.isEmpty()) {
        const QString debugPath = parser.value(saveDebugImageOption);
        if (saveMarkerDebugImage(imagePath, debugPath, markerDebug)) {
            std::cout << "已保存调试图: " << debugPath.toStdString() << '\n';
        } else {
            std::cerr << "保存调试图失败: " << debugPath.toStdString() << '\n';
        }
    }

    std::cout << "invoked=" << lbnResult.invoked << '\n';
    std::cout << "success=" << lbnResult.success << '\n';
    std::cout << "message=" << lbnResult.message.toStdString() << '\n';
    std::cout << "matchedPointCount=" << lbnResult.matchedPointCount << '\n';

    if (lbnResult.poseMatrix.isValid()) {
        std::cout << "Rt 4x4:\n";
        printPoseMatrix(lbnResult.poseMatrix);
    }

    const bool wantBootstrap = parser.isSet(bootstrapTemplateOption);
    if (!lbnResult.success && wantBootstrap && markerDebug.centers3dCount >= 3) {
        QString bootstrapPath = parser.value(bootstrapTemplateOption);
        if (bootstrapPath.isEmpty()) {
            bootstrapPath = QDir(QFileInfo(plyPath).absolutePath())
                                .filePath(QStringLiteral("detected_template_bootstrap.txt"));
        }
        if (!writeCenters3dTemplate(bootstrapPath, markerDebug)) {
            std::cerr << "无法写入 bootstrap 模板: " << bootstrapPath.toStdString() << '\n';
            return 5;
        }
        std::cout << "主模板匹配失败，使用检测 3D 点 bootstrap 模板: "
                  << bootstrapPath.toStdString() << '\n';

        auto bootstrapConfig = lbnConfig;
        bootstrapConfig.templateFile = bootstrapPath;
        lbn_pose::Estimator bootstrapEstimator(toLbnCoreConfig(bootstrapConfig));
        const auto bootstrapResult =
            bootstrapEstimator.estimateFrom3d(markerDebugToPoints3d(markerDebug));

        std::cout << "bootstrap success=" << (bootstrapResult.success ? 1 : 0) << '\n';
        std::cout << "bootstrap message=" << bootstrapResult.message << '\n';
        std::cout << "bootstrap matchedPointCount=" << bootstrapResult.matchedPointCount << '\n';
        if (bootstrapResult.success && !bootstrapResult.rtGlobal.empty()) {
            scan_tracking::vision::PoseMatrix4x4 pose;
            float values[16] = {};
            if (lbn_pose::rtGlobalToRowMajor16(bootstrapResult.rtGlobal, values)) {
                for (int i = 0; i < 16; ++i) {
                    pose.values[static_cast<std::size_t>(i)] = values[i];
                }
                pose.valid = true;
                std::cout << "bootstrap Rt 4x4:\n";
                printPoseMatrix(pose);
            }
            return 0;
        }
    }

    return lbnResult.success ? 0 : 5;
}
