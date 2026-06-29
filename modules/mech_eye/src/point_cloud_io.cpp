#include "scan_tracking/mech_eye/point_cloud_io.h"

#include "scan_tracking/common/capture_cache_paths.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QLoggingCategory>

#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <vector>

Q_LOGGING_CATEGORY(LOG_POINT_CLOUD_IO, "mech_eye.point_cloud_io")

namespace scan_tracking::mech_eye {

namespace {

enum class PlyFormat {
    Unknown,
    Ascii,
    BinaryLittleEndian,
};

struct PlyHeader {
    PlyFormat format = PlyFormat::Unknown;
    int vertexCount = 0;
    bool hasNormals = false;
    int headerEndOffset = 0;
};

bool isFinitePoint(float x, float y, float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

bool parsePlyHeader(QIODevice* device, PlyHeader* header, QString* errorMessage)
{
    if (header == nullptr || device == nullptr) {
        return false;
    }

    const QByteArray firstLine = device->readLine();
    if (firstLine.trimmed() != "ply") {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("缺少 PLY 头");
        }
        return false;
    }

    PlyHeader parsed;
    bool inHeader = true;
    while (inHeader && !device->atEnd()) {
        const QByteArray rawLine = device->readLine();
        const QString line = QString::fromUtf8(rawLine).trimmed();
        if (line.startsWith(QStringLiteral("format "))) {
            if (line.contains(QStringLiteral("ascii"))) {
                parsed.format = PlyFormat::Ascii;
            } else if (line.contains(QStringLiteral("binary_little_endian"))) {
                parsed.format = PlyFormat::BinaryLittleEndian;
            }
        } else if (line.startsWith(QStringLiteral("element vertex"))) {
            parsed.vertexCount = line.section(QLatin1Char(' '), 2).toInt();
        } else if (line == QStringLiteral("property float nx")) {
            parsed.hasNormals = true;
        } else if (line == QStringLiteral("end_header")) {
            inHeader = false;
            parsed.headerEndOffset = static_cast<int>(device->pos());
        }
    }

    if (parsed.format == PlyFormat::Unknown || parsed.vertexCount <= 0) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("PLY 头无效");
        }
        return false;
    }

    *header = parsed;
    return true;
}

void writeBinaryFloat(std::ofstream& ofs, float value)
{
    ofs.write(reinterpret_cast<const char*>(&value), sizeof(float));
}

}  // namespace

QString defaultScanCacheDirectory()
{
    return scan_tracking::common::defaultCaptureCacheRoot();
}

QString buildSegmentPlyPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp)
{
    const QString baseDir = scan_tracking::common::captureCacheMech3DDir(configuredRoot);
    if (baseDir.isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote() << "无法创建 mech_3d 缓存目录";
        return QString();
    }

    const QString ts =
        timestamp.trimmed().isEmpty() ? scan_tracking::common::buildCaptureTimestamp() : timestamp;
    const QString fileName =
        QStringLiteral("segment_%1_task%2_%3.ply").arg(segmentIndex).arg(taskId).arg(ts);
    return QDir(baseDir).absoluteFilePath(fileName);
}

bool savePointCloudFrameToPly(const PointCloudFrame& frame, const QString& absolutePath)
{
    if (!frame.isValid() || absolutePath.trimmed().isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("savePointCloudFrameToPly：帧或路径无效");
        return false;
    }

    const auto& points = *frame.pointsXYZ;

    const int pointCount = frame.pointCount;
    const int availablePointCount = static_cast<int>(points.size() / 3);
    const int count = std::min(pointCount, availablePointCount);
    if (count <= 0) {
        qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("savePointCloudFrameToPly：无点可写");
        return false;
    }

    QFileInfo fileInfo(absolutePath);
    QDir().mkpath(fileInfo.absolutePath());

    std::ofstream ofs(absolutePath.toStdString(), std::ios::binary);
    if (!ofs.is_open()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("savePointCloudFrameToPly：无法打开") << absolutePath;
        return false;
    }

    ofs << "ply\nformat binary_little_endian 1.0\nelement vertex " << count
        << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";

    for (int index = 0; index < count; ++index) {
        const auto base = static_cast<std::size_t>(index * 3);
        writeBinaryFloat(ofs, points[base]);
        writeBinaryFloat(ofs, points[base + 1]);
        writeBinaryFloat(ofs, points[base + 2]);
    }

    ofs.close();

    qInfo(LOG_POINT_CLOUD_IO).noquote()
        << QStringLiteral("PLY(binary) 已保存：") << absolutePath
        << QStringLiteral(" points=") << count;

    return true;
}

bool loadPointCloudFrameFromPly(const QString& absolutePath, PointCloudFrame* outFrame)
{
    if (outFrame == nullptr || absolutePath.trimmed().isEmpty()) {
        return false;
    }

    QFile file(absolutePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("loadPointCloudFrameFromPly：无法打开") << absolutePath;
        return false;
    }

    PlyHeader header;
    QString headerError;
    if (!parsePlyHeader(&file, &header, &headerError)) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("loadPointCloudFrameFromPly：") << headerError << absolutePath;
        return false;
    }

    auto points = std::make_shared<std::vector<float>>();
    auto normals = std::make_shared<std::vector<float>>();
    points->reserve(static_cast<std::size_t>(header.vertexCount) * 3);
    if (header.hasNormals) {
        normals->reserve(static_cast<std::size_t>(header.vertexCount) * 3);
    }

    if (header.format == PlyFormat::BinaryLittleEndian) {
        const int floatsPerVertex = header.hasNormals ? 6 : 3;
        const qint64 bytesNeeded =
            static_cast<qint64>(header.vertexCount) * floatsPerVertex * static_cast<qint64>(sizeof(float));
        const QByteArray body = file.read(bytesNeeded);
        if (body.size() != bytesNeeded) {
            qWarning(LOG_POINT_CLOUD_IO).noquote()
                << QStringLiteral("loadPointCloudFrameFromPly：二进制体长度不足") << absolutePath;
            return false;
        }

        for (int index = 0; index < header.vertexCount; ++index) {
            const auto offset = static_cast<std::size_t>(index * floatsPerVertex * sizeof(float));
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            std::memcpy(&x, body.data() + offset, sizeof(float));
            std::memcpy(&y, body.data() + offset + sizeof(float), sizeof(float));
            std::memcpy(&z, body.data() + offset + 2 * sizeof(float), sizeof(float));

            points->push_back(x);
            points->push_back(y);
            points->push_back(z);

            if (header.hasNormals) {
                float nx = 0.0f;
                float ny = 0.0f;
                float nz = 1.0f;
                std::memcpy(&nx, body.data() + offset + 3 * sizeof(float), sizeof(float));
                std::memcpy(&ny, body.data() + offset + 4 * sizeof(float), sizeof(float));
                std::memcpy(&nz, body.data() + offset + 5 * sizeof(float), sizeof(float));
                normals->push_back(nx);
                normals->push_back(ny);
                normals->push_back(nz);
            }
        }
    } else {
        int loaded = 0;
        while (!file.atEnd() && loaded < header.vertexCount) {
            const QByteArray rawLine = file.readLine();
            const QString line = QString::fromUtf8(rawLine).trimmed();
            if (line.isEmpty()) {
                continue;
            }

            const QStringList tokens = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
            if (tokens.size() < 3) {
                continue;
            }

            const float x = tokens[0].toFloat();
            const float y = tokens[1].toFloat();
            const float z = tokens[2].toFloat();
            if (!isFinitePoint(x, y, z)) {
                continue;
            }

            points->push_back(x);
            points->push_back(y);
            points->push_back(z);

            if (header.hasNormals && tokens.size() >= 6) {
                normals->push_back(tokens[3].toFloat());
                normals->push_back(tokens[4].toFloat());
                normals->push_back(tokens[5].toFloat());
            } else if (header.hasNormals) {
                normals->push_back(0.0f);
                normals->push_back(0.0f);
                normals->push_back(1.0f);
            }

            ++loaded;
        }
    }

    file.close();

    if (points->empty()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("loadPointCloudFrameFromPly：无有效点") << absolutePath;
        return false;
    }

    const int pointCount = static_cast<int>(points->size() / 3);
    outFrame->pointsXYZ = std::move(points);
    if (header.hasNormals && static_cast<int>(normals->size()) == pointCount * 3) {
        outFrame->normalsXYZ = std::move(normals);
    } else {
        outFrame->normalsXYZ.reset();
    }
    outFrame->pointCount = pointCount;
    outFrame->width = pointCount;
    outFrame->height = 1;

    qInfo(LOG_POINT_CLOUD_IO).noquote()
        << QStringLiteral("PLY 已加载：") << absolutePath
        << QStringLiteral(" pointCount=") << pointCount
        << QStringLiteral(" hasNormals=") << outFrame->hasNormals();

    return true;
}

void releasePointCloudFrameBuffers(PointCloudFrame* frame)
{
    if (frame == nullptr) {
        return;
    }
    frame->pointsXYZ.reset();
    frame->normalsXYZ.reset();
}

QString buildSegmentMech2DPngPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp)
{
    const QString baseDir = scan_tracking::common::captureCacheMech2DDir(configuredRoot);
    if (baseDir.isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote() << "无法创建 mech_2d 缓存目录";
        return QString();
    }

    const QString ts =
        timestamp.trimmed().isEmpty() ? scan_tracking::common::buildCaptureTimestamp() : timestamp;
    const QString fileName =
        QStringLiteral("segment_%1_task%2_%3.png").arg(segmentIndex).arg(taskId).arg(ts);
    return QDir(baseDir).absoluteFilePath(fileName);
}

bool saveGrayTextureFrameToPng(const GrayTextureFrame& frame, const QString& absolutePath)
{
    if (!frame.isValid() || absolutePath.trimmed().isEmpty()) {
        return false;
    }

    QImage image(frame.width, frame.height, QImage::Format_Grayscale8);
    if (image.isNull()) {
        return false;
    }

    for (int row = 0; row < frame.height; ++row) {
        auto* scanLine = image.scanLine(row);
        const auto offset = static_cast<std::size_t>(row * frame.width);
        for (int col = 0; col < frame.width; ++col) {
            scanLine[col] = (*frame.pixels)[offset + static_cast<std::size_t>(col)];
        }
    }

    if (!image.save(absolutePath, "PNG")) {
        qWarning(LOG_POINT_CLOUD_IO).noquote() << QStringLiteral("saveGrayTextureFrameToPng 失败：") << absolutePath;
        return false;
    }

    qInfo(LOG_POINT_CLOUD_IO).noquote()
        << QStringLiteral("Mech 2D PNG 已保存：") << absolutePath << frame.width << QStringLiteral("x") << frame.height;
    return true;
}

}  // namespace scan_tracking::mech_eye
