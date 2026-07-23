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
    bool hasRgb = false;
    int headerEndOffset = 0;
    /// 每个顶点二进制字节数（仅 binary）；未知时为 0
    int bytesPerVertex = 0;
};

bool isFinitePoint(float x, float y, float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

int propertyByteSize(const QString& typeToken)
{
    if (typeToken == QLatin1String("float") || typeToken == QLatin1String("float32")
        || typeToken == QLatin1String("int") || typeToken == QLatin1String("int32")
        || typeToken == QLatin1String("uint") || typeToken == QLatin1String("uint32")) {
        return 4;
    }
    if (typeToken == QLatin1String("double") || typeToken == QLatin1String("float64")) {
        return 8;
    }
    if (typeToken == QLatin1String("uchar") || typeToken == QLatin1String("uint8")
        || typeToken == QLatin1String("char") || typeToken == QLatin1String("int8")) {
        return 1;
    }
    if (typeToken == QLatin1String("ushort") || typeToken == QLatin1String("uint16")
        || typeToken == QLatin1String("short") || typeToken == QLatin1String("int16")) {
        return 2;
    }
    return 0;
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
    bool inVertexElement = false;
    while (inHeader && !device->atEnd()) {
        const QByteArray rawLine = device->readLine();
        const QString line = QString::fromUtf8(rawLine).trimmed();
        if (line.startsWith(QStringLiteral("format "))) {
            if (line.contains(QStringLiteral("ascii"))) {
                parsed.format = PlyFormat::Ascii;
            } else if (line.contains(QStringLiteral("binary_little_endian"))) {
                parsed.format = PlyFormat::BinaryLittleEndian;
            }
        } else if (line.startsWith(QStringLiteral("element "))) {
            inVertexElement = line.startsWith(QStringLiteral("element vertex"));
            if (inVertexElement) {
                parsed.vertexCount = line.section(QLatin1Char(' '), 2).toInt();
            }
        } else if (inVertexElement && line.startsWith(QStringLiteral("property "))) {
            const QStringList tokens = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
            if (tokens.size() >= 3) {
                const int size = propertyByteSize(tokens[1]);
                if (size > 0) {
                    parsed.bytesPerVertex += size;
                }
                if (tokens[2] == QLatin1String("nx")) {
                    parsed.hasNormals = true;
                } else if (tokens[2] == QLatin1String("red") || tokens[2] == QLatin1String("r")) {
                    parsed.hasRgb = true;
                }
            }
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

    // 兼容旧写法：未统计到 property 时按 xyz / xyz+nxny nz 回退
    if (parsed.bytesPerVertex <= 0) {
        parsed.bytesPerVertex = parsed.hasNormals ? 24 : 12;
        if (parsed.hasRgb) {
            parsed.bytesPerVertex += 3;
        }
    }

    *header = parsed;
    return true;
}

void writeBinaryFloat(std::ofstream& ofs, float value)
{
    ofs.write(reinterpret_cast<const char*>(&value), sizeof(float));
}

void writeBinaryUChar(std::ofstream& ofs, uint8_t value)
{
    ofs.write(reinterpret_cast<const char*>(&value), sizeof(uint8_t));
}

}  // namespace

QString defaultScanCacheDirectory()
{
    return scan_tracking::common::defaultCaptureCacheRoot();
}

QString buildSegmentPlyPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex)
{
    const QString pointDir = scan_tracking::common::capturePointDirectory(
        configuredRoot, pathId, deviceTag, segmentIndex);
    if (pointDir.isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("无法创建点位目录 pathId=") << pathId
            << QStringLiteral(" device=") << deviceTag
            << QStringLiteral(" point=") << segmentIndex;
        return QString();
    }
    return QDir(pointDir).absoluteFilePath(QStringLiteral("cloud.ply"));
}

bool savePointCloudFrameToPly(
    const PointCloudFrame& frame,
    const QString& absolutePath,
    const GrayTextureFrame* texture)
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

    const uint8_t* texturePixels = nullptr;
    int texturePixelCount = 0;
    if (texture != nullptr && texture->isValid()) {
        texturePixels = texture->pixels->data();
        texturePixelCount = static_cast<int>(texture->pixels->size());
        if (texturePixelCount < count) {
            qWarning(LOG_POINT_CLOUD_IO).noquote()
                << QStringLiteral("savePointCloudFrameToPly：纹理像素不足，RGB 将写 0")
                << QStringLiteral(" texturePixels=") << texturePixelCount
                << QStringLiteral(" points=") << count;
            texturePixels = nullptr;
        }
    }

    QFileInfo fileInfo(absolutePath);
    QDir().mkpath(fileInfo.absolutePath());

    std::ofstream ofs(absolutePath.toStdString(), std::ios::binary);
    if (!ofs.is_open()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("savePointCloudFrameToPly：无法打开") << absolutePath;
        return false;
    }

    ofs << "ply\n"
        << "format binary_little_endian 1.0\n"
        << "element vertex " << count << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n"
        << "property uchar red\n"
        << "property uchar green\n"
        << "property uchar blue\n"
        << "end_header\n";

    for (int index = 0; index < count; ++index) {
        const auto base = static_cast<std::size_t>(index * 3);
        writeBinaryFloat(ofs, points[base]);
        writeBinaryFloat(ofs, points[base + 1]);
        writeBinaryFloat(ofs, points[base + 2]);

        uint8_t gray = 0;
        if (texturePixels != nullptr) {
            gray = texturePixels[static_cast<std::size_t>(index)];
        }
        writeBinaryUChar(ofs, gray);
        writeBinaryUChar(ofs, gray);
        writeBinaryUChar(ofs, gray);
    }

    ofs.close();

    qInfo(LOG_POINT_CLOUD_IO).noquote()
        << QStringLiteral("PLY(binary xyz+rgb) 已保存：") << absolutePath
        << QStringLiteral(" points=") << count
        << QStringLiteral(" textured=") << (texturePixels != nullptr);

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
        const qint64 bytesNeeded =
            static_cast<qint64>(header.vertexCount) * header.bytesPerVertex;
        const QByteArray body = file.read(bytesNeeded);
        if (body.size() != bytesNeeded) {
            qWarning(LOG_POINT_CLOUD_IO).noquote()
                << QStringLiteral("loadPointCloudFrameFromPly：二进制体长度不足") << absolutePath;
            return false;
        }

        for (int index = 0; index < header.vertexCount; ++index) {
            const auto offset = static_cast<std::size_t>(index * header.bytesPerVertex);
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            std::memcpy(&x, body.data() + offset, sizeof(float));
            std::memcpy(&y, body.data() + offset + sizeof(float), sizeof(float));
            std::memcpy(&z, body.data() + offset + 2 * sizeof(float), sizeof(float));

            points->push_back(x);
            points->push_back(y);
            points->push_back(z);

            if (header.hasNormals && header.bytesPerVertex >= 24) {
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
        << QStringLiteral(" hasRgb=") << header.hasRgb
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

QString buildSegmentMechTexturePngPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex)
{
    const QString pointDir = scan_tracking::common::capturePointDirectory(
        configuredRoot, pathId, deviceTag, segmentIndex);
    if (pointDir.isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("无法创建点位目录(texture) pathId=") << pathId
            << QStringLiteral(" device=") << deviceTag
            << QStringLiteral(" point=") << segmentIndex;
        return QString();
    }
    return QDir(pointDir).absoluteFilePath(QStringLiteral("texture.png"));
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
        << QStringLiteral("Mech 纹理 PNG 已保存：") << absolutePath << frame.width << QStringLiteral("x") << frame.height;
    return true;
}

}  // namespace scan_tracking::mech_eye
