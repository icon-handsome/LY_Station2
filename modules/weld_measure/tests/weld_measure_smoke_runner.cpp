#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QString>
#include <QtCore/QTextStream>

#include "scan_tracking/weld_measure/weld_measure_service.h"

using scan_tracking::weld_measure::WeldFrameMeasurement;
using scan_tracking::weld_measure::WeldMeasureError;
using scan_tracking::weld_measure::WeldMeasureService;

namespace {

QString DefaultIniPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/weld_measure/weld_measurement-arm.ini"));
}

QString DefaultScanPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/weld_measure/Data/path1/Scan_Path1_Arm_cloud_1.pcd"));
}

QString DefaultOutPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("weld_measure_smoke_frame.json"));
}

bool LoadTextXyz(const QString& path, std::vector<float>* xyz, size_t* count, QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error) {
            *error = QStringLiteral("Cannot open: %1").arg(path);
        }
        return false;
    }

    xyz->clear();
    QTextStream in(&file);
    in.setCodec("UTF-8");
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty()) {
            continue;
        }
        line.replace(QLatin1Char(','), QLatin1Char(' '));
        const QStringList parts = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
        if (parts.size() < 3) {
            continue;
        }
        bool okX = false;
        bool okY = false;
        bool okZ = false;
        const float x = parts[0].toFloat(&okX);
        const float y = parts[1].toFloat(&okY);
        const float z = parts[2].toFloat(&okZ);
        if (!okX || !okY || !okZ || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }
        xyz->push_back(x);
        xyz->push_back(y);
        xyz->push_back(z);
    }

    *count = xyz->size() / 3u;
    if (*count == 0) {
        if (error) {
            *error = QStringLiteral("Text point cloud empty: %1").arg(path);
        }
        return false;
    }
    return true;
}

bool LoadBinaryPcdXyz(
    const QString& path,
    std::vector<float>* xyz,
    size_t* count,
    size_t maxKeep,
    QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) {
            *error = QStringLiteral("Cannot open PCD: %1").arg(path);
        }
        return false;
    }

    QByteArray fields;
    QByteArray sizes;
    QByteArray countsLine;
    qint64 points = 0;
    bool binary = false;
    while (!file.atEnd()) {
        const QByteArray line = file.readLine();
        if (line.startsWith("FIELDS ")) {
            fields = line.mid(7).trimmed();
        } else if (line.startsWith("SIZE ")) {
            sizes = line.mid(5).trimmed();
        } else if (line.startsWith("COUNT ")) {
            countsLine = line.mid(6).trimmed();
        } else if (line.startsWith("POINTS ")) {
            points = line.mid(7).trimmed().toLongLong();
        } else if (line.startsWith("DATA binary")) {
            binary = true;
            break;
        } else if (line.startsWith("DATA ascii")) {
            binary = false;
            break;
        }
    }

    if (!binary || points <= 0) {
        // Fallback: text xyz rows.
        file.close();
        return LoadTextXyz(path, xyz, count, error);
    }

    const QList<QByteArray> fieldTok = fields.split(' ');
    const QList<QByteArray> sizeTok = sizes.split(' ');
    const QList<QByteArray> countTok = countsLine.split(' ');
    if (fieldTok.size() == 0 || fieldTok.size() != sizeTok.size() || fieldTok.size() != countTok.size()) {
        if (error) {
            *error = QStringLiteral("Invalid PCD header field layout: %1").arg(path);
        }
        return false;
    }

    size_t pointStep = 0;
    int xOff = -1;
    int yOff = -1;
    int zOff = -1;
    for (int i = 0; i < fieldTok.size(); ++i) {
        const int sz = sizeTok[i].toInt();
        const int cnt = countTok[i].toInt();
        if (fieldTok[i] == "x") {
            xOff = static_cast<int>(pointStep);
        } else if (fieldTok[i] == "y") {
            yOff = static_cast<int>(pointStep);
        } else if (fieldTok[i] == "z") {
            zOff = static_cast<int>(pointStep);
        }
        pointStep += static_cast<size_t>(sz) * static_cast<size_t>(cnt);
    }
    if (xOff < 0 || yOff < 0 || zOff < 0 || pointStep == 0) {
        if (error) {
            *error = QStringLiteral("PCD missing xyz offsets: %1").arg(path);
        }
        return false;
    }

    size_t stride = 1;
    size_t keep = static_cast<size_t>(points);
    if (maxKeep > 0 && keep > maxKeep) {
        stride = (keep + maxKeep - 1) / maxKeep;
        keep = (keep + stride - 1) / stride;
    }

    xyz->clear();
    xyz->reserve(keep * 3);
    std::vector<char> buf(pointStep);
    size_t n = 0;
    for (qint64 i = 0; i < points; ++i) {
        if (file.read(buf.data(), static_cast<qint64>(pointStep)) != static_cast<qint64>(pointStep)) {
            break;
        }
        if (static_cast<size_t>(i) % stride != 0) {
            continue;
        }
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        std::memcpy(&x, buf.data() + xOff, sizeof(float));
        std::memcpy(&y, buf.data() + yOff, sizeof(float));
        std::memcpy(&z, buf.data() + zOff, sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }
        xyz->push_back(x);
        xyz->push_back(y);
        xyz->push_back(z);
        ++n;
        if (n >= keep) {
            break;
        }
    }

    *count = n;
    if (n == 0) {
        if (error) {
            *error = QStringLiteral("PCD yielded no points: %1").arg(path);
        }
        return false;
    }
    return true;
}

bool WriteSmokeJson(
    const QString& path,
    const QString& cloudPath,
    const QString& configPath,
    size_t pointCount,
    int frameIndex,
    const WeldFrameMeasurement& r)
{
    QJsonObject obj;
    obj.insert(QStringLiteral("source"),
               QStringLiteral("IPC WeldMeasureService V2.0 measureFrame"));
    obj.insert(QStringLiteral("input_cloud"), cloudPath);
    obj.insert(QStringLiteral("config"), configPath);
    obj.insert(QStringLiteral("frame_index"), frameIndex);
    obj.insert(QStringLiteral("loaded_points"), static_cast<qint64>(pointCount));
    obj.insert(QStringLiteral("valid_sections"), r.validSections);
    obj.insert(QStringLiteral("total_sections"), r.totalSections);
    obj.insert(QStringLiteral("mismatch_mm"), r.average.mismatchMm);
    obj.insert(QStringLiteral("reinforcement_mm"), r.average.reinforcementMm);
    obj.insert(QStringLiteral("angularity_mm"), r.average.angularityMm);
    obj.insert(QStringLiteral("included_angle_rad"), r.average.includedAngleRad);
    obj.insert(QStringLiteral("left_undercut_mm"), r.average.leftUndercutMm);
    obj.insert(QStringLiteral("right_undercut_mm"), r.average.rightUndercutMm);
    obj.insert(QStringLiteral("max_undercut_mm"), r.average.maxUndercutMm);
    obj.insert(QStringLiteral("left_undercut_length_mm"), r.leftUndercutLengthMm);
    obj.insert(QStringLiteral("right_undercut_length_mm"), r.rightUndercutLengthMm);
    obj.insert(QStringLiteral("valid"), r.average.valid);

    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return false;
    }
    file.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
    return true;
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    // Usage:
    //   smoke [ini] [scan.pcd|txt] [out.json] [frameIndex]
    const QString iniPath = (argc >= 2) ? QString::fromLocal8Bit(argv[1]) : DefaultIniPath();
    const QString cloudPath = (argc >= 3) ? QString::fromLocal8Bit(argv[2]) : DefaultScanPath();
    const QString outPath = (argc >= 4) ? QString::fromLocal8Bit(argv[3]) : DefaultOutPath();
    const int frameIndex = (argc >= 5) ? QString::fromLocal8Bit(argv[4]).toInt() : 1;

    std::printf("WeldMeasure IPC smoke (measureFrame / V2.0)\n");
    std::printf("Ini: %s\n", qPrintable(iniPath));
    std::printf("Cloud: %s\n", qPrintable(cloudPath));
    std::printf("Out: %s\n", qPrintable(outPath));
    std::printf("Frame: %d\n", frameIndex);
    std::fflush(stdout);

    std::vector<float> xyz;
    size_t pointCount = 0;
    QString loadError;
    const bool loaded = cloudPath.endsWith(QStringLiteral(".pcd"), Qt::CaseInsensitive)
        ? LoadBinaryPcdXyz(cloudPath, &xyz, &pointCount, 400000, &loadError)
        : LoadTextXyz(cloudPath, &xyz, &pointCount, &loadError);
    if (!loaded) {
        std::fprintf(stderr, "Load failed: %s\n", qPrintable(loadError));
        return 2;
    }
    std::printf("Loaded points: %zu\n", pointCount);
    std::fflush(stdout);

    WeldMeasureService service;
    WeldMeasureError error;
    if (!service.initializeFromIni(iniPath, &error)) {
        std::fprintf(stderr, "initializeFromIni failed: %s (code=%d)\n",
                     qPrintable(error.message), error.statusCode);
        return 3;
    }
    std::printf("Config: %s\n", qPrintable(service.configPath()));
    std::printf("Model: %s\n", qPrintable(service.modelPath()));
    std::fflush(stdout);

    WeldFrameMeasurement frame;
    if (!service.measureFrame(frameIndex, xyz.data(), pointCount, &frame, &error)) {
        std::fprintf(stderr, "measureFrame failed: %s (code=%d)\n",
                     qPrintable(error.message), error.statusCode);
        return 4;
    }
    if (!frame.average.valid || frame.validSections <= 0) {
        std::fprintf(stderr, "measureFrame returned no valid sections (valid=%d sections=%d)\n",
                     frame.average.valid ? 1 : 0, frame.validSections);
        return 5;
    }

    std::printf("\n=== measureFrame result ===\n");
    std::printf("valid_sections: %d/%d\n", frame.validSections, frame.totalSections);
    std::printf("mismatch_mm: %.6f\n", frame.average.mismatchMm);
    std::printf("reinforcement_mm: %.6f\n", frame.average.reinforcementMm);
    std::printf("angularity_mm: %.6f\n", frame.average.angularityMm);
    std::printf("included_angle_deg: %.6f\n",
                frame.average.includedAngleRad * 180.0 / 3.14159265358979323846);
    std::printf("left/right undercut depth: %.6f / %.6f\n",
                frame.average.leftUndercutMm, frame.average.rightUndercutMm);
    std::printf("left/right undercut length: %.6f / %.6f\n",
                frame.leftUndercutLengthMm, frame.rightUndercutLengthMm);
    std::fflush(stdout);

    if (!WriteSmokeJson(outPath, cloudPath, service.configPath(), pointCount, frameIndex, frame)) {
        std::fprintf(stderr, "Failed to write: %s\n", qPrintable(outPath));
        return 6;
    }
    std::printf("Saved: %s\n", qPrintable(outPath));
    return 0;
}
