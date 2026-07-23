#include <algorithm>
#include <cstddef>
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
#include <QtCore/QRegularExpression>
#include <QtCore/QString>

#include "scan_tracking/length_volume_measure/length_volume_measure_service.h"

using scan_tracking::length_volume_measure::LengthVolumeConfig;
using scan_tracking::length_volume_measure::LengthVolumeMeasureError;
using scan_tracking::length_volume_measure::LengthVolumeMeasureService;
using scan_tracking::length_volume_measure::LengthVolumeMeasurement;

namespace {

struct PcdField {
    QString name;
    QString type;  // F/U/I
    int size = 4;
    int count = 1;
};

int FieldByteSize(const PcdField& field)
{
    return field.size * field.count;
}

float ReadFloatField(const char* base, const PcdField& field)
{
    if (field.type == QLatin1String("F") && field.size == 4) {
        float value = 0.0f;
        std::memcpy(&value, base, sizeof(float));
        return value;
    }
    if (field.type == QLatin1String("F") && field.size == 8) {
        double value = 0.0;
        std::memcpy(&value, base, sizeof(double));
        return static_cast<float>(value);
    }
    return 0.0f;
}

bool LoadXyzPcd(const QString& path, std::vector<float>* xyz, size_t* count, QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) {
            *error = QStringLiteral("Cannot open: %1").arg(path);
        }
        return false;
    }

    std::vector<PcdField> fields;
    QString dataType = QStringLiteral("ascii");
    qint64 points = 0;
    qint64 width = 0;

    while (!file.atEnd()) {
        const QByteArray raw = file.readLine();
        QString line = QString::fromLatin1(raw).trimmed();
        if (!line.isEmpty() && line.endsWith(QLatin1Char('\r'))) {
            line.chop(1);
        }
        if (line.isEmpty()) {
            continue;
        }

        if (line.startsWith(QLatin1String("FIELDS"), Qt::CaseInsensitive)) {
            fields.clear();
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            for (int i = 1; i < parts.size(); ++i) {
                PcdField field;
                field.name = parts[i];
                fields.push_back(field);
            }
        } else if (line.startsWith(QLatin1String("SIZE"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            for (int i = 1; i < parts.size() && (i - 1) < static_cast<int>(fields.size()); ++i) {
                fields[static_cast<size_t>(i - 1)].size = parts[i].toInt();
            }
        } else if (line.startsWith(QLatin1String("TYPE"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            for (int i = 1; i < parts.size() && (i - 1) < static_cast<int>(fields.size()); ++i) {
                fields[static_cast<size_t>(i - 1)].type = parts[i];
            }
        } else if (line.startsWith(QLatin1String("COUNT"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            for (int i = 1; i < parts.size() && (i - 1) < static_cast<int>(fields.size()); ++i) {
                fields[static_cast<size_t>(i - 1)].count = parts[i].toInt();
            }
        } else if (line.startsWith(QLatin1String("WIDTH"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            if (parts.size() >= 2) {
                width = parts[1].toLongLong();
            }
        } else if (line.startsWith(QLatin1String("POINTS"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            if (parts.size() >= 2) {
                points = parts[1].toLongLong();
            }
        } else if (line.startsWith(QLatin1String("DATA"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            if (parts.size() >= 2) {
                dataType = parts[1].toLower();
            }
            break;
        }
    }

    size_t xIndex = static_cast<size_t>(-1);
    size_t yIndex = static_cast<size_t>(-1);
    size_t zIndex = static_cast<size_t>(-1);
    for (size_t i = 0; i < fields.size(); ++i) {
        if (fields[i].name == QLatin1String("x")) {
            xIndex = i;
        } else if (fields[i].name == QLatin1String("y")) {
            yIndex = i;
        } else if (fields[i].name == QLatin1String("z")) {
            zIndex = i;
        }
    }
    if (xIndex == static_cast<size_t>(-1) || yIndex == static_cast<size_t>(-1) ||
        zIndex == static_cast<size_t>(-1)) {
        if (error) {
            *error = QStringLiteral("PCD missing x/y/z: %1").arg(path);
        }
        return false;
    }

    if (points == 0) {
        points = width;
    }
    if (points <= 0) {
        if (error) {
            *error = QStringLiteral("PCD point count is zero: %1").arg(path);
        }
        return false;
    }

    xyz->clear();
    xyz->reserve(static_cast<size_t>(points) * 3);

    if (dataType == QLatin1String("ascii")) {
        for (qint64 i = 0; i < points; ++i) {
            if (file.atEnd()) {
                break;
            }
            QString line = QString::fromLatin1(file.readLine()).trimmed();
            if (line.isEmpty()) {
                --i;
                continue;
            }
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            if (parts.size() <= static_cast<int>(std::max({xIndex, yIndex, zIndex}))) {
                continue;
            }
            bool okX = false;
            bool okY = false;
            bool okZ = false;
            const float x = parts[static_cast<int>(xIndex)].toFloat(&okX);
            const float y = parts[static_cast<int>(yIndex)].toFloat(&okY);
            const float z = parts[static_cast<int>(zIndex)].toFloat(&okZ);
            if (!okX || !okY || !okZ) {
                continue;
            }
            xyz->push_back(x);
            xyz->push_back(y);
            xyz->push_back(z);
        }
    } else if (dataType == QLatin1String("binary")) {
        int pointStep = 0;
        std::vector<int> offsets(fields.size(), 0);
        for (size_t i = 0; i < fields.size(); ++i) {
            offsets[i] = pointStep;
            pointStep += FieldByteSize(fields[i]);
        }
        std::vector<char> buffer(static_cast<size_t>(pointStep));
        for (qint64 i = 0; i < points; ++i) {
            if (file.read(buffer.data(), pointStep) != pointStep) {
                if (error) {
                    *error = QStringLiteral("Unexpected EOF in binary PCD: %1").arg(path);
                }
                return false;
            }
            const float x = ReadFloatField(buffer.data() + offsets[xIndex], fields[xIndex]);
            const float y = ReadFloatField(buffer.data() + offsets[yIndex], fields[yIndex]);
            const float z = ReadFloatField(buffer.data() + offsets[zIndex], fields[zIndex]);
            xyz->push_back(x);
            xyz->push_back(y);
            xyz->push_back(z);
        }
    } else {
        if (error) {
            *error = QStringLiteral("Unsupported PCD DATA type: %1").arg(path);
        }
        return false;
    }

    *count = xyz->size() / 3;
    if (*count == 0) {
        if (error) {
            *error = QStringLiteral("No points loaded: %1").arg(path);
        }
        return false;
    }
    return true;
}

/// Same numeric params as tools/length_volume_compare/baseline_A/config.ini
LengthVolumeConfig MakeBaselineAConfig()
{
    LengthVolumeConfig config;
    config.cropInputCloud = false;

    config.axisDirectionX = 0.0f;
    config.axisDirectionY = 0.0f;
    config.axisDirectionZ = 1.0f;
    config.endPoint1X = -24.047901f;
    config.endPoint1Y = 599.578979f;
    config.endPoint1Z = 3.182239f;
    config.endPoint2X = -50.648342f;
    config.endPoint2Y = 597.784058f;
    config.endPoint2Z = 2197.218262f;

    config.voxelSize = 3.0f;
    config.outlierK = 1;  // SOR off — same as baseline A
    config.outlierStd = 5.0;

    config.icpMaxIterations = 80;
    config.icpMaxCorrespondenceDistance = 100.0f;
    config.icpTransformationEpsilon = 1e-8;
    config.icpEuclideanFitnessEpsilon = 1e-6;

    config.updateCylinderAxis = false;
    config.cylinderFitIterations = 3;
    config.cylinderInlierBand = 0.0f;

    config.normalK = 15;
    config.endNormalMinAbsDot = 0.90;
    config.endpointDetectionMethod = QStringLiteral("outsideScan");
    config.axialBinWidth = 2.0f;
    config.endSearchHalfWidth = 80.0f;
    config.outsideScanPeakSearchWidth = 20.0f;
    config.minPointsPerBin = 10;
    config.minConsecutiveInsideBins = 3;
    config.outsideCheckBins = 2;
    config.refineHalfWidth = 6.0f;
    config.refineEdgePercentile = 0.0001;
    config.endpointMaxRadius = 0.0f;
    return config;
}

QString DefaultDataDir()
{
    return QStringLiteral("D:/work/LY/第二工位测量源码/长度和容积测量/Data");
}

QString DefaultOutPath()
{
    return QStringLiteral("D:/work/LY/IPC_Station2/tools/length_volume_compare/baseline_B/smoke_B.json");
}

bool WriteSmokeJson(const QString& path,
                    const QString& scanPath,
                    const QString& templatePath,
                    size_t scanCount,
                    size_t templateCount,
                    double volumeRadiusMm,
                    const LengthVolumeMeasurement& r)
{
    QJsonObject obj;
    obj.insert(QStringLiteral("source"),
               QStringLiteral("IPC LengthVolumeMeasureService + third_party LengthVolumeMeasure.dll"));
    obj.insert(QStringLiteral("scan"), scanPath);
    obj.insert(QStringLiteral("template"), templatePath);
    obj.insert(QStringLiteral("scan_point_count"), static_cast<qint64>(scanCount));
    obj.insert(QStringLiteral("template_point_count"), static_cast<qint64>(templateCount));
    obj.insert(QStringLiteral("volume_radius_mm"), volumeRadiusMm);
    obj.insert(QStringLiteral("length_mm"), r.lengthMm);
    obj.insert(QStringLiteral("volume_liters"), r.volumeLiters);
    obj.insert(QStringLiteral("fitted_outer_radius_mm"), r.fittedOuterRadiusMm);
    obj.insert(QStringLiteral("icp_fitness_score"), r.icpFitnessScore);
    obj.insert(QStringLiteral("icp_converged"), r.icpConverged);
    obj.insert(QStringLiteral("valid"), r.valid);

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

    const QString dataDir = (argc >= 2) ? QString::fromLocal8Bit(argv[1]) : DefaultDataDir();
    const QString outPath = (argc >= 3) ? QString::fromLocal8Bit(argv[2]) : DefaultOutPath();
    const double volumeRadiusMm = (argc >= 4) ? QString::fromLocal8Bit(argv[3]).toDouble() : 600.0;
    const QString scanPath = QDir(dataDir).filePath(QStringLiteral("sample_cylinder_cut.pcd"));
    const QString templatePath = QDir(dataDir).filePath(QStringLiteral("sample_cylinder.pcd"));

    std::printf("LengthVolumeMeasure IPC smoke (LengthVolumeMeasureService)\n");
    std::printf("Data dir: %s\n", qPrintable(dataDir));
    std::printf("Scan: %s\n", qPrintable(scanPath));
    std::printf("Template: %s\n", qPrintable(templatePath));
    std::printf("Out: %s\n", qPrintable(outPath));
    std::printf("volume_radius_mm: %.6f\n", volumeRadiusMm);
    std::fflush(stdout);

    std::vector<float> scanXyz;
    std::vector<float> templateXyz;
    size_t nScan = 0;
    size_t nTemplate = 0;
    QString loadError;
    if (!LoadXyzPcd(scanPath, &scanXyz, &nScan, &loadError) ||
        !LoadXyzPcd(templatePath, &templateXyz, &nTemplate, &loadError)) {
        std::fprintf(stderr, "Load failed: %s\n", qPrintable(loadError));
        return 2;
    }
    std::printf("Loaded scan/template points: %zu / %zu\n", nScan, nTemplate);
    std::fflush(stdout);

    LengthVolumeMeasureService service;
    LengthVolumeMeasureError error;
    const LengthVolumeConfig config = MakeBaselineAConfig();
    if (!service.initialize(config, templateXyz.data(), nTemplate, &error)) {
        std::fprintf(stderr, "initialize failed: %s (code=%d)\n",
                     qPrintable(error.message), error.statusCode);
        return 3;
    }
    std::printf("LengthVolumeMeasureService ready (memory template, SOR=off)\n");
    std::fflush(stdout);

    LengthVolumeMeasurement result;
    if (!service.measure(scanXyz.data(), nScan, volumeRadiusMm, &result, &error) || !result.valid) {
        std::fprintf(stderr, "measure failed: %s (code=%d)\n",
                     qPrintable(error.message), error.statusCode);
        return 4;
    }

    std::printf("\n=== smoke_B result ===\n");
    std::printf("length_mm: %.12f\n", result.lengthMm);
    std::printf("volume_liters: %.12f\n", result.volumeLiters);
    std::printf("volume_radius_mm: %.12f\n", result.volumeRadiusMm);
    std::printf("fitted_outer_radius_mm: %.12f\n", result.fittedOuterRadiusMm);
    std::printf("icp_fitness_score: %.12f\n", result.icpFitnessScore);
    std::printf("icp_converged: %d\n", result.icpConverged ? 1 : 0);
    std::printf("valid: %d\n", result.valid ? 1 : 0);
    std::fflush(stdout);

    if (!WriteSmokeJson(outPath, scanPath, templatePath, nScan, nTemplate, volumeRadiusMm, result)) {
        std::fprintf(stderr, "Failed to write: %s\n", qPrintable(outPath));
        return 5;
    }
    std::printf("Saved: %s\n", qPrintable(outPath));
    std::fflush(stdout);

    // Avoid shutdown-path PCL cloud dtor crash on this toolchain.
    std::fflush(stdout);
    std::fflush(stderr);
    std::_Exit(0);
}
