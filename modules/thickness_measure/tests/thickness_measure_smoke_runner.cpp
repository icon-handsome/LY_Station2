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
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QRegularExpression>
#include <QtCore/QString>

#include "scan_tracking/thickness_measure/thickness_measure_service.h"

using scan_tracking::thickness_measure::ThicknessConfig;
using scan_tracking::thickness_measure::ThicknessMeasureError;
using scan_tracking::thickness_measure::ThicknessMeasureService;
using scan_tracking::thickness_measure::ThicknessMethod;
using scan_tracking::thickness_measure::ThicknessPairMeasurement;

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

/// Same numeric params as tools/thickness_compare/baseline_A/thickness_config.json
ThicknessConfig MakeBaselineAConfig()
{
    ThicknessConfig config;
    config.preprocess.enableOutlierRemoval = false;  // SOR crashes on this toolchain for full sample
    config.preprocess.meanK = 10;
    config.preprocess.stddevMulThresh = 5.0;
    config.preprocess.enableVoxelDownsample = true;
    config.preprocess.leafSize = 1.0;

    config.icp.maxIterations = 100;
    config.icp.maxCorrespondenceDistance = 100.0;
    config.icp.transformationEpsilon = 0.001;
    config.icp.euclideanFitnessEpsilon = 0.001;

    config.thicknessMethod = ThicknessMethod::NearestBetweenSurfaces;

    config.axisPoint = {-40.3683, 160.9239, 1875.7435};
    config.axisDirection = {-0.11217465, 0.72954082, 0.67467552};
    config.templateFeaturePoints[0] = {-615.117615, -463.260071, 3423.801270};
    config.templateFeaturePoints[1] = {-608.145386, -499.079773, 3483.108154};
    return config;
}

QString DefaultDataDir()
{
    return QStringLiteral("D:/work/LY/第二工位测量源码/厚度测量/input");
}

QString DefaultOutPath()
{
    return QStringLiteral("D:/work/LY/IPC_Station2/tools/thickness_compare/baseline_B/smoke_B.json");
}

const char* MethodName(ThicknessMethod method)
{
    return method == ThicknessMethod::TangentPlaneProjection ? "tangent_plane_projection"
                                                             : "nearest_between_surfaces";
}

bool WriteSmokeJson(const QString& path,
                    const QString& innerPath,
                    const QString& outerPath,
                    size_t innerCount,
                    size_t outerCount,
                    const ThicknessPairMeasurement& r)
{
    QJsonObject obj;
    obj.insert(QStringLiteral("source"),
               QStringLiteral("IPC ThicknessMeasureService + third_party ThicknessMeasure.dll"));
    obj.insert(QStringLiteral("inner_scan"), innerPath);
    obj.insert(QStringLiteral("outer_scan"), outerPath);
    obj.insert(QStringLiteral("inner_point_count"), static_cast<qint64>(innerCount));
    obj.insert(QStringLiteral("outer_point_count"), static_cast<qint64>(outerCount));
    obj.insert(QStringLiteral("inner_icp_fitness_score"), r.innerIcpFitnessScore);
    obj.insert(QStringLiteral("outer_icp_fitness_score"), r.outerIcpFitnessScore);
    obj.insert(QStringLiteral("thickness_method"), QString::fromUtf8(MethodName(r.thicknessMethod)));
    obj.insert(QStringLiteral("thickness_mm"), r.thicknessMm);
    obj.insert(QStringLiteral("valid"), r.valid);

    QJsonObject n0;
    n0.insert(QStringLiteral("x"), r.nearestScanPoints[0].x);
    n0.insert(QStringLiteral("y"), r.nearestScanPoints[0].y);
    n0.insert(QStringLiteral("z"), r.nearestScanPoints[0].z);
    QJsonObject n1;
    n1.insert(QStringLiteral("x"), r.nearestScanPoints[1].x);
    n1.insert(QStringLiteral("y"), r.nearestScanPoints[1].y);
    n1.insert(QStringLiteral("z"), r.nearestScanPoints[1].z);
    QJsonArray nearest;
    nearest.append(n0);
    nearest.append(n1);
    obj.insert(QStringLiteral("nearest_scan_points"), nearest);

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
    const QString innerPath = QDir(dataDir).filePath(QStringLiteral("0_template_inner_sample.pcd"));
    const QString outerPath = QDir(dataDir).filePath(QStringLiteral("0_template_outer_sample.pcd"));

    std::printf("ThicknessMeasure IPC smoke (ThicknessMeasureService)\n");
    std::printf("Data dir: %s\n", qPrintable(dataDir));
    std::printf("Inner: %s\n", qPrintable(innerPath));
    std::printf("Outer: %s\n", qPrintable(outerPath));
    std::printf("Out:   %s\n", qPrintable(outPath));
    std::fflush(stdout);

    std::vector<float> innerXyz;
    std::vector<float> outerXyz;
    size_t nInner = 0;
    size_t nOuter = 0;
    QString loadError;
    if (!LoadXyzPcd(innerPath, &innerXyz, &nInner, &loadError) ||
        !LoadXyzPcd(outerPath, &outerXyz, &nOuter, &loadError)) {
        std::fprintf(stderr, "Load failed: %s\n", qPrintable(loadError));
        return 2;
    }
    std::printf("Loaded inner/outer points: %zu / %zu\n", nInner, nOuter);
    std::fflush(stdout);

    // Baseline A uses the same clouds as both templates and scans.
    ThicknessMeasureService service;
    ThicknessMeasureError error;
    const ThicknessConfig config = MakeBaselineAConfig();
    if (!service.initialize(
            config,
            innerXyz.data(),
            nInner,
            outerXyz.data(),
            nOuter,
            &error)) {
        std::fprintf(stderr, "initialize failed: %s (code=%d)\n",
                     qPrintable(error.message), error.statusCode);
        return 3;
    }
    std::printf("ThicknessMeasureService ready (memory templates, SOR=off)\n");
    std::fflush(stdout);

    ThicknessPairMeasurement pair;
    if (!service.measurePair(
            innerXyz.data(),
            nInner,
            outerXyz.data(),
            nOuter,
            &pair,
            &error) ||
        !pair.valid) {
        std::fprintf(stderr, "measurePair failed: %s (code=%d)\n",
                     qPrintable(error.message), error.statusCode);
        return 4;
    }

    std::printf("\n=== smoke_B result ===\n");
    std::printf("thickness_mm: %.12f\n", pair.thicknessMm);
    std::printf("inner_icp_fitness_score: %.12f\n", pair.innerIcpFitnessScore);
    std::printf("outer_icp_fitness_score: %.12f\n", pair.outerIcpFitnessScore);
    std::printf("thickness_method: %s\n", MethodName(pair.thicknessMethod));
    std::printf("valid: %d\n", pair.valid ? 1 : 0);
    std::printf("nearest_outer: %.6f %.6f %.6f\n",
                pair.nearestScanPoints[0].x,
                pair.nearestScanPoints[0].y,
                pair.nearestScanPoints[0].z);
    std::printf("nearest_inner: %.6f %.6f %.6f\n",
                pair.nearestScanPoints[1].x,
                pair.nearestScanPoints[1].y,
                pair.nearestScanPoints[1].z);
    std::fflush(stdout);

    if (!WriteSmokeJson(outPath, innerPath, outerPath, nInner, nOuter, pair)) {
        std::fprintf(stderr, "Failed to write: %s\n", qPrintable(outPath));
        return 5;
    }
    std::printf("Saved: %s\n", qPrintable(outPath));
    std::fflush(stdout);

    // Avoid shutdown-path PCL cloud dtor crash on this toolchain (same as algo park workaround).
    // Process exit is enough after numbers are flushed.
    std::fflush(stdout);
    std::fflush(stderr);
    std::_Exit(0);
}
