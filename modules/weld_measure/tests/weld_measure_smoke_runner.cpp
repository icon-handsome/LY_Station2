#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QString>
#include <QtCore/QTextStream>

#include "scan_tracking/weld_measure/weld_measure_service.h"

using scan_tracking::weld_measure::WeldMeasureError;
using scan_tracking::weld_measure::WeldMeasureService;
using scan_tracking::weld_measure::WeldSectionMeasurement;

namespace {

QString DefaultDataDir()
{
    // Same sample as baseline_A; ASCII path under tools/weld_compare/data.
    return QStringLiteral("D:/work/LY/IPC_Station2/tools/weld_compare/data/1_000000.txt");
}

QString DefaultOutPath()
{
    return QStringLiteral("D:/work/LY/IPC_Station2/tools/weld_compare/baseline_B/smoke_B.json");
}

bool LoadTextXyz(const QString& path, std::vector<float>* xyz, size_t* count, QString* error)
{
    if (xyz == nullptr || count == nullptr) {
        return false;
    }

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
            *error = QStringLiteral("Text point cloud contains no valid XYZ rows: %1").arg(path);
        }
        return false;
    }
    return true;
}

bool WriteSmokeJson(
    const QString& path,
    const QString& cloudPath,
    const QString& modelPath,
    size_t pointCount,
    const WeldSectionMeasurement& r)
{
    QJsonObject obj;
    obj.insert(QStringLiteral("source"),
               QStringLiteral("IPC WeldMeasureService + third_party WeldMeasure.dll"));
    obj.insert(QStringLiteral("input_cloud"), cloudPath);
    obj.insert(QStringLiteral("onnx_model"), modelPath.isEmpty() ? QStringLiteral("(from_ini)") : modelPath);
    obj.insert(QStringLiteral("loaded_points"), static_cast<qint64>(pointCount));
    obj.insert(QStringLiteral("mismatch_mm"), r.mismatchMm);
    obj.insert(QStringLiteral("reinforcement_mm"), r.reinforcementMm);
    obj.insert(QStringLiteral("included_angle_rad"), r.includedAngleRad);
    obj.insert(QStringLiteral("included_angle_deg"), r.includedAngleRad * 180.0 / 3.14159265358979323846);
    obj.insert(QStringLiteral("angularity_mm"), r.angularityMm);
    obj.insert(QStringLiteral("left_undercut_mm"), r.leftUndercutMm);
    obj.insert(QStringLiteral("right_undercut_mm"), r.rightUndercutMm);
    obj.insert(QStringLiteral("max_undercut_mm"), r.maxUndercutMm);
    obj.insert(QStringLiteral("toe_center_x"), r.toeCenterX);
    obj.insert(QStringLiteral("undercut_raw_flag"), r.undercutRawFlag);
    obj.insert(QStringLiteral("valid"), r.valid);

    QJsonArray leftToe;
    leftToe.append(r.leftToeX);
    leftToe.append(r.leftToeY);
    leftToe.append(r.leftToeZ);
    obj.insert(QStringLiteral("left_toe_mm"), leftToe);

    QJsonArray rightToe;
    rightToe.append(r.rightToeX);
    rightToe.append(r.rightToeY);
    rightToe.append(r.rightToeZ);
    obj.insert(QStringLiteral("right_toe_mm"), rightToe);

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
    const QString modelArg = (argc >= 4) ? QString::fromLocal8Bit(argv[3]) : QString();

    QString cloudPath = dataDir;
    const QFileInfo dataInfo(dataDir);
    if (dataInfo.isDir() || !dataInfo.exists()) {
        const QString candidate = QDir(dataDir).filePath(QStringLiteral("1_000000.txt"));
        if (QFileInfo::exists(candidate)) {
            cloudPath = candidate;
        }
    }

    std::printf("WeldMeasure IPC smoke (WeldMeasureService)\n");
    std::printf("Cloud: %s\n", qPrintable(cloudPath));
    std::printf("Out: %s\n", qPrintable(outPath));
    std::fflush(stdout);

    std::vector<float> xyz;
    size_t pointCount = 0;
    QString loadError;
    if (!LoadTextXyz(cloudPath, &xyz, &pointCount, &loadError)) {
        std::fprintf(stderr, "Load failed: %s\n", qPrintable(loadError));
        return 2;
    }
    std::printf("Loaded points: %zu\n", pointCount);
    std::fflush(stdout);

    WeldMeasureService service;
    WeldMeasureError error;
    const bool ok = modelArg.isEmpty()
        ? service.initializeFromIni(QString(), &error)
        : (QFileInfo(modelArg).suffix().compare(QStringLiteral("ini"), Qt::CaseInsensitive) == 0
               ? service.initializeFromIni(modelArg, &error)
               : service.initialize(modelArg, &error));
    if (!ok) {
        std::fprintf(stderr, "initialize failed: %s (code=%d)\n",
                     qPrintable(error.message), error.statusCode);
        return 3;
    }
    if (!service.configPath().isEmpty()) {
        std::printf("Config: %s\n", qPrintable(service.configPath()));
    }
    std::printf("Model: %s\n", qPrintable(service.modelPath()));
    std::printf("WeldMeasureService ready\n");
    std::fflush(stdout);

    WeldSectionMeasurement result;
    if (!service.measureSection(xyz.data(), pointCount, &result, &error) || !result.valid) {
        std::fprintf(stderr, "measureSection failed: %s (code=%d) valid=%d\n",
                     qPrintable(error.message), error.statusCode, result.valid ? 1 : 0);
        return 4;
    }

    const double includedAngleDeg = result.includedAngleRad * 180.0 / 3.14159265358979323846;
    std::printf("\n=== smoke_B result ===\n");
    std::printf("mismatch_mm: %.12f\n", result.mismatchMm);
    std::printf("reinforcement_mm: %.12f\n", result.reinforcementMm);
    std::printf("included_angle_rad: %.12f\n", result.includedAngleRad);
    std::printf("included_angle_deg: %.12f\n", includedAngleDeg);
    std::printf("angularity_mm: %.12f\n", result.angularityMm);
    std::printf("left_undercut_mm: %.12f\n", result.leftUndercutMm);
    std::printf("right_undercut_mm: %.12f\n", result.rightUndercutMm);
    std::printf("max_undercut_mm: %.12f\n", result.maxUndercutMm);
    std::printf("left_toe_mm: %.6f, %.6f, %.6f\n",
                result.leftToeX, result.leftToeY, result.leftToeZ);
    std::printf("right_toe_mm: %.6f, %.6f, %.6f\n",
                result.rightToeX, result.rightToeY, result.rightToeZ);
    std::printf("valid: %d\n", result.valid ? 1 : 0);
    std::fflush(stdout);

    if (!WriteSmokeJson(outPath, cloudPath, service.modelPath(), pointCount, result)) {
        std::fprintf(stderr, "Failed to write: %s\n", qPrintable(outPath));
        return 5;
    }
    std::printf("Saved: %s\n", qPrintable(outPath));
    std::fflush(stdout);
    return 0;
}
