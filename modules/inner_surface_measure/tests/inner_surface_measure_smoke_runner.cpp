#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QRegularExpression>
#include <QtCore/QString>

#include "scan_tracking/inner_surface_measure/inner_surface_measure_service.h"

using scan_tracking::inner_surface_measure::InnerSurfaceAverageMeasurement;
using scan_tracking::inner_surface_measure::InnerSurfaceConfig;
using scan_tracking::inner_surface_measure::InnerSurfaceFrameMeasurement;
using scan_tracking::inner_surface_measure::InnerSurfaceMeasureError;
using scan_tracking::inner_surface_measure::InnerSurfaceMeasureService;

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

InnerSurfaceConfig MakeConfigFromIniSample()
{
    InnerSurfaceConfig config;
    config.voxelSize = 1.0f;
    config.outlierK = 0;  // path4: skip SOR
    config.outlierStd = 3.0;
    config.fitIterations = 5;
    config.cylinderInlierBand = 0.0f;
    config.sectionHalfWidth = 2.0f;
    config.icpMaxIterations = 100;
    config.icpMaxCorrespondenceDistance = 100.0f;
    config.icpTransformationEpsilon = 1e-8;
    config.icpEuclideanFitnessEpsilon = 1e-6;
    config.cylinderPointX = 0.0;
    config.cylinderPointY = 0.0;
    config.cylinderPointZ = 0.0;
    config.cylinderAxisX = 0.0;
    config.cylinderAxisY = 0.0;
    config.cylinderAxisZ = 1.0;
    config.cylinderRadius = 600.0f;
    config.containerLengthMm = 2194.0;
    return config;
}

void PrintFrame(const char* title, const InnerSurfaceFrameMeasurement& r)
{
    std::printf("%s diameter: %.6f\n", title, r.diameterMm);
    std::printf("%s circumference: %.6f\n", title, r.circumferenceMm);
    std::printf("%s section roundness: %.6f, %.6f, %.6f\n",
                title, r.sectionRoundness[0], r.sectionRoundness[1], r.sectionRoundness[2]);
    std::printf("%s average roundness: %.6f\n", title, r.averageRoundness);
    std::printf("%s ICP converged/fitness: %s / %.6f\n",
                title, r.icpConverged ? "true" : "false", r.icpFitnessScore);
    std::printf("%s used points: %d valid: %d\n\n", title, r.usedPointCount, r.valid ? 1 : 0);
}

QString DefaultDataDir()
{
    return QStringLiteral("D:/work/LY/IPC_Station2/third_party/inner_surface_measure/Data");
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    const QString dataDir = (argc >= 2) ? QString::fromLocal8Bit(argv[1]) : DefaultDataDir();
    const QString templatePath = QDir(dataDir).filePath(QStringLiteral("sample_cylinder.pcd"));
    const QString frame1Path = QDir(dataDir).filePath(QStringLiteral("sample_cylinder_cut_1.pcd"));
    const QString frame2Path = QDir(dataDir).filePath(QStringLiteral("sample_cylinder_cut_2.pcd"));

    std::printf("InnerSurfaceMeasure IPC smoke (InnerSurfaceMeasureService)\n");
    std::printf("Data dir: %s\n", qPrintable(dataDir));

    std::vector<float> templateXyz;
    std::vector<float> frame1Xyz;
    std::vector<float> frame2Xyz;
    size_t nTemplate = 0;
    size_t n1 = 0;
    size_t n2 = 0;
    QString loadError;
    if (!LoadXyzPcd(templatePath, &templateXyz, &nTemplate, &loadError) ||
        !LoadXyzPcd(frame1Path, &frame1Xyz, &n1, &loadError) ||
        !LoadXyzPcd(frame2Path, &frame2Xyz, &n2, &loadError)) {
        std::fprintf(stderr, "Load failed: %s\n", qPrintable(loadError));
        return 2;
    }
    std::printf("Loaded template/frame1/frame2 points: %zu / %zu / %zu\n", nTemplate, n1, n2);

    InnerSurfaceMeasureService service;
    InnerSurfaceMeasureError error;
    const InnerSurfaceConfig config = MakeConfigFromIniSample();
    if (!service.initialize(config, templateXyz.data(), nTemplate, &error)) {
        std::fprintf(stderr, "initialize failed: %s (code=%d)\n",
                     qPrintable(error.message), error.statusCode);
        return 3;
    }

    InnerSurfaceAverageMeasurement average;
    InnerSurfaceFrameMeasurement frame1;
    InnerSurfaceFrameMeasurement frame2;
    if (!service.measureTwoFramesAverage(
            frame1Xyz.data(), n1,
            frame2Xyz.data(), n2,
            &average, &frame1, &frame2, &error)) {
        std::fprintf(stderr, "measureTwoFramesAverage failed: %s (code=%d)\n",
                     qPrintable(error.message), error.statusCode);
        return 4;
    }

    PrintFrame("Frame 1", frame1);
    PrintFrame("Frame 2", frame2);
    std::printf("Average diameter: %.6f\n", average.diameterMm);
    std::printf("Average circumference: %.6f\n", average.circumferenceMm);
    std::printf("Average roundness (6 sections): %.6f\n", average.roundness);
    std::printf("Container length (mm): %.3f\n", average.containerLengthMm);
    std::printf("Average volume (L): %.6f\n", average.volumeLiters);
    std::printf("Average valid: %d\n", average.valid ? 1 : 0);
    return 0;
}
