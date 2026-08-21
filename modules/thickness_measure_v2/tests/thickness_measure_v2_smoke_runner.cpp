#include <cstdio>
#include <cstring>
#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QString>

#include "scan_tracking/thickness_measure_v2/thickness_measure_v2_service.h"

using scan_tracking::thickness_measure_v2::ThicknessMeasureV2Service;
using scan_tracking::thickness_measure_v2::ThicknessV2AverageMeasurement;
using scan_tracking::thickness_measure_v2::ThicknessV2Error;
using scan_tracking::thickness_measure_v2::ThicknessV2PairClouds;
using scan_tracking::thickness_measure_v2::ThicknessV2PairMeasurement;

namespace {

struct PcdField {
    QString name;
    QString type;
    int size = 4;
    int count = 1;
};

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

    while (!file.atEnd()) {
        QString line = QString::fromLatin1(file.readLine()).trimmed();
        if (line.endsWith(QLatin1Char('\r'))) {
            line.chop(1);
        }
        if (line.isEmpty()) {
            continue;
        }
        if (line.startsWith(QLatin1String("FIELDS"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
            fields.clear();
            for (int i = 1; i < parts.size(); ++i) {
                PcdField f;
                f.name = parts[i];
                fields.push_back(f);
            }
        } else if (line.startsWith(QLatin1String("SIZE"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
            for (int i = 1; i < parts.size() && i - 1 < fields.size(); ++i) {
                fields[i - 1].size = parts[i].toInt();
            }
        } else if (line.startsWith(QLatin1String("TYPE"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
            for (int i = 1; i < parts.size() && i - 1 < fields.size(); ++i) {
                fields[i - 1].type = parts[i];
            }
        } else if (line.startsWith(QLatin1String("COUNT"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
            for (int i = 1; i < parts.size() && i - 1 < fields.size(); ++i) {
                fields[i - 1].count = parts[i].toInt();
            }
        } else if (line.startsWith(QLatin1String("POINTS"), Qt::CaseInsensitive)) {
            points = line.section(QLatin1Char(' '), 1).trimmed().toLongLong();
        } else if (line.startsWith(QLatin1String("DATA"), Qt::CaseInsensitive)) {
            dataType = line.section(QLatin1Char(' '), 1).trimmed().toLower();
            break;
        }
    }

    int xIndex = -1;
    int yIndex = -1;
    int zIndex = -1;
    int stride = 0;
    for (int i = 0; i < fields.size(); ++i) {
        if (fields[i].name.compare(QLatin1String("x"), Qt::CaseInsensitive) == 0) {
            xIndex = i;
        } else if (fields[i].name.compare(QLatin1String("y"), Qt::CaseInsensitive) == 0) {
            yIndex = i;
        } else if (fields[i].name.compare(QLatin1String("z"), Qt::CaseInsensitive) == 0) {
            zIndex = i;
        }
        stride += fields[i].size * fields[i].count;
    }
    if (xIndex < 0 || yIndex < 0 || zIndex < 0 || points <= 0) {
        if (error) {
            *error = QStringLiteral("Invalid PCD header: %1").arg(path);
        }
        return false;
    }

    xyz->clear();
    xyz->reserve(static_cast<size_t>(points) * 3);
    if (dataType.startsWith(QLatin1String("ascii"))) {
        while (!file.atEnd() && static_cast<qint64>(xyz->size() / 3) < points) {
            QString line = QString::fromLatin1(file.readLine()).trimmed();
            if (line.isEmpty()) {
                continue;
            }
            const QStringList parts = line.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
            if (parts.size() <= zIndex) {
                continue;
            }
            xyz->push_back(parts[xIndex].toFloat());
            xyz->push_back(parts[yIndex].toFloat());
            xyz->push_back(parts[zIndex].toFloat());
        }
    } else {
        const QByteArray blob = file.readAll();
        if (blob.size() < points * stride) {
            if (error) {
                *error = QStringLiteral("Binary PCD truncated: %1").arg(path);
            }
            return false;
        }
        int xOff = 0;
        int yOff = 0;
        int zOff = 0;
        int off = 0;
        for (int i = 0; i < fields.size(); ++i) {
            if (i == xIndex) {
                xOff = off;
            }
            if (i == yIndex) {
                yOff = off;
            }
            if (i == zIndex) {
                zOff = off;
            }
            off += fields[i].size * fields[i].count;
        }
        for (qint64 i = 0; i < points; ++i) {
            const char* base = blob.constData() + i * stride;
            xyz->push_back(ReadFloatField(base + xOff, fields[xIndex]));
            xyz->push_back(ReadFloatField(base + yOff, fields[yIndex]));
            xyz->push_back(ReadFloatField(base + zOff, fields[zIndex]));
        }
    }

    *count = xyz->size() / 3;
    return *count > 0;
}

QString ResolveBesideExe(const QString& relative)
{
    return QDir(QCoreApplication::applicationDirPath()).filePath(relative);
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    const QString ini = argc > 1
        ? QString::fromLocal8Bit(argv[1])
        : ThicknessMeasureV2Service::defaultConfigPath();

    std::printf("ThicknessMeasureV2 smoke\n");
    std::printf("  config: %s\n", qPrintable(ini));
    if (!QFileInfo::exists(ini)) {
        std::printf("FAIL: config not found (deploy runtime first)\n");
        return 2;
    }

    ThicknessMeasureV2Service service;
    ThicknessV2Error error;
    if (!service.initializeFromIni(ini, &error)) {
        std::printf("FAIL: initializeFromIni: %s\n", qPrintable(error.message));
        return 3;
    }
    std::printf("OK: context ready\n");

    QString innerPath = argc > 2
        ? QString::fromLocal8Bit(argv[2])
        : ResolveBesideExe(QStringLiteral("config/thickness_measure_v2/Data/0_scan_inner_sample.pcd"));
    QString outerPath = argc > 3
        ? QString::fromLocal8Bit(argv[3])
        : ResolveBesideExe(QStringLiteral("config/thickness_measure_v2/Data/0_scan_outer_sample.pcd"));

    // Fallback to SDK tree when running from build dir without POST_BUILD deploy layout.
    if (!QFileInfo::exists(innerPath)) {
        innerPath = QDir(QCoreApplication::applicationDirPath())
                        .absoluteFilePath(QStringLiteral("../../../../../third_party/thickness_measure_v2/Data/0_scan_inner_sample.pcd"));
    }
    if (!QFileInfo::exists(outerPath)) {
        outerPath = QDir(QCoreApplication::applicationDirPath())
                        .absoluteFilePath(QStringLiteral("../../../../../third_party/thickness_measure_v2/Data/0_scan_outer_sample.pcd"));
    }

    if (!QFileInfo::exists(innerPath) || !QFileInfo::exists(outerPath)) {
        std::printf("SKIP measure: sample PCD not found\n  inner=%s\n  outer=%s\n",
                     qPrintable(innerPath),
                     qPrintable(outerPath));
        return 0;
    }

    std::vector<float> innerXyz;
    std::vector<float> outerXyz;
    size_t innerCount = 0;
    size_t outerCount = 0;
    QString loadError;
    if (!LoadXyzPcd(innerPath, &innerXyz, &innerCount, &loadError) ||
        !LoadXyzPcd(outerPath, &outerXyz, &outerCount, &loadError)) {
        std::printf("FAIL: load PCD: %s\n", qPrintable(loadError));
        return 4;
    }
    std::printf("  inner pts=%zu outer pts=%zu\n", innerCount, outerCount);

    ThicknessV2PairMeasurement pair;
    if (!service.measurePair(
            innerXyz.data(), innerCount, outerXyz.data(), outerCount, &pair, &error) ||
        !pair.valid) {
        std::printf("FAIL: measurePair: %s\n", qPrintable(error.message));
        return 5;
    }
    std::printf("OK: thickness=%.3f mm method=%s sections=%d\n",
                pair.thicknessMm,
                qPrintable(pair.method),
                pair.sectionCount);

    ThicknessV2PairClouds clouds;
    clouds.inner.xyz = innerXyz.data();
    clouds.inner.pointCount = innerCount;
    clouds.outer.xyz = outerXyz.data();
    clouds.outer.pointCount = outerCount;
    ThicknessV2AverageMeasurement average;
    if (!service.measurePairsAverage({clouds}, &average, &error) || !average.valid) {
        std::printf("FAIL: measurePairsAverage: %s\n", qPrintable(error.message));
        return 6;
    }
    std::printf("OK: average=%.3f mm success=%zu/%zu\n",
                average.thicknessMm,
                average.successCount,
                average.pairCount);
    return 0;
}
