#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QElapsedTimer>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QString>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/station2_inspection.h"
#include "scan_tracking/inner_surface_measure/inner_surface_measure_service.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"
#include "scan_tracking/thickness_measure_v2/thickness_measure_v2_service.h"

namespace {

using scan_tracking::common::ScanDeviceKind;
using scan_tracking::flow_control::InspectionCloudSnapshot;
using scan_tracking::flow_control::InspectionQuota;
using scan_tracking::flow_control::InspectionSegmentCloud;
using scan_tracking::mech_eye::PointCloudFrame;

struct LoadedCloud {
    int localIndex = 0;
    int pointCount = 0;
    std::shared_ptr<std::vector<float>> xyz;  // dense finite xyz only
};

QString preferredCloudPath(const QString& root, int segmentIndex)
{
    const QString stitched = QDir(root).filePath(
        QStringLiteral("arm/%1/Path4_Arm_cloud_stitched_%1.ply").arg(segmentIndex));
    if (QFileInfo::exists(stitched)) {
        return stitched;
    }
    return QDir(root).filePath(
        QStringLiteral("arm/%1/Path4_Arm_cloud_%1.ply").arg(segmentIndex));
}

std::shared_ptr<std::vector<float>> packFiniteXyz(
    const std::shared_ptr<std::vector<float>>& src,
    int pointCount)
{
    auto out = std::make_shared<std::vector<float>>();
    if (!src || pointCount <= 0) {
        return out;
    }
    out->reserve(static_cast<size_t>(pointCount) * 3);
    for (int i = 0; i < pointCount; ++i) {
        const float x = (*src)[i * 3 + 0];
        const float y = (*src)[i * 3 + 1];
        const float z = (*src)[i * 3 + 2];
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            out->push_back(x);
            out->push_back(y);
            out->push_back(z);
        }
    }
    return out;
}

const LoadedCloud* findCloud(const std::vector<LoadedCloud>& clouds, int localIndex)
{
    for (const LoadedCloud& c : clouds) {
        if (c.localIndex == localIndex) {
            return &c;
        }
    }
    return nullptr;
}

struct PcdField {
    QString name;
    QString type;
    int size = 4;
    int count = 1;
};

bool loadXyzPcd(const QString& path, std::vector<float>* xyz, size_t* count, QString* error)
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
        if (line.isEmpty()) {
            continue;
        }
        if (line.startsWith(QLatin1String("FIELDS"), Qt::CaseInsensitive)) {
            fields.clear();
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            for (int i = 1; i < parts.size(); ++i) {
                PcdField f;
                f.name = parts[i];
                fields.push_back(f);
            }
        } else if (line.startsWith(QLatin1String("SIZE"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            for (int i = 1; i < parts.size() && i - 1 < fields.size(); ++i) {
                fields[i - 1].size = parts[i].toInt();
            }
        } else if (line.startsWith(QLatin1String("TYPE"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            for (int i = 1; i < parts.size() && i - 1 < fields.size(); ++i) {
                fields[i - 1].type = parts[i];
            }
        } else if (line.startsWith(QLatin1String("COUNT"), Qt::CaseInsensitive)) {
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            for (int i = 1; i < parts.size() && i - 1 < fields.size(); ++i) {
                fields[i - 1].count = parts[i].toInt();
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

    int xIdx = -1;
    int yIdx = -1;
    int zIdx = -1;
    int pointStride = 0;
    for (int i = 0; i < fields.size(); ++i) {
        if (fields[i].name.compare(QLatin1String("x"), Qt::CaseInsensitive) == 0) {
            xIdx = i;
        } else if (fields[i].name.compare(QLatin1String("y"), Qt::CaseInsensitive) == 0) {
            yIdx = i;
        } else if (fields[i].name.compare(QLatin1String("z"), Qt::CaseInsensitive) == 0) {
            zIdx = i;
        }
        pointStride += fields[i].size * fields[i].count;
    }
    if (xIdx < 0 || yIdx < 0 || zIdx < 0 || points <= 0) {
        if (error) {
            *error = QStringLiteral("PCD missing xyz/points: %1").arg(path);
        }
        return false;
    }

    xyz->clear();
    xyz->reserve(static_cast<size_t>(points) * 3);
    if (dataType == QLatin1String("ascii")) {
        while (!file.atEnd() && static_cast<qint64>(xyz->size() / 3) < points) {
            const QString line = QString::fromLatin1(file.readLine()).trimmed();
            if (line.isEmpty()) {
                continue;
            }
            const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            if (parts.size() <= qMax(xIdx, qMax(yIdx, zIdx))) {
                continue;
            }
            const float x = parts[xIdx].toFloat();
            const float y = parts[yIdx].toFloat();
            const float z = parts[zIdx].toFloat();
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                xyz->push_back(x);
                xyz->push_back(y);
                xyz->push_back(z);
            }
        }
    } else {
        const QByteArray body = file.readAll();
        if (body.size() < points * pointStride) {
            if (error) {
                *error = QStringLiteral("PCD binary truncated: %1").arg(path);
            }
            return false;
        }
        int xOff = 0;
        int yOff = 0;
        int zOff = 0;
        int off = 0;
        for (int i = 0; i < fields.size(); ++i) {
            if (i == xIdx) {
                xOff = off;
            }
            if (i == yIdx) {
                yOff = off;
            }
            if (i == zIdx) {
                zOff = off;
            }
            off += fields[i].size * fields[i].count;
        }
        for (qint64 i = 0; i < points; ++i) {
            const char* base = body.constData() + i * pointStride;
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            std::memcpy(&x, base + xOff, sizeof(float));
            std::memcpy(&y, base + yOff, sizeof(float));
            std::memcpy(&z, base + zOff, sizeof(float));
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                xyz->push_back(x);
                xyz->push_back(y);
                xyz->push_back(z);
            }
        }
    }

    *count = xyz->size() / 3;
    return *count > 0;
}

std::shared_ptr<std::vector<float>> translateCloud(
    const std::shared_ptr<std::vector<float>>& src,
    float dx,
    float dy,
    float dz)
{
    auto out = std::make_shared<std::vector<float>>();
    if (!src || src->size() < 3) {
        return out;
    }
    out->resize(src->size());
    const int n = static_cast<int>(src->size() / 3);
    for (int i = 0; i < n; ++i) {
        (*out)[i * 3 + 0] = (*src)[i * 3 + 0] + dx;
        (*out)[i * 3 + 1] = (*src)[i * 3 + 1] + dy;
        (*out)[i * 3 + 2] = (*src)[i * 3 + 2] + dz;
    }
    return out;
}

bool cloudCentroid(const std::vector<float>& xyz, float* cx, float* cy, float* cz)
{
    if (xyz.size() < 3 || cx == nullptr || cy == nullptr || cz == nullptr) {
        return false;
    }
    const int n = static_cast<int>(xyz.size() / 3);
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    for (int i = 0; i < n; ++i) {
        sx += xyz[i * 3 + 0];
        sy += xyz[i * 3 + 1];
        sz += xyz[i * 3 + 2];
    }
    *cx = static_cast<float>(sx / n);
    *cy = static_cast<float>(sy / n);
    *cz = static_cast<float>(sz / n);
    return true;
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    if (argc < 2 || argc > 3) {
        std::fprintf(
            stderr,
            "Usage: scan_tracking_path4_offline <path_4 directory> [stride]\n");
        return 1;
    }

    const QString root = QDir::cleanPath(QString::fromLocal8Bit(argv[1]));
    if (!QFileInfo(root).isDir()) {
        std::fprintf(stderr, "Path4 directory does not exist: %s\n", qPrintable(root));
        return 2;
    }

    const int stride = argc == 3 ? std::atoi(argv[2]) : 1;
    if (stride < 1) {
        std::fprintf(stderr, "stride must be positive\n");
        return 1;
    }

    scan_tracking::common::ConfigManager::initialize();
    if (scan_tracking::common::ConfigManager::instance() == nullptr ||
        scan_tracking::common::ConfigManager::instance()->findScanPathById(4) == nullptr) {
        std::fprintf(stderr, "ConfigManager failed to load pathId=4 from scan_paths\n");
        return 2;
    }

    const bool algoEnabled =
        scan_tracking::common::ConfigManager::instance()->flowControlConfig().algorithmEnabled;
    std::printf("algorithmEnabled=%s\n", algoEnabled ? "true" : "false");
    std::fflush(stdout);

    constexpr int kArmCount = 12;
    QElapsedTimer loadTimer;
    loadTimer.start();

    InspectionCloudSnapshot snapshot;
    snapshot.runCaptureRoot = root;
    snapshot.segments.reserve(kArmCount);
    std::vector<LoadedCloud> loaded;
    loaded.reserve(kArmCount);

    qint64 totalPoints = 0;
    for (int index = 1; index <= kArmCount; ++index) {
        const QString path = preferredCloudPath(root, index);
        PointCloudFrame frame;
        if (!scan_tracking::mech_eye::loadPointCloudFrameFromPly(path, &frame)) {
            std::fprintf(stderr, "Failed to load PLY: %s\n", qPrintable(path));
            return 3;
        }

        auto packed = packFiniteXyz(frame.pointsXYZ, frame.pointCount);
        if (stride > 1 && packed && !packed->empty()) {
            auto sampled = std::make_shared<std::vector<float>>();
            const int n = static_cast<int>(packed->size() / 3);
            sampled->reserve((n / stride + 1) * 3);
            for (int point = 0; point < n; point += stride) {
                sampled->push_back((*packed)[point * 3]);
                sampled->push_back((*packed)[point * 3 + 1]);
                sampled->push_back((*packed)[point * 3 + 2]);
            }
            packed = std::move(sampled);
        }

        LoadedCloud cloud;
        cloud.localIndex = index;
        cloud.xyz = packed;
        cloud.pointCount = packed ? static_cast<int>(packed->size() / 3) : 0;
        loaded.push_back(cloud);

        InspectionSegmentCloud segment;
        segment.device = ScanDeviceKind::Arm;
        segment.localIndex = index;
        segment.captureOk = true;
        segment.xyz = cloud.xyz;
        segment.pointCount = cloud.pointCount;
        snapshot.segments.push_back(std::move(segment));

        totalPoints += cloud.pointCount;
        std::printf(
            "Loaded arm %d/%d: finite=%d (%s)\n",
            index,
            kArmCount,
            cloud.pointCount,
            qPrintable(QFileInfo(path).fileName()));
        std::fflush(stdout);
    }

    std::printf(
        "Loaded %d arm clouds: %lld finite points (stride=%d), %.3f s\n",
        kArmCount,
        static_cast<long long>(totalPoints),
        stride,
        loadTimer.nsecsElapsed() / 1e9);
    std::fflush(stdout);

    int thicknessOk = 0;
    int innerOk = 0;

    // --- thickness pairs 1-2 / 11-12 ---
    std::printf("\n==== thickness V2 pairs ====\n");
    {
        const QString thicknessIni =
            QDir(QCoreApplication::applicationDirPath())
                .filePath(QStringLiteral("config/thickness_measure_v3/thickness_measurement.ini"));
        const QString outerTemplatePath =
            QDir(QCoreApplication::applicationDirPath())
                .filePath(QStringLiteral(
                    "config/thickness_measure_v3/Data/Template_Path4_Arm_cloud_stitched_1_sample.pcd"));
        std::printf("thickness ini: %s\n", qPrintable(thicknessIni));
        std::printf("outer template: %s exists=%d\n",
                    qPrintable(outerTemplatePath),
                    QFileInfo::exists(outerTemplatePath) ? 1 : 0);

        std::vector<float> templateXyz;
        size_t templateCount = 0;
        QString templateLoadError;
        float tcx = 0.0f;
        float tcy = 0.0f;
        float tcz = 0.0f;
        const bool haveTemplateCentroid =
            loadXyzPcd(outerTemplatePath, &templateXyz, &templateCount, &templateLoadError) &&
            cloudCentroid(templateXyz, &tcx, &tcy, &tcz);
        if (!haveTemplateCentroid) {
            std::printf("template centroid failed: %s\n", qPrintable(templateLoadError));
        } else {
            std::printf(
                "template pts=%zu centroid=(%.1f,%.1f,%.1f)\n",
                templateCount,
                tcx,
                tcy,
                tcz);
        }

        scan_tracking::thickness_measure_v2::ThicknessMeasureV2Service thickness;
        scan_tracking::thickness_measure_v2::ThicknessV2Error thicknessError;
        if (!thickness.initializeFromIni(thicknessIni, &thicknessError)) {
            std::printf("thickness init failed: %s\n", qPrintable(thicknessError.message));
        } else {
            const int pairs[2][2] = {{1, 2}, {11, 12}};
            QVector<scan_tracking::thickness_measure_v2::ThicknessV2PairClouds> pairClouds;
            // Keep translated buffers alive for the measure call.
            std::vector<std::shared_ptr<std::vector<float>>> keepAlive;
            keepAlive.reserve(4);

            for (const auto& pair : pairs) {
                const LoadedCloud* inner = findCloud(loaded, pair[0]);
                const LoadedCloud* outer = findCloud(loaded, pair[1]);
                if (inner == nullptr || outer == nullptr || !inner->xyz || !outer->xyz) {
                    std::printf("pair %d/%d missing clouds\n", pair[0], pair[1]);
                    continue;
                }

                std::shared_ptr<std::vector<float>> innerAligned = inner->xyz;
                std::shared_ptr<std::vector<float>> outerAligned = outer->xyz;
                if (haveTemplateCentroid) {
                    float ocx = 0.0f;
                    float ocy = 0.0f;
                    float ocz = 0.0f;
                    if (cloudCentroid(*outer->xyz, &ocx, &ocy, &ocz)) {
                        const float dx = tcx - ocx;
                        const float dy = tcy - ocy;
                        const float dz = tcz - ocz;
                        // Same translation on inner+outer keeps relative pose for InnerOuterICP.
                        innerAligned = translateCloud(inner->xyz, dx, dy, dz);
                        outerAligned = translateCloud(outer->xyz, dx, dy, dz);
                        std::printf(
                            "pair %d/%d prealign outerCentroid=(%.1f,%.1f,%.1f) delta=(%.1f,%.1f,%.1f)\n",
                            pair[0],
                            pair[1],
                            ocx,
                            ocy,
                            ocz,
                            dx,
                            dy,
                            dz);
                    }
                }
                keepAlive.push_back(innerAligned);
                keepAlive.push_back(outerAligned);

                scan_tracking::thickness_measure_v2::ThicknessV2PairClouds view;
                view.inner.xyz = innerAligned->data();
                view.inner.pointCount = innerAligned->size() / 3;
                view.outer.xyz = outerAligned->data();
                view.outer.pointCount = outerAligned->size() / 3;
                pairClouds.push_back(view);
                std::printf(
                    "pair inner=%d outer=%d pts=%zu/%zu\n",
                    pair[0],
                    pair[1],
                    view.inner.pointCount,
                    view.outer.pointCount);
            }

            scan_tracking::thickness_measure_v2::ThicknessV2AverageMeasurement average;
            if (!thickness.measurePairsAverage(pairClouds, &average, &thicknessError) ||
                average.successCount == 0) {
                std::printf(
                    "thickness FAILED: %s success=%zu/%zu\n",
                    qPrintable(thicknessError.message),
                    average.successCount,
                    average.pairCount > 0 ? average.pairCount : pairClouds.size());
            } else {
                thicknessOk = 1;
                std::printf(
                    "thicknessMm=%.6f success=%zu/%zu\n",
                    average.thicknessMm,
                    average.successCount,
                    average.pairCount);
            }
        }
    }
    std::fflush(stdout);

    // --- inner surface ends 3 + 10 ---
    std::printf("\n==== inner surface ends 3+10 ====\n");
    {
        const LoadedCloud* end1 = findCloud(loaded, 3);
        const LoadedCloud* end2 = findCloud(loaded, 10);
        if (end1 == nullptr || end2 == nullptr || !end1->xyz || !end2->xyz) {
            std::printf("inner ends missing\n");
        } else {
            scan_tracking::inner_surface_measure::InnerSurfaceMeasureService inner;
            scan_tracking::inner_surface_measure::InnerSurfaceMeasureError innerError;
            const QString iniPath =
                scan_tracking::inner_surface_measure::InnerSurfaceMeasureService::defaultConfigPath();
            std::printf("inner ini: %s exists=%d\n", qPrintable(iniPath), QFileInfo::exists(iniPath) ? 1 : 0);

            // Always use memory init for offline field clouds: larger ICP gate + known containerLength.
            const QString templatePath = QDir(QFileInfo(iniPath).absolutePath())
                                              .filePath(QStringLiteral("Data/sample_cylinder.pcd"));
            std::vector<float> templateXyz;
            size_t templateCount = 0;
            QString loadError;
            bool ready = false;
            if (!loadXyzPcd(templatePath, &templateXyz, &templateCount, &loadError)) {
                std::printf("template load failed: %s\n", qPrintable(loadError));
            } else {
                scan_tracking::inner_surface_measure::InnerSurfaceConfig cfg;
                cfg.voxelSize = 2.0f;  // heavier downsample for multi-million field clouds
                cfg.outlierK = 0;
                cfg.fitIterations = 5;
                cfg.sectionHalfWidth = 4.0f;
                cfg.icpMaxIterations = 80;
                cfg.icpMaxCorrespondenceDistance = 1500.0f;
                cfg.cylinderAxisZ = 1.0;
                cfg.cylinderRadius = 600.0f;
                cfg.containerLengthMm = 2194.0;
                ready = inner.initialize(cfg, templateXyz.data(), templateCount, &innerError);
                if (!ready) {
                    std::printf(
                        "initialize(memory) failed code=%d msg=%s\n",
                        innerError.statusCode,
                        qPrintable(innerError.message));
                } else {
                    std::printf(
                        "inner ready: templatePts=%zu icpMaxCorr=%.0f voxel=%.1f\n",
                        templateCount,
                        cfg.icpMaxCorrespondenceDistance,
                        cfg.voxelSize);
                }
            }

            if (ready) {
                // Offline-only: field clouds are in world/robot frame; sample template is local.
                float tcx = 0.0f;
                float tcy = 0.0f;
                float tcz = 0.0f;
                std::shared_ptr<std::vector<float>> end1Aligned = end1->xyz;
                std::shared_ptr<std::vector<float>> end2Aligned = end2->xyz;
                if (cloudCentroid(templateXyz, &tcx, &tcy, &tcz)) {
                    float c1x = 0.0f;
                    float c1y = 0.0f;
                    float c1z = 0.0f;
                    float c2x = 0.0f;
                    float c2y = 0.0f;
                    float c2z = 0.0f;
                    if (cloudCentroid(*end1->xyz, &c1x, &c1y, &c1z) &&
                        cloudCentroid(*end2->xyz, &c2x, &c2y, &c2z)) {
                        end1Aligned = translateCloud(end1->xyz, tcx - c1x, tcy - c1y, tcz - c1z);
                        end2Aligned = translateCloud(end2->xyz, tcx - c2x, tcy - c2y, tcz - c2z);
                        std::printf(
                            "prealign centroid template=(%.1f,%.1f,%.1f) "
                            "end3=(%.1f,%.1f,%.1f) end10=(%.1f,%.1f,%.1f)\n",
                            tcx,
                            tcy,
                            tcz,
                            c1x,
                            c1y,
                            c1z,
                            c2x,
                            c2y,
                            c2z);
                    }
                }

                scan_tracking::inner_surface_measure::InnerSurfaceAverageMeasurement avg;
                scan_tracking::inner_surface_measure::InnerSurfaceFrameMeasurement f1;
                scan_tracking::inner_surface_measure::InnerSurfaceFrameMeasurement f2;
                QElapsedTimer innerTimer;
                innerTimer.start();
                const bool ok = inner.measureTwoFramesAverage(
                    end1Aligned->data(),
                    end1Aligned->size() / 3,
                    end2Aligned->data(),
                    end2Aligned->size() / 3,
                    &avg,
                    &f1,
                    &f2,
                    &innerError);
                std::printf("inner elapsed: %.3f s\n", innerTimer.nsecsElapsed() / 1e9);
                if (!ok || !avg.valid) {
                    std::printf("inner FAILED: %s\n", qPrintable(innerError.message));
                    std::printf(
                        "note: sample_cylinder.pcd is a demo template; field ends need a "
                        "site-matched inner template (same frame as scan) for production.\n");
                } else {
                    innerOk = 1;
                    std::printf("diameterMm=%.6f\n", avg.diameterMm);
                    std::printf("circumferenceMm=%.6f\n", avg.circumferenceMm);
                    std::printf("roundness=%.6f\n", avg.roundness);
                    std::printf("containerLengthMm=%.3f\n", avg.containerLengthMm);
                    std::printf("volumeLiters=%.6f\n", avg.volumeLiters);
                    std::printf(
                        "frame1: d=%.3f round=%.3f pts=%d icp=%s fitness=%.3f\n",
                        f1.diameterMm,
                        f1.averageRoundness,
                        f1.usedPointCount,
                        f1.icpConverged ? "true" : "false",
                        f1.icpFitnessScore);
                    std::printf(
                        "frame2: d=%.3f round=%.3f pts=%d icp=%s fitness=%.3f\n",
                        f2.diameterMm,
                        f2.averageRoundness,
                        f2.usedPointCount,
                        f2.icpConverged ? "true" : "false",
                        f2.icpFitnessScore);
                }
            }
        }
    }
    std::fflush(stdout);

    // --- formal evaluate path (needs algorithmEnabled=true) ---
    std::printf("\n==== evaluateStation2Inspection ====\n");
    if (!algoEnabled) {
        std::printf("skipped: config.ini FlowControl.algorithmEnabled=false\n");
    } else {
        InspectionQuota quota;
        quota.expectedArmCount = kArmCount;
        quota.expectedTelescopicCount = 0;
        quota.pathId = 4;
        quota.pathName = QStringLiteral("thickness_inner_surface");
        quota.algorithm = QStringLiteral("thickness_inner_surface");

        QElapsedTimer measureTimer;
        measureTimer.start();
        const auto result = scan_tracking::flow_control::evaluateStation2Inspection(
            snapshot, 0, quota);
        std::printf("evaluate elapsed: %.3f s\n", measureTimer.nsecsElapsed() / 1e9);
        const QByteArray text =
            scan_tracking::flow_control::formatInspectionResultTextBlock(result).toUtf8();
        std::fwrite(text.constData(), 1, static_cast<std::size_t>(text.size()), stdout);
        std::fflush(stdout);
    }

    std::printf("\n==== summary ====\n");
    std::printf("thicknessOk=%d innerOk=%d\n", thicknessOk, innerOk);
    std::fflush(stdout);

    scan_tracking::common::ConfigManager::cleanup();
    return (thicknessOk && innerOk) ? 0 : 4;
}
