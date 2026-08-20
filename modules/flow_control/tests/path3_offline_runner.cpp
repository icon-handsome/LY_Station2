#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QElapsedTimer>
#include <QtCore/QFileInfo>
#include <QtCore/QString>

#include "scan_tracking/container_total_length_measure/container_total_length_service.h"
#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/station2_inspection.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"

namespace {

using scan_tracking::common::ScanDeviceKind;
using scan_tracking::flow_control::InspectionCloudSnapshot;
using scan_tracking::flow_control::InspectionQuota;
using scan_tracking::flow_control::InspectionSegmentCloud;
using scan_tracking::mech_eye::PointCloudFrame;

QString stitchedCloudPath(const QString& root, int segmentIndex)
{
    return QDir(root).filePath(
        QStringLiteral("arm/%1/Path3_Arm_cloud_stitched_%1.ply").arg(segmentIndex));
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    if (argc < 2 || argc > 4) {
        std::fprintf(stderr, "Usage: scan_tracking_path3_offline <path_3 directory> [segment_count] [stride]\n");
        return 1;
    }

    const QString root = QDir::cleanPath(QString::fromLocal8Bit(argv[1]));
    if (!QFileInfo(root).isDir()) {
        std::fprintf(stderr, "Path3 directory does not exist: %s\n", qPrintable(root));
        return 2;
    }

    if (argc == 3 && QString::fromLocal8Bit(argv[2]) == QStringLiteral("init")) {
        scan_tracking::container_total_length_measure::ContainerTotalLengthService service;
        scan_tracking::container_total_length_measure::ContainerTotalLengthError error;
        const bool ready = service.initializeFromIni(QString(), &error);
        std::printf(
            "initialize=%s status=%d message=%s\n",
            ready ? "ok" : "failed",
            error.statusCode,
            qPrintable(error.message));
        return ready ? 0 : 4;
    }

    QElapsedTimer loadTimer;
    loadTimer.start();
    InspectionCloudSnapshot snapshot;
    snapshot.runCaptureRoot = root;
    const int segmentCount = argc >= 3 ? std::atoi(argv[2]) : 8;
    if (segmentCount < 1 || segmentCount > 8) {
        std::fprintf(stderr, "segment_count must be between 1 and 8\n");
        return 1;
    }
    const int stride = argc == 4 ? std::atoi(argv[3]) : 1;
    if (stride < 1) {
        std::fprintf(stderr, "stride must be positive\n");
        return 1;
    }
    snapshot.segments.reserve(segmentCount);

    qint64 totalPoints = 0;
    for (int index = 1; index <= segmentCount; ++index) {
        const QString path = stitchedCloudPath(root, index);
        PointCloudFrame frame;
        if (!scan_tracking::mech_eye::loadPointCloudFrameFromPly(path, &frame)) {
            std::fprintf(stderr, "Failed to load stitched PLY: %s\n", qPrintable(path));
            return 3;
        }

        InspectionSegmentCloud segment;
        segment.device = ScanDeviceKind::Arm;
        segment.localIndex = index;
        segment.captureOk = true;
        segment.cxpParticipated = true;
        segment.lbPoseOk = true;
        segment.xyz = std::move(frame.pointsXYZ);
        segment.pointCount = frame.pointCount;
        int finiteBeforeSample = 0;
        float minX = std::numeric_limits<float>::max();
        float maxX = -std::numeric_limits<float>::max();
        for (int point = 0; point < frame.pointCount; ++point) {
            const float x = (*frame.pointsXYZ)[point * 3];
            const float y = (*frame.pointsXYZ)[point * 3 + 1];
            const float z = (*frame.pointsXYZ)[point * 3 + 2];
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                ++finiteBeforeSample;
                minX = std::min(minX, x);
                maxX = std::max(maxX, x);
            }
        }
        std::printf("  finite=%d/%d xRange=[%g,%g]\n", finiteBeforeSample,
                    frame.pointCount, minX, maxX);
        if (stride > 1) {
            auto sampled = std::make_shared<std::vector<float>>();
            sampled->reserve((segment.pointCount / stride + 1) * 3);
            for (int point = 0; point < segment.pointCount; point += stride) {
                sampled->push_back((*segment.xyz)[point * 3]);
                sampled->push_back((*segment.xyz)[point * 3 + 1]);
                sampled->push_back((*segment.xyz)[point * 3 + 2]);
            }
            segment.xyz = std::move(sampled);
            segment.pointCount = static_cast<int>(segment.xyz->size() / 3);
        }
        snapshot.segments.push_back(std::move(segment));
        totalPoints += frame.pointCount;

        std::printf("Loaded arm %d/%d: %d points\n", index, segmentCount, frame.pointCount);
        std::fflush(stdout);
    }

    std::printf(
        "Loaded %d stitched clouds: %lld points, %.3f s\n",
        segmentCount,
        static_cast<long long>(totalPoints),
        loadTimer.nsecsElapsed() / 1e9);
    std::fflush(stdout);

    InspectionQuota quota;
    quota.expectedArmCount = segmentCount;
    quota.expectedTelescopicCount = 0;
    quota.pathId = 3;
    quota.pathName = QStringLiteral("length_volume");
    quota.algorithm = QStringLiteral("length_volume");

    const auto result = scan_tracking::flow_control::evaluateStation2Inspection(
        snapshot, 0, quota);
    const QByteArray text =
        scan_tracking::flow_control::formatInspectionResultTextBlock(result).toUtf8();
    std::fwrite(text.constData(), 1, static_cast<std::size_t>(text.size()), stdout);
    std::fflush(stdout);
    return result.resultCode == 1 ? 0 : 4;
}
