#include <cstdio>
#include <memory>
#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QString>

#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/station2_inspection.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"

namespace {

using scan_tracking::common::ScanDeviceKind;
using scan_tracking::flow_control::InspectionCloudSnapshot;
using scan_tracking::flow_control::InspectionQuota;
using scan_tracking::flow_control::InspectionSegmentCloud;
using scan_tracking::mech_eye::PointCloudFrame;

QString cloudPath(
    const QString& root,
    int pathId,
    ScanDeviceKind device,
    int segmentIndex)
{
    const bool arm = device == ScanDeviceKind::Arm;
    return QDir(root).filePath(
        QStringLiteral("%1/%2/Path%3_%4_cloud_%2.ply")
            .arg(arm ? QStringLiteral("arm") : QStringLiteral("telescopic"))
            .arg(segmentIndex)
            .arg(pathId)
            .arg(arm ? QStringLiteral("Arm") : QStringLiteral("Telescopic")));
}

bool appendDevice(
    const QString& root,
    int pathId,
    ScanDeviceKind device,
    int count,
    InspectionCloudSnapshot* snapshot)
{
    for (int index = 1; index <= count; ++index) {
        const QString path = cloudPath(root, pathId, device, index);
        PointCloudFrame frame;
        if (!scan_tracking::mech_eye::loadPointCloudFrameFromPly(path, &frame)) {
            std::fprintf(stderr, "Failed to load PLY: %s\n", qPrintable(path));
            return false;
        }

        InspectionSegmentCloud segment;
        segment.device = device;
        segment.localIndex = index;
        segment.captureOk = true;
        segment.xyz = std::move(frame.pointsXYZ);
        segment.pointCount = frame.pointCount;
        snapshot->segments.push_back(std::move(segment));

        std::printf(
            "Loaded %s %d/%d: %d points\n",
            device == ScanDeviceKind::Arm ? "arm" : "telescopic",
            index,
            count,
            frame.pointCount);
        std::fflush(stdout);
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    if (argc != 2) {
        std::fprintf(
            stderr,
            "Usage: scan_tracking_path1_offline <path_1 or path_5 directory>\n");
        return 1;
    }

    const QString root = QDir::cleanPath(QString::fromLocal8Bit(argv[1]));
    if (!QFileInfo(root).isDir()) {
        std::fprintf(stderr, "Path1 directory does not exist: %s\n", qPrintable(root));
        return 2;
    }

    const QString directoryName = QFileInfo(root).fileName();
    const int pathId = directoryName.compare(QStringLiteral("path_5"), Qt::CaseInsensitive) == 0
        ? 5
        : 1;
    const int armCount = pathId == 5 ? 18 : 12;
    const int telescopicCount = pathId == 5 ? 18 : 7;

    InspectionCloudSnapshot snapshot;
    snapshot.runCaptureRoot = root;
    if (!appendDevice(root, pathId, ScanDeviceKind::Arm, armCount, &snapshot) ||
        !appendDevice(
            root, pathId, ScanDeviceKind::Telescopic, telescopicCount, &snapshot)) {
        return 3;
    }

    InspectionQuota quota;
    quota.expectedArmCount = armCount;
    quota.expectedTelescopicCount = telescopicCount;
    quota.pathId = pathId;
    quota.pathName = pathId == 5 ? QStringLiteral("ring_weld") : QStringLiteral("straight_weld");
    quota.algorithm = QStringLiteral("weld_section");

    const auto result = scan_tracking::flow_control::evaluateStation2Inspection(
        snapshot, 0, quota);
    const QByteArray text =
        scan_tracking::flow_control::formatInspectionResultTextBlock(result).toUtf8();
    std::fwrite(text.constData(), 1, static_cast<std::size_t>(text.size()), stdout);
    std::fflush(stdout);
    return result.resultCode == 1 ? 0 : 4;
}
