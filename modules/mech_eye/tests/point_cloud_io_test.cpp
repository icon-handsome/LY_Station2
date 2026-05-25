#include "scan_tracking/common/capture_cache_paths.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"

#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>
#include <QtTest/QtTest>

using namespace scan_tracking::mech_eye;

class PointCloudIoTest : public QObject {
    Q_OBJECT

private slots:
    void roundTripSaveLoad();
    void plyPathUsesPointcloudSubdir();
};

void PointCloudIoTest::roundTripSaveLoad()
{
    PointCloudFrame frame;
    frame.pointsXYZ = std::make_shared<std::vector<float>>();
    frame.normalsXYZ = std::make_shared<std::vector<float>>();
    frame.pointsXYZ->insert(frame.pointsXYZ->end(), {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
    });
    frame.normalsXYZ->insert(frame.normalsXYZ->end(), {
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,
    });
    frame.pointCount = 3;
    frame.width = 3;
    frame.height = 1;
    frame.frameId = 42;

    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());

    const QString ts = QStringLiteral("20260525_120000_000");
    const QString plyPath = buildSegmentPlyPath(tempDir.path(), 1, 100u, ts);
    QVERIFY(plyPath.contains(QStringLiteral("pointcloud")));
    QVERIFY(!plyPath.isEmpty());
    QVERIFY(savePointCloudFrameToPly(frame, plyPath));
    QVERIFY(QFile::exists(plyPath));

    releasePointCloudFrameBuffers(&frame);
    QVERIFY(!frame.isValid());

    PointCloudFrame loaded;
    QVERIFY(loadPointCloudFrameFromPly(plyPath, &loaded));
    QCOMPARE(loaded.pointCount, 3);
    QVERIFY(loaded.isValid());
    QVERIFY(loaded.hasNormals());
    QCOMPARE(loaded.pointsXYZ->size(), static_cast<std::size_t>(9));
}

void PointCloudIoTest::plyPathUsesPointcloudSubdir()
{
    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());

    const QString root = scan_tracking::common::captureCachePointCloudDir(tempDir.path());
    QVERIFY(root.endsWith(QStringLiteral("pointcloud")));
}

QTEST_MAIN(PointCloudIoTest)
#include "point_cloud_io_test.moc"
