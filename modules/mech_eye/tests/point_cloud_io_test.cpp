#include "scan_tracking/common/capture_cache_paths.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"

#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>
#include <QtTest/QtTest>

#include <cmath>
#include <limits>

using namespace scan_tracking::mech_eye;

class PointCloudIoTest : public QObject {
    Q_OBJECT

private slots:
    void roundTripSaveLoad();
    void binaryPlyPreservesNanPoints();
    void plyPathUsesPathDevicePointLayout();
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

    const QString plyPath = buildSegmentPlyPath(tempDir.path(), 1, QStringLiteral("arm"), 1);
    QVERIFY(plyPath.contains(QStringLiteral("path_1")));
    QVERIFY(plyPath.contains(QStringLiteral("arm")));
    QVERIFY(plyPath.contains(QStringLiteral("/1/")) || plyPath.contains(QStringLiteral("\\1\\")));
    QVERIFY(plyPath.endsWith(QStringLiteral("cloud.ply")));
    QVERIFY(!plyPath.isEmpty());
    QVERIFY(savePointCloudFrameToPly(frame, plyPath));
    QVERIFY(QFile::exists(plyPath));

    releasePointCloudFrameBuffers(&frame);
    QVERIFY(!frame.isValid());

    PointCloudFrame loaded;
    QVERIFY(loadPointCloudFrameFromPly(plyPath, &loaded));
    QCOMPARE(loaded.pointCount, 3);
    QVERIFY(loaded.isValid());
    QVERIFY(!loaded.hasNormals());
    QCOMPARE(loaded.pointsXYZ->size(), static_cast<std::size_t>(9));

    QFile plyFile(plyPath);
    QVERIFY(plyFile.open(QIODevice::ReadOnly));
    const QByteArray header = plyFile.read(256);
    QVERIFY(header.contains("format binary_little_endian 1.0"));
    QVERIFY(!header.contains("property float nx"));
}

void PointCloudIoTest::binaryPlyPreservesNanPoints()
{
    PointCloudFrame frame;
    frame.pointsXYZ = std::make_shared<std::vector<float>>();
    frame.normalsXYZ = std::make_shared<std::vector<float>>();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    frame.pointsXYZ->insert(frame.pointsXYZ->end(), {
        1.0f, 2.0f, 3.0f,
        nan, nan, nan,
    });
    frame.normalsXYZ->insert(frame.normalsXYZ->end(), {
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,
    });
    frame.pointCount = 2;
    frame.width = 2;
    frame.height = 1;

    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());

    const QString plyPath =
        buildSegmentPlyPath(tempDir.path(), 5, QStringLiteral("telescopic"), 2);
    QVERIFY(plyPath.contains(QStringLiteral("path_5")));
    QVERIFY(plyPath.contains(QStringLiteral("telescopic")));
    QVERIFY(savePointCloudFrameToPly(frame, plyPath));

    PointCloudFrame loaded;
    QVERIFY(loadPointCloudFrameFromPly(plyPath, &loaded));
    QCOMPARE(loaded.pointCount, 2);
    QCOMPARE(loaded.pointsXYZ->size(), static_cast<std::size_t>(6));
    QVERIFY(std::isfinite((*loaded.pointsXYZ)[0]));
    QVERIFY(!std::isfinite((*loaded.pointsXYZ)[3]));
}

void PointCloudIoTest::plyPathUsesPathDevicePointLayout()
{
    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());

    const QString armPath = buildSegmentPlyPath(tempDir.path(), 1, QStringLiteral("arm"), 3);
    const QString telPath =
        buildSegmentPlyPath(tempDir.path(), 1, QStringLiteral("telescopic"), 3);
    QVERIFY(armPath.contains(QStringLiteral("path_1")));
    QVERIFY(armPath.contains(QStringLiteral("arm")));
    QVERIFY(telPath.contains(QStringLiteral("telescopic")));
    QCOMPARE(QFileInfo(armPath).fileName(), QStringLiteral("cloud.ply"));
    QCOMPARE(QFileInfo(telPath).fileName(), QStringLiteral("cloud.ply"));
    QCOMPARE(QFileInfo(armPath).dir().dirName(), QStringLiteral("3"));
    QCOMPARE(QFileInfo(telPath).dir().dirName(), QStringLiteral("3"));
}

QTEST_MAIN(PointCloudIoTest)
#include "point_cloud_io_test.moc"
