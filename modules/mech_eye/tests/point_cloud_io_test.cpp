#include "scan_tracking/common/capture_cache_paths.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"

#include <QCoreApplication>
#include <QFile>
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
    void plyPathUsesMech3dSubdir();
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
    QVERIFY(plyPath.contains(QStringLiteral("mech_3d")));
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

    const QString plyPath = buildSegmentPlyPath(tempDir.path(), 2, 200u, QStringLiteral("20260629_120000_000"));
    QVERIFY(savePointCloudFrameToPly(frame, plyPath));

    PointCloudFrame loaded;
    QVERIFY(loadPointCloudFrameFromPly(plyPath, &loaded));
    QCOMPARE(loaded.pointCount, 2);
    QCOMPARE(loaded.pointsXYZ->size(), static_cast<std::size_t>(6));
    QVERIFY(std::isfinite((*loaded.pointsXYZ)[0]));
    QVERIFY(!std::isfinite((*loaded.pointsXYZ)[3]));
}

void PointCloudIoTest::plyPathUsesMech3dSubdir()
{
    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());

    const QString root = scan_tracking::common::captureCacheMech3DDir(tempDir.path());
    QVERIFY(root.endsWith(QStringLiteral("mech_3d")));
}

QTEST_MAIN(PointCloudIoTest)
#include "point_cloud_io_test.moc"
