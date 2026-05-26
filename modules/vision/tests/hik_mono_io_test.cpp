#include "scan_tracking/vision/hik_mono_io.h"

#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>
#include <QtTest/QtTest>

using namespace scan_tracking::vision;

class HikMonoIoTest : public QObject {
    Q_OBJECT

private slots:
    void roundTripSaveBmp();
};

void HikMonoIoTest::roundTripSaveBmp()
{
    HikMonoFrame frame;
    frame.width = 2;
    frame.height = 2;
    frame.stride = 2;
    frame.pixels = std::make_shared<std::vector<std::uint8_t>>(std::initializer_list<std::uint8_t>{
        10, 20,
        30, 40,
    });

    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());

    const QString ts = QStringLiteral("20260525_120000_000");
    const QString path = buildSegmentHikMonoPath(tempDir.path(), 2, 99u, QStringLiteral("hikA"), ts);
    QVERIFY(path.contains(QStringLiteral("hik_mono")));
    QVERIFY(path.contains(QStringLiteral("camera_a")));
    QVERIFY(path.endsWith(QStringLiteral(".bmp")));
    QVERIFY(saveHikMonoFrameToBmp(frame, path));
    QVERIFY(QFile::exists(path));

    QFile file(path);
    QVERIFY(file.open(QIODevice::ReadOnly));
    const QByteArray content = file.readAll();
    QVERIFY(content.size() >= 2);
    QCOMPARE(content.at(0), char('B'));
    QCOMPARE(content.at(1), char('M'));
}

QTEST_MAIN(HikMonoIoTest)
#include "hik_mono_io_test.moc"
