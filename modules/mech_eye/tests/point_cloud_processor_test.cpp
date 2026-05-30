#include <QtTest/QTest>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"

using namespace scan_tracking::mech_eye;
using namespace scan_tracking::common;

class PointCloudProcessorTest : public QObject {
    Q_OBJECT

private slots:
    void filtersDepthOutliersAndDownsample();
    void passthroughWhenDisabled();
    void transformsPointCloudWithPoseMatrices();
};

void PointCloudProcessorTest::filtersDepthOutliersAndDownsample()
{
    PointCloudFrame input;
    input.width = 100;
    input.height = 1;
    input.pointCount = 100;
    input.frameId = 1;
    input.pointsXYZ = std::make_shared<std::vector<float>>();
    input.pointsXYZ->reserve(300);

    for (int i = 0; i < 95; ++i) {
        input.pointsXYZ->push_back(static_cast<float>(i));
        input.pointsXYZ->push_back(0.0f);
        input.pointsXYZ->push_back(500.0f);
    }
    for (int i = 0; i < 5; ++i) {
        input.pointsXYZ->push_back(0.0f);
        input.pointsXYZ->push_back(0.0f);
        input.pointsXYZ->push_back(5000.0f);
    }

    PointCloudProcessingConfig config;
    config.enabled = true;
    config.depthMinMm = 400.0f;
    config.depthMaxMm = 600.0f;
    config.outlierRemovalEnabled = true;
    config.outlierMeanK = 10;
    config.outlierStddevMul = 2.0f;
    config.smoothingEnabled = false;
    config.downsampleEnabled = true;
    config.voxelLeafSizeMm = 50.0f;
    config.minPointsAfterProcessing = 1;

    PointCloudFrame output;
    PointCloudProcessReport report;
    QVERIFY(processPointCloudFrame(input, config, &output, &report));
    QVERIFY(output.isValid());
    QVERIFY(output.pointCount < input.pointCount);
    QVERIFY(report.outputPointCount > 0);

    for (int index = 0; index < output.pointCount; ++index) {
        const float z = (*output.pointsXYZ)[static_cast<std::size_t>(index * 3 + 2)];
        QVERIFY(z >= 400.0f);
        QVERIFY(z <= 600.0f);
    }
}

void PointCloudProcessorTest::passthroughWhenDisabled()
{
    PointCloudFrame input;
    input.pointCount = 3;
    input.width = 3;
    input.height = 1;
    input.pointsXYZ = std::make_shared<std::vector<float>>(std::vector<float>{
        1.0f, 2.0f, 3.0f,
        4.0f, 5.0f, 6.0f,
        7.0f, 8.0f, 9.0f,
    });

    PointCloudProcessingConfig config;
    config.enabled = false;

    PointCloudFrame output;
    QVERIFY(processPointCloudFrame(input, config, &output));
    QCOMPARE(output.pointCount, input.pointCount);
    QCOMPARE(*output.pointsXYZ, *input.pointsXYZ);
}

void PointCloudProcessorTest::transformsPointCloudWithPoseMatrices()
{
    PointCloudFrame input;
    input.pointCount = 1;
    input.width = 1;
    input.height = 1;
    input.pointsXYZ = std::make_shared<std::vector<float>>(std::vector<float>{1.0f, 2.0f, 3.0f});

    const std::array<float, 16> identity = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };
    std::array<float, 16> translation = identity;
    translation[3] = 10.0f;

    PointCloudFrame output;
    QString message;
    QVERIFY(transformPointCloudFrame(input, identity, translation, &output, &message));
    QCOMPARE(output.pointCount, 1);
    QCOMPARE((*output.pointsXYZ)[0], 11.0f);
    QCOMPARE((*output.pointsXYZ)[1], 2.0f);
    QCOMPARE((*output.pointsXYZ)[2], 3.0f);
}

QTEST_MAIN(PointCloudProcessorTest)
#include "point_cloud_processor_test.moc"
