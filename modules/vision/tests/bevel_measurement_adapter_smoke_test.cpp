#include "scan_tracking/vision/bevel_measurement_adapter.h"

#include <cmath>
#include <iostream>

#include <QCoreApplication>

namespace {

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        return false;
    }
    return true;
}

scan_tracking::mech_eye::PointCloudFrame makeFrame(int pointCount)
{
    scan_tracking::mech_eye::PointCloudFrame frame;
    if (pointCount <= 0) {
        return frame;
    }

    auto points = std::make_shared<std::vector<float>>();
    points->reserve(static_cast<std::size_t>(pointCount * 3));
    for (int index = 0; index < pointCount; ++index) {
        points->push_back(static_cast<float>(index));
        points->push_back(static_cast<float>(index + 1));
        points->push_back(static_cast<float>(index + 2));
    }

    frame.pointsXYZ = points;
    frame.pointCount = pointCount;
    frame.width = pointCount;
    frame.height = 1;
    return frame;
}

bool testToPclPointCloud()
{
    const auto frame = makeFrame(4);
    const auto cloud = scan_tracking::vision::bevel::toPclPointCloud(frame);

    bool ok = true;
    ok &= expectTrue(cloud != nullptr, "cloud pointer should not be null");
    ok &= expectTrue(cloud->size() == 4, "cloud should contain four points");
    ok &= expectTrue(cloud->width == 4 && cloud->height == 1, "cloud layout should be Nx1");
    return ok;
}

bool testBuildBevelSolveOptions()
{
    scan_tracking::common::BevelRecipe recipe;
    recipe.active = true;
    recipe.bevelType = 1;
    recipe.angleDeg = 30.0f;
    recipe.lengthMm = 6.0f;

    const auto options =
        scan_tracking::vision::bevel::buildBevelSolveOptions(recipe, 2.0f, 1.0f);

    bool ok = true;
    ok &= expectTrue(options.forcedBevelType == 1, "forced bevel type should match recipe");
    ok &= expectTrue(options.overrideStandard, "standard override should be enabled");
    ok &= expectTrue(std::abs(options.standardAngleMinDeg - 28.0) < 1e-6,
                     "angle min should be target minus tolerance");
    ok &= expectTrue(std::abs(options.standardAngleMaxDeg - 32.0) < 1e-6,
                     "angle max should be target plus tolerance");
    ok &= expectTrue(std::abs(options.standardLengthMin - 5.0) < 1e-6,
                     "length min should be target minus tolerance");
    ok &= expectTrue(std::abs(options.standardLengthMax - 7.0) < 1e-6,
                     "length max should be target plus tolerance");
    return ok;
}

bool testRejectsEmptyCloud()
{
    scan_tracking::common::BevelRecipe recipe;
    recipe.active = true;
    recipe.bevelType = 0;
    recipe.angleDeg = 45.0f;
    recipe.lengthMm = 1.0f;

    const auto frame = makeFrame(0);
    const auto result =
        scan_tracking::vision::bevel::runBevelMeasurement(frame, recipe, 2.0f, 1.0f);

    bool ok = true;
    ok &= expectTrue(!result.invoked, "empty cloud should not invoke Po_Kou");
    ok &= expectTrue(!result.ok, "empty cloud should fail");
    ok &= expectTrue(result.message.contains(QStringLiteral("缺少有效输入点云")),
                     "empty cloud should explain missing input");
    return ok;
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    bool ok = true;
    ok &= testToPclPointCloud();
    ok &= testBuildBevelSolveOptions();
    ok &= testRejectsEmptyCloud();

    if (!ok) {
        return 1;
    }

    std::cout << "Bevel adapter smoke tests passed\n";
    return 0;
}
