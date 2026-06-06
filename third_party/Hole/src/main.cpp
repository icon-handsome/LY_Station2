#include "HeadMeasure\Config.h"
#include "HeadMeasure/MeasurePipeline.h"

#include <exception>
#include <iomanip>
#include <iostream>

namespace {

void printResult(const hm::MeasureResult& r) 
{
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\nmeasure_result\n";
    std::cout << "inner_diameter_mm=" << r.innerDiameterMm << '\n';
    std::cout << "inner_circumference_mm=" << r.innerCircumferenceMm << '\n';
    std::cout << "roundness_tolerance_mm=" << r.roundnessToleranceMm << '\n';
    std::cout << "straight_side_deviation_mm=" << r.straightSideSlopeDeg << '\n';
    std::cout << "straight_side_height_mm=" << r.straightSideHeightMm << '\n';
    std::cout << "opening name=" << r.opening.name
              << " center_to_inner_wall_mm=" << r.opening.centerToInnerWallDistanceMm
              << " axis_to_head_axis_angle_deg=" << r.opening.axisToHeadAxisAngleDeg
              << '\n';
}

}  // namespace

int main(int argc, char** argv)
{
   const hm::MeasureConfig config = hm::loadConfig("D:/1 自研/15 兰铀算法/测量算法/柱面和开孔测量/config/sample_config.json");
   hm::MeasurePipeline pipeline(config);
   const hm::MeasureResult result = pipeline.run();
   printResult(result);
   return 0;
}

