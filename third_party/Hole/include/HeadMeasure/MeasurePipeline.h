#pragma once

#include "HeadMeasure/Types.h"

namespace hm {

class MeasurePipeline {
public:
    explicit MeasurePipeline(MeasureConfig config);
    MeasureResult run();
    MeasureResult runWithScanCloud(const CloudConstPtr& rawScan);

private:
    MeasureResult runPipelineWithPreprocessedScan(const CloudConstPtr& scan);

    MeasureConfig config_;
};

}  // namespace hm
