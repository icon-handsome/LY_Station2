#ifndef CONTAINER_TOTAL_LENGTH_API_H_
#define CONTAINER_TOTAL_LENGTH_API_H_

#include "Measurement.h"

// Public DLL facade. All methods use caller-owned result and error buffers.
class CONTAINER_TOTAL_LENGTH_API ContainerTotalLength
{
public:
    ContainerTotalLength();

    // Loads configPath, its input/template clouds, and performs the measurement.
    // errorBuffer may be null. The message is truncated to errorBufferSize - 1.
    bool Measure(const char* configPath,
                 MeasurementResult* result,
                 char* errorBuffer,
                 int errorBufferSize) const;

    // Runs the algorithm on already-loaded clouds.
    bool MeasureClouds(CloudConstPtr rawInput,
                       CloudConstPtr templateCloud,
                       const Config& config,
                       MeasurementResult* result,
                       char* errorBuffer,
                       int errorBufferSize) const;
};

#endif
