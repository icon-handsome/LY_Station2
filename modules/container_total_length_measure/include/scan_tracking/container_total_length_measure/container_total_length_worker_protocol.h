#pragma once

/// File-based job protocol between ContainerTotalLengthService (host) and
/// container-total-length-worker.exe (child process that loads ContainerTotalLength.dll).
///
/// Work directory layout:
///   request.txt   — job description (UTF-8 key=value)
///   cloud.bin     — float32 XYZ interleaved (point_count * 3)
///   result.txt    — written by worker after ctl_measure (crash-safe before teardown)
///
/// request.txt keys:
///   config=<absolute path to config.ini>
///   cloud=<filename>          — relative to workdir (usually cloud.bin)
///   point_count=<N>
///
/// result.txt keys (subset used by host):
///   status=<ctl_status int>
///   message=<text>
///   valid=<0|1>
///   length_mm=<double>
///   left_end_position=<double>
///   right_end_position=<double>
///   icp_fitness=<double>
///   fitted_radius_mm=<float>
///   cylinder_point_x/y/z=<float>
///   cylinder_axis_x/y/z=<float>
///   icp_converged=<0|1>
///   input_point_count=<int>
///
/// Host spawns one short-lived worker per measure() so DLL-side leaks (if any)
/// are reclaimed when the child process exits. Worker + DLL + PCL live under a
/// private directory so they do not collide with other algorithm runtimes.

namespace scan_tracking::container_total_length_measure::worker_protocol {

constexpr const char* kRequestFileName = "request.txt";
constexpr const char* kResultFileName = "result.txt";
constexpr const char* kCloudFileName = "cloud.bin";
constexpr const char* kWorkerExeName = "container-total-length-worker.exe";
/// Private runtime dir beside the host exe (DLL + PCL 1.12 live here).
constexpr const char* kWorkerRelPath = "workers/container_total_length/container-total-length-worker.exe";
constexpr const char* kWorkerRelDir = "workers/container_total_length";

/// Default host wait (full raw cloud + ICP can be slow).
constexpr int kDefaultTimeoutMs = 300000;

}  // namespace scan_tracking::container_total_length_measure::worker_protocol
