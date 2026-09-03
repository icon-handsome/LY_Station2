#pragma once

/// File-based job protocol between ThicknessMeasureV2Service (host) and
/// thickness-measure-v3-worker.exe (child process that loads ThicknessMeasureV3.dll).
///
/// Work directory layout:
///   request.txt          — job description (UTF-8 key=value)
///   pair_<i>_inner.bin   — float32 XYZ interleaved (point_count * 3)
///   pair_<i>_outer.bin
///   result.txt           — written by worker before tmv3_destroy (crash-safe)
///
/// request.txt keys:
///   mode=pair|pairs_average
///   config=<absolute path to thickness_measurement.ini>
///   pair_count=<N>
///   inner_<i>=<filename>   inner_<i>_count=<points>
///   outer_<i>=<filename>   outer_<i>_count=<points>
///
/// result.txt keys (subset used by host):
///   status=<tmv3_status int>
///   message=<text>
///   thickness_mm=<double>
///   pair_count=<size_t>
///   success_count=<size_t>
///   valid=<0|1>
///   method=... section_count=... (pair mode)
///   inner_outer_icp_fitness=... outer_template_icp_fitness=... (pair mode)

namespace scan_tracking::thickness_measure_v2::worker_protocol {

constexpr const char* kRequestFileName = "request.txt";
constexpr const char* kResultFileName = "result.txt";
constexpr const char* kWorkerExeName = "thickness-measure-v3-worker.exe";

/// Default host wait for worker (PCL/ONNX + full clouds can be slow).
constexpr int kDefaultTimeoutMs = 300000;

}  // namespace scan_tracking::thickness_measure_v2::worker_protocol
