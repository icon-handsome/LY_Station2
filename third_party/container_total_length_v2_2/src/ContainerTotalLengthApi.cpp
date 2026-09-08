#include "ContainerTotalLengthApi.h"

#include "Measurement.h"
#include "PointCloudIO.h"

#include <cmath>
#include <cstring>
#include <exception>
#include <mutex>
#include <string>

struct ctl_context {
    Config config;
    CloudPtr templateCloud;
    std::mutex mutex;
};

namespace {

void SetMessage(char* message, size_t size, const std::string& value)
{
    if (!message || size == 0) return;
    const size_t count = value.size() < size - 1 ? value.size() : size - 1;
    std::memcpy(message, value.data(), count);
    message[count] = '\0';
}

std::string DirectoryOf(const std::string& path)
{
    const size_t slash = path.find_last_of("/\\");
    return slash == std::string::npos ? "." : path.substr(0, slash);
}

bool IsAbsolute(const std::string& path)
{
    return !path.empty() && (path[0] == '/' || path[0] == '\\' ||
        (path.size() > 1 && path[1] == ':'));
}

std::string JoinPath(const std::string& directory, const std::string& path)
{
    if (path.empty() || IsAbsolute(path) || directory.empty() || directory == ".") return path;
    return directory + "/" + path;
}

void ResetResult(ctl_result* result)
{
    if (result) std::memset(result, 0, sizeof(*result));
}

} // namespace

extern "C" ctl_status ctl_create_from_ini(const char* config_path,
    ctl_context** out_context, char* message, size_t message_size)
{
    if (!config_path || !*config_path || !out_context) {
        SetMessage(message, message_size, "config_path and out_context are required");
        return CTL_ERR_INVALID_ARGUMENT;
    }
    *out_context = nullptr;
    try {
        ctl_context* context = new ctl_context;
        std::string error;
        if (!LoadIniConfig(config_path, &context->config, &error)) {
            delete context;
            SetMessage(message, message_size, error);
            return CTL_ERR_CONFIG;
        }
        const std::string templatePath = JoinPath(
            DirectoryOf(config_path), context->config.templateCloudPath);
        context->templateCloud = LoadCloud(templatePath, &error);
        if (!context->templateCloud || context->templateCloud->empty()) {
            delete context;
            SetMessage(message, message_size, error.empty() ? "template cloud is empty" : error);
            return CTL_ERR_TEMPLATE;
        }
        *out_context = context;
        SetMessage(message, message_size, "ok");
        return CTL_OK;
    } catch (const std::exception& exception) {
        SetMessage(message, message_size, exception.what());
        return CTL_ERR_CONFIG;
    } catch (...) {
        SetMessage(message, message_size, "unknown exception while creating context");
        return CTL_ERR_CONFIG;
    }
}

extern "C" ctl_status ctl_measure(ctl_context* context, const float* xyz,
    size_t point_count, ctl_result* result, char* message, size_t message_size)
{
    ResetResult(result);
    if (!context) return CTL_ERR_NOT_INITIALIZED;
    if (!xyz || point_count == 0 || !result) {
        SetMessage(message, message_size, "xyz, point_count and result are required");
        return CTL_ERR_INVALID_ARGUMENT;
    }
    try {
        std::lock_guard<std::mutex> lock(context->mutex);
        CloudPtr input(new CloudT);
        input->reserve(point_count);
        for (size_t i = 0; i < point_count; ++i) {
            const float x = xyz[i * 3];
            const float y = xyz[i * 3 + 1];
            const float z = xyz[i * 3 + 2];
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            if (context->config.cropInputCloud &&
                (x < context->config.cropMinPoint.x() || x > context->config.cropMaxPoint.x() ||
                 y < context->config.cropMinPoint.y() || y > context->config.cropMaxPoint.y() ||
                 z < context->config.cropMinPoint.z() || z > context->config.cropMaxPoint.z())) continue;
            PointT point;
            point.x = x; point.y = y; point.z = z;
            input->push_back(point);
        }
        input->width = static_cast<unsigned int>(input->size());
        input->height = 1;
        input->is_dense = false;
        if (input->empty()) {
            SetMessage(message, message_size, "xyz contains no finite points");
            return CTL_ERR_INPUT;
        }

        MeasurementResult measurement;
        std::string error;
        if (!MeasureContainerLength(input, context->templateCloud, context->config,
                                    &measurement, &error)) {
            SetMessage(message, message_size, error.empty() ? "length measurement failed" : error);
            return CTL_ERR_MEASURE;
        }
        result->length_mm = measurement.length;
        result->left_end_position = measurement.ends[0].refinedPosition;
        result->right_end_position = measurement.ends[1].refinedPosition;
        result->icp_fitness = measurement.icpFitness;
        result->cylinder_point_x = measurement.cylinder.point.x();
        result->cylinder_point_y = measurement.cylinder.point.y();
        result->cylinder_point_z = measurement.cylinder.point.z();
        result->cylinder_axis_x = measurement.cylinder.axis.x();
        result->cylinder_axis_y = measurement.cylinder.axis.y();
        result->cylinder_axis_z = measurement.cylinder.axis.z();
        result->icp_converged = measurement.icpConverged ? 1 : 0;
        result->input_point_count = static_cast<int>(input->size());
        result->valid = measurement.length > 0.0 ? 1 : 0;
        SetMessage(message, message_size, "ok");
        return result->valid ? CTL_OK : CTL_ERR_MEASURE;
    } catch (const std::exception& exception) {
        SetMessage(message, message_size, exception.what());
        return CTL_ERR_MEASURE;
    } catch (...) {
        SetMessage(message, message_size, "unknown exception while measuring");
        return CTL_ERR_MEASURE;
    }
}

extern "C" void ctl_destroy(ctl_context* context) { delete context; }

extern "C" const char* ctl_status_string(ctl_status status)
{
    switch (status) {
    case CTL_OK: return "ok";
    case CTL_ERR_INVALID_ARGUMENT: return "invalid argument";
    case CTL_ERR_CONFIG: return "config error";
    case CTL_ERR_TEMPLATE: return "template error";
    case CTL_ERR_INPUT: return "input error";
    case CTL_ERR_MEASURE: return "measurement error";
    case CTL_ERR_NOT_INITIALIZED: return "not initialized";
    default: return "unknown error";
    }
}
