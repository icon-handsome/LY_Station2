#pragma once

#include <QtCore/QString>

#include <cstdint>
#include <vector>

namespace scan_tracking {
namespace orbbec_gemini {

struct OrbbecDepthFrameView {
    const uint16_t* data = nullptr;
    int width = 0;
    int height = 0;
    float valueScale = 1.0f;
};

struct OrbbecPointView {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

bool saveDepthFramePngs(
    const OrbbecDepthFrameView& frame,
    const QString& rawPngPath,
    const QString& previewPngPath,
    int* validPixelCountOut = nullptr);

bool savePointCloudPly(
    const std::vector<OrbbecPointView>& points,
    const QString& plyPath);

QString buildOrbbecCaptureBaseName(quint64 requestId, const QString& timestamp);

QString buildOrbbecCapturePaths(
    const QString& cacheRoot,
    quint64 requestId,
    const QString& timestamp,
    QString* depthRawPngPath,
    QString* depthPreviewPngPath,
    QString* pointCloudPlyPath);

}  // namespace orbbec_gemini
}  // namespace scan_tracking
